`timescale 1ns / 1ps

// =====================================================================
// u02_pkg (bản thu gọn ĐA2, 25/11/2026)
//  - 2 frame buffer (ping-pong), chỉ số buffer 1 bit (fb_idx_t)
//  - Bỏ timestamp/deadline: descriptor 58 bit, completion 54 bit
//  - Completion chỉ cho ROI đã được dispatch: CPL_OK hoặc CPL_BUS_ERR
//    (ROI STALE / INVALID được đếm trong regfile, không vào hàng completion)
// =====================================================================
package u02_pkg;

    // -------------------------------------------------------------
    // 1. Tham số xung nhịp & Khung hình
    // -------------------------------------------------------------
    // Tần số ghi dạng int (kHz) để công cụ tổng hợp không cảnh báo kiểu real
    localparam int CLK_SYS_FREQ_KHZ  = 150_000;   // clk_sys chung cho U01, U02, AXI, APB
    localparam int CLK_ISP_FREQ_KHZ  = 25_175;    // clk_isp ingress camera VGA 640x480

    localparam int FRAME_WIDTH       = 640;
    localparam int FRAME_HEIGHT      = 480;
    localparam int PIXEL_BYTES       = 3;         // RGB888: 3 Bytes / Pixel
    localparam int FRAME_STRIDE_B    = 1920;      // 640 px * 3 B/px = 1920 Bytes (15 * 128B burst align)
    localparam int FRAME_SIZE_B      = 921600;    // 1920 * 480 = 921,600 Bytes / Khung

    // Địa chỉ cơ sở 2 Frame Buffer (ping-pong) trong DDR (mỗi khe 1 MB)
    localparam int          NUM_FB        = 2;
    localparam int          FB_IDX_W      = 1;
    typedef logic [FB_IDX_W-1:0] fb_idx_t;
    localparam logic [31:0] FB0_BASE_ADDR = 32'h8000_0000;
    localparam logic [31:0] FB1_BASE_ADDR = 32'h8010_0000;

    // Trạng thái mỗi frame buffer (frame_buf_mgr)
    typedef enum logic [1:0] {
        FB_FREE    = 2'd0,
        FB_WRITING = 2'd1,
        FB_READY   = 2'd2,   // chỉ vào READY sau BRESP cuối cùng của khung
        FB_READING = 2'd3
    } fb_state_t;

    // Giới hạn kích thước ROI
    localparam int ROI_MIN_DIM       = 16;
    localparam int ROI_MAX_DIM       = 128;
    localparam int MAX_ROI_BYTES     = 49152;     // 128 * 128 * 3 = 49,152 Bytes (Quyết định bộ đệm U01)

    // Hàng đợi ROI và hàng completion
    localparam int ROI_QUEUE_DEPTH   = 16;

    // Giá trị chèn lề (Padding) & Magic Byte
    localparam logic [7:0] PAD_VAL_DEFAULT = 8'h00; // Đen = 0x00 (tương đương -128 sau lượng tử hóa)
    localparam logic [7:0] HEADER_MAGIC    = 8'hA5; // Magic Byte kiểm tra tính toàn vẹn gói
    localparam int         IBUF_N_DEFAULT  = 1;     // Số buffer đầu vào mặc định của U01

    // Độ rộng Bus giao tiếp
    localparam int AXI_ADDR_WIDTH    = 32;
    localparam int AXI_DATA_WIDTH    = 64;
    localparam int AXI_ID_WIDTH      = 1;         // 1 ID -> phản hồi đúng thứ tự
    localparam int MAX_BURST_BEATS   = 16;        // 16 beat x 8 B = 128 B
    localparam int MAX_OUTSTANDING   = 4;         // mỗi kênh (đọc / ghi)
    localparam int AXI_PAGE_BYTES    = 4096;      // burst không được vượt ranh giới 4 KB
    localparam int AXIS_DATA_WIDTH   = 64;
    localparam int AXIS_KEEP_WIDTH   = 8;

    // Nhánh ghi: khung được ghi bằng các burst đầy đủ 16 beat, căn lề 128 B
    localparam int BURST_BYTES       = MAX_BURST_BEATS * AXI_DATA_WIDTH / 8;   // 128
    localparam int FRAME_BURSTS      = FRAME_SIZE_B / BURST_BYTES;             // 7200 (vừa 13 bit)

    // -------------------------------------------------------------
    // 2. Định nghĩa cấu trúc Header Beat 64-bit (U02 -> U01)
    // -------------------------------------------------------------
    // Bố cục:
    // [15:0]  W         (16..128)
    // [31:16] H         (16..128)
    // [39:32] roi_id    (8-bit)
    // [47:40] frame_id  (8-bit)
    // [48]    has_pad   (ROI có phần tràn biên frame)
    // [50:49] dec_log2  (0 -> dec=1, 1 -> dec=2, 2 -> dec=4)
    // [55:51] reserved  (5'b0)
    // [63:56] 0xA5      (Magic byte)
    typedef struct packed {
        logic [7:0]  magic;      // [63:56]
        logic [4:0]  reserved;   // [55:51]
        logic [1:0]  dec_log2;   // [50:49]
        logic        has_pad;    // [48]
        logic [7:0]  frame_id;   // [47:40]
        logic [7:0]  roi_id;     // [39:32]
        logic [15:0] h;          // [31:16]
        logic [15:0] w;          // [15:0]
    } u02_header_t;

    // -------------------------------------------------------------
    // 3. Descriptor nhiệm vụ ROI (Lưu trong roi_desc_queue) - 58 bit
    // -------------------------------------------------------------
    // Không chứa base_addr: frame_buf_mgr đổi frame_id -> địa chỉ lúc dispatch;
    // nếu buffer của frame đó đã bị thu hồi, ROI bị bỏ (STALE, đếm trong regfile).
    typedef struct packed {
        logic signed [11:0]  roi_x;      // có dấu, cho phép tràn biên
        logic signed [11:0]  roi_y;
        logic [7:0]          roi_w;      // W ngõ ra (16..128)
        logic [7:0]          roi_h;      // H ngõ ra (16..128)
        logic [1:0]          dec_log2;   // 0..2; 3 = không hợp lệ
        logic [7:0]          roi_id;
        logic [7:0]          frame_id;
    } u02_roi_desc_t;

    // -------------------------------------------------------------
    // 4. Completion descriptor (hàng completion) - 54 bit
    // -------------------------------------------------------------
    // Chỉ ROI đã dispatch mới có completion.
    typedef enum logic [1:0] {
        CPL_OK      = 2'd0,
        CPL_BUS_ERR = 2'd1     // RRESP lỗi (SLVERR/DECERR) trong lúc đọc ROI
    } cpl_status_t;

    typedef struct packed {
        logic [7:0]          roi_id;
        logic [7:0]          frame_id;
        cpl_status_t         status;
        logic [19:0]         bytes_ddr;    // byte đọc thực tế trên AXI (= beat * 8, tối đa 184.320)
        logic [15:0]         bytes_stream; // byte payload phát sang U01 (W*H*3, tối đa 49.152)
    } u02_roi_cpl_t;

    // -------------------------------------------------------------
    // 5. Thông tin hàng (row_cmd_gen -> row_info FIFO -> axis_crop) - 91 bit
    // -------------------------------------------------------------
    // Mỗi hàng ngõ ra = lead_pad pixel pad + n_valid pixel đọc từ DDR
    //                 + trail_pad pixel pad;  lead + n_valid + trail = W.
    // Hàng nằm ngoài khung: lead_pad = W, n_valid = trail_pad = 0.
    // hdr chỉ có ý nghĩa ở hàng first (axis_crop phát header trước hàng đầu).
    typedef struct packed {
        u02_header_t  hdr;        // header beat của ROI
        logic         first;      // hàng ngõ ra đầu tiên của ROI
        logic         last;       // hàng ngõ ra cuối cùng (beat cuối mang tlast)
        logic [1:0]   dec_log2;   // bước lấy mẫu cột: lấy 1 pixel mỗi 2^dec_log2
        logic [7:0]   lead_pad;   // 0..128
        logic [7:0]   n_valid;    // 0..128
        logic [7:0]   trail_pad;  // 0..128
    } u02_row_info_t;

    // -------------------------------------------------------------
    // 6. Lệnh ghi (isp_ingress -> axi_master_wr)
    // -------------------------------------------------------------
    // Mỗi lệnh WR_DATA / WR_DATA_LAST đi kèm đúng 16 từ 64-bit trong FIFO
    // dữ liệu. WR_ABORT không kèm dữ liệu: báo hủy khung buf_idx (sau khi
    // các burst đã gửi trước đó nhận đủ BRESP thì phát abort_evt).
    typedef enum logic [1:0] {
        WR_DATA      = 2'd0,   // burst thường
        WR_DATA_LAST = 2'd1,   // burst cuối khung: BRESP về -> commit_evt
        WR_ABORT     = 2'd2    // hủy khung
    } wr_kind_t;

    typedef struct packed {
        logic [31:0] addr;       // địa chỉ burst (căn lề 128 B)
        fb_idx_t     buf_idx;    // frame buffer đích (0..NUM_FB-1)
        wr_kind_t    kind;
    } u02_wr_cmd_t;

    /* verilator lint_off UNUSEDSIGNAL */
    // -------------------------------------------------------------
    // 7. Hàm tiện ích
    // -------------------------------------------------------------
    // Kiểm tra descriptor lúc push; sai -> bỏ, bật cờ err_invalid, không vào hàng đợi
    function automatic logic roi_desc_valid(input u02_roi_desc_t d);
        return (d.roi_w >= 8'(ROI_MIN_DIM)) && (d.roi_w <= 8'(ROI_MAX_DIM)) &&
               (d.roi_h >= 8'(ROI_MIN_DIM)) && (d.roi_h <= 8'(ROI_MAX_DIM)) &&
               (d.dec_log2 != 2'd3);
    endfunction

    // Đóng gói header beat từ descriptor
    function automatic u02_header_t make_header(input u02_roi_desc_t d, input logic has_pad);
        u02_header_t h;
        h          = '0;
        h.magic    = HEADER_MAGIC;
        h.dec_log2 = d.dec_log2;
        h.has_pad  = has_pad;
        h.frame_id = d.frame_id;
        h.roi_id   = d.roi_id;
        h.h        = {8'd0, d.roi_h};
        h.w        = {8'd0, d.roi_w};
        return h;
    endfunction
    /* verilator lint_on UNUSEDSIGNAL */

    // Địa chỉ cơ sở của frame buffer idx (0..NUM_FB-1); dùng hàm thay vì
    // localparam mảng để tương thích cả Icarus, Verilator và công cụ tổng hợp
    function automatic logic [31:0] fb_base(input fb_idx_t idx);
        return idx[0] ? FB1_BASE_ADDR : FB0_BASE_ADDR;
    endfunction

endpackage
