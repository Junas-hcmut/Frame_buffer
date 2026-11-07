`timescale 1ns / 1ps

// =====================================================================
// page_splitter (burst legalizer)
//
// Nhận 1 yêu cầu đọc {addr, len} (thường là 1 dòng ROI) và bẻ thành các
// burst con, mỗi burst nằm trọn trong 1 khối căn lề BLK_BYTES (mặc định
// 128 B = 16 beat x 8 B). Vì BLK_BYTES chia hết 4096 nên không burst nào
// vượt ranh giới 4 KB của AXI.
//
//  - Mỗi chu kỳ phát tối đa 1 burst con; đầu ra đi qua thanh ghi.
//  - s_req_ready và m_req_* đều lấy từ flip-flop (không có đường tổ hợp
//    xuyên qua module theo chiều dữ liệu).
//  - m_req_arlen = số beat - 1, đã tính cả byte lệch đầu (addr[2:0]).
//  - eol / eof chỉ gắn vào burst con cuối cùng của yêu cầu.
//  - Yêu cầu len = 0: được nhận rồi bỏ, phát xung err_zero_len
//    (AXI không có burst rỗng). Upstream không nên sinh yêu cầu này.
//  - Giữa hai yêu cầu có 1 chu kỳ trống; không ảnh hưởng thông lượng vì
//    1 dòng ROI (<= 128 px) cần <= 128 chu kỳ để xuất ra, trong khi chỉ
//    tạo <= 5 burst con.
// =====================================================================
module page_splitter
    import u02_pkg::*;
#(
    parameter int ADDR_WIDTH = AXI_ADDR_WIDTH,                  // 32
    parameter int LEN_WIDTH  = 12,                              // tối đa 4095 B / yêu cầu
    parameter int BUS_BYTES  = AXI_DATA_WIDTH / 8,              // 8
    parameter int BLK_BYTES  = MAX_BURST_BEATS * BUS_BYTES      // 128
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // Upstream: yêu cầu đọc 1 dòng
    input  logic                   s_req_valid,
    output logic                   s_req_ready,
    input  logic [ADDR_WIDTH-1:0]  s_req_addr,
    input  logic [LEN_WIDTH-1:0]   s_req_len,
    input  logic [7:0]             s_req_roi_id,
    input  logic [7:0]             s_req_frame_id,
    input  logic                   s_req_eol,
    input  logic                   s_req_eof,

    // Downstream: burst con hợp lệ cho axi_master_rd
    output logic                   m_req_valid,
    input  logic                   m_req_ready,
    output logic [ADDR_WIDTH-1:0]  m_req_addr,
    output logic [LEN_WIDTH-1:0]   m_req_len,      // 1..BLK_BYTES byte
    output logic [7:0]             m_req_arlen,    // ARLEN = số beat - 1 (<= 15)
    output logic [7:0]             m_req_roi_id,
    output logic [7:0]             m_req_frame_id,
    output logic                   m_req_eol,
    output logic                   m_req_eof,
    output logic                   m_req_first,    // burst con đầu tiên của yêu cầu
    output logic                   m_req_last,     // burst con cuối cùng của yêu cầu
    output logic                   m_req_split,    // yêu cầu bị bẻ thành >= 2 burst con

    output logic                   err_zero_len    // xung: nhận yêu cầu len = 0
);

    localparam int BLK_W = $clog2(BLK_BYTES);   // 7
    localparam int OFF_W = $clog2(BUS_BYTES);   // 3

    // ---------------- Kiểm tra tham số lúc mô phỏng ----------------
    // synopsys translate_off
    initial begin
        if ((1 << BLK_W) != BLK_BYTES || BLK_BYTES > AXI_PAGE_BYTES || (AXI_PAGE_BYTES % BLK_BYTES) != 0)
            $fatal(1, "page_splitter: BLK_BYTES=%0d phai la luy thua cua 2 va chia het %0d", BLK_BYTES, AXI_PAGE_BYTES);
        if ((1 << OFF_W) != BUS_BYTES || BLK_BYTES < BUS_BYTES)
            $fatal(1, "page_splitter: BUS_BYTES=%0d khong hop le", BUS_BYTES);
        if (LEN_WIDTH <= BLK_W)
            $fatal(1, "page_splitter: LEN_WIDTH=%0d phai > %0d", LEN_WIDTH, BLK_W);
    end
    // synopsys translate_on

    // ---------------- Trạng thái bộ sinh burst ----------------
    logic                  gen_active;
    logic [ADDR_WIDTH-1:0] cur_addr;
    logic [LEN_WIDTH-1:0]  rem_len;
    logic [7:0]            roi_q, frame_q;
    logic                  eol_q, eof_q, first_q;

    // Số byte còn lại tới hết khối hiện tại: 1..BLK_BYTES
    wire [BLK_W:0]     to_blk  = (BLK_W+1)'(BLK_BYTES) - {1'b0, cur_addr[BLK_W-1:0]};
    wire               is_last = (rem_len <= LEN_WIDTH'(to_blk));
    wire [BLK_W:0]     chunk   = is_last ? rem_len[BLK_W:0] : to_blk;

    // Số beat = ceil((byte lệch đầu + chunk) / BUS_BYTES); luôn <= BLK_BYTES/BUS_BYTES
    wire [BLK_W+1:0]   span    = (BLK_W+2)'(cur_addr[OFF_W-1:0]) + (BLK_W+2)'(chunk) + (BLK_W+2)'(BUS_BYTES - 1);
    wire [BLK_W+1:0]   beats   = span >> OFF_W;

    // ---------------- Bắt tay ----------------
    wire load_ok  = !m_req_valid || m_req_ready;       // thanh ghi đầu ra trống hoặc sắp được lấy
    wire emit     = gen_active && load_ok;              // nạp 1 burst con vào thanh ghi đầu ra
    wire s_hs     = s_req_valid && s_req_ready;
    wire zero_len = (s_req_len == '0);

    assign s_req_ready = !gen_active;

    // ---------------- Điều khiển (có reset) ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gen_active   <= 1'b0;
            m_req_valid  <= 1'b0;
            err_zero_len <= 1'b0;
        end else begin
            if (s_hs && !zero_len)        gen_active <= 1'b1;
            else if (emit && is_last)     gen_active <= 1'b0;

            if (load_ok)                  m_req_valid <= gen_active;

            err_zero_len <= s_hs && zero_len;
        end
    end

    // ---------------- Datapath (không reset) ----------------
    always_ff @(posedge clk) begin
        if (s_hs) begin
            cur_addr <= s_req_addr;
            rem_len  <= s_req_len;
            roi_q    <= s_req_roi_id;
            frame_q  <= s_req_frame_id;
            eol_q    <= s_req_eol;
            eof_q    <= s_req_eof;
            first_q  <= 1'b1;
        end else if (emit) begin
            cur_addr <= cur_addr + ADDR_WIDTH'(chunk);
            rem_len  <= rem_len  - LEN_WIDTH'(chunk);
            first_q  <= 1'b0;
        end

        if (emit) begin
            m_req_addr     <= cur_addr;
            m_req_len      <= LEN_WIDTH'(chunk);
            m_req_arlen    <= 8'(beats - 1'b1);
            m_req_roi_id   <= roi_q;
            m_req_frame_id <= frame_q;
            m_req_eol      <= eol_q & is_last;
            m_req_eof      <= eof_q & is_last;
            m_req_first    <= first_q;
            m_req_last     <= is_last;
            m_req_split    <= !(first_q && is_last);
        end
    end

endmodule
