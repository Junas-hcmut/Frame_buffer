`timescale 1ns / 1ps

// Thanh ghi APB của U02 (cùng miền clk_sys với toàn bộ U02).
//
// Bản đồ thanh ghi (cửa sổ 4 KB, byte address):
//   0x000 CTRL      RW   [0] enable
//   0x004 STATUS    RO/W1C
//                        [0] busy (RO)        [1] done (W1C, sticky)
//                        [2] q_full (RO)      [3] q_empty (RO)
//                        [4] err_overflow (W1C, push khi hàng đợi đầy)
//                        [5] err_invalid  (W1C, descriptor sai W/H/dec)
//                        [12:8] q_count (RO)
//   0x008 IRQ_EN    RW   [1] done  [4] err_overflow  [5] err_invalid
//   0x00C PAD_VAL   RW   [7:0]   giá trị byte chèn lề (mặc định 0x00)
//   0x010 CAP       RO   [7:0] ROI_MIN  [15:8] ROI_MAX  [23:16] QUEUE_DEPTH  [31:24] VERSION
//   0x014 IBUF_N    RW   [3:0]   số bộ đệm đầu vào của U01 (credit ban đầu)
//   0x020 ROI_POS   RW   [11:0] x (có dấu)   [27:16] y (có dấu)
//   0x024 ROI_DIM   RW   [7:0]  W            [23:16] H
//   0x028 ROI_ID    RW   [7:0]  roi_id       [15:8] frame_id   [17:16] dec_log2
//   0x02C ROI_DL_LO RW   deadline[31:0]
//   0x030 ROI_DL_HI RW   deadline[47:32]
//   0x034 ROI_PUSH  WO   ghi bit0 = 1 -> kiểm tra & đẩy descriptor; đọc luôn ra 0
//   0x038 TS_LO     RO   timestamp[31:0]  (đọc TS_LO chốt TS_HI để đọc 48 bit nhất quán)
//   0x03C TS_HI     RO   timestamp[47:32] đã chốt
// Địa chỉ ngoài bản đồ: PSLVERR = 1, PRDATA = 0.
module regfile
    import u02_pkg::*;
#(
    parameter int PADDR_W = 12
)(
    input  logic                         clk,
    input  logic                         rst_n,

    // APB4 slave
    input  logic [PADDR_W-1:0]           paddr,
    input  logic                         psel,
    input  logic                         penable,
    input  logic                         pwrite,
    input  logic [31:0]                  pwdata,
    output logic [31:0]                  prdata,
    output logic                         pready,
    output logic                         pslverr,

    // Cấu hình
    output logic                         cfg_enable,
    output logic [7:0]                   cfg_pad_val,
    output logic [3:0]                   cfg_ibuf_n,

    // Đẩy descriptor sang roi_desc_queue
    output logic                         q_push,
    output u02_roi_desc_t                q_desc,
    input  logic                         q_overflow,    // xung từ roi_desc_queue
    output logic                         q_invalid,     // xung: descriptor sai (cho roi_cpl_queue ghi CPL_INVALID)
    input  logic                         q_full,
    input  logic                         q_empty,
    input  logic [$clog2(ROI_QUEUE_DEPTH):0] q_count,

    // Trạng thái phần cứng
    input  logic                         hw_busy,
    input  logic                         hw_done,       // xung 1 chu kỳ khi xong 1 ROI
    input  logic [TS_WIDTH-1:0]          ts_now,

    output logic                         irq
);

    localparam logic [7:0] VERSION = 8'h01;
    localparam int         CW      = $clog2(ROI_QUEUE_DEPTH) + 1;

    localparam logic [11:0] A_CTRL      = 12'h000;
    localparam logic [11:0] A_STATUS    = 12'h004;
    localparam logic [11:0] A_IRQ_EN    = 12'h008;
    localparam logic [11:0] A_PAD_VAL   = 12'h00C;
    localparam logic [11:0] A_CAP       = 12'h010;
    localparam logic [11:0] A_IBUF_N    = 12'h014;
    localparam logic [11:0] A_ROI_POS   = 12'h020;
    localparam logic [11:0] A_ROI_DIM   = 12'h024;
    localparam logic [11:0] A_ROI_ID    = 12'h028;
    localparam logic [11:0] A_ROI_DL_LO = 12'h02C;
    localparam logic [11:0] A_ROI_DL_HI = 12'h030;
    localparam logic [11:0] A_ROI_PUSH  = 12'h034;
    localparam logic [11:0] A_TS_LO     = 12'h038;
    localparam logic [11:0] A_TS_HI     = 12'h03C;

    wire [11:0] addr     = 12'(paddr);
    wire        apb_acc  = psel && penable;
    wire        apb_wr   = apb_acc &&  pwrite;
    wire        apb_rd   = apb_acc && !pwrite;

    logic mapped;
    always_comb begin
        case (addr)
            A_CTRL, A_STATUS, A_IRQ_EN, A_PAD_VAL, A_CAP, A_IBUF_N,
            A_ROI_POS, A_ROI_DIM, A_ROI_ID, A_ROI_DL_LO, A_ROI_DL_HI,
            A_ROI_PUSH, A_TS_LO, A_TS_HI: mapped = 1'b1;
            default:                      mapped = 1'b0;
        endcase
    end

    assign pready  = 1'b1;
    assign pslverr = apb_acc && !mapped;

    // ---------------- Thanh ghi ----------------
    logic                 enable_r;
    logic [5:0]           irq_en_r;
    logic [7:0]           pad_val_r;
    logic [3:0]           ibuf_n_r;
    logic signed [11:0]   roi_x_r, roi_y_r;
    logic [7:0]           roi_w_r, roi_h_r, roi_id_r, frame_id_r;
    logic [1:0]           dec_r;
    logic [TS_WIDTH-1:0]  deadline_r;
    logic                 st_done, st_ovf, st_inv;
    logic [15:0]          ts_hi_snap;

    // Descriptor dựng từ các thanh ghi tạm + timestamp lúc push
    u02_roi_desc_t staged;
    always_comb begin
        staged          = '0;
        staged.roi_x    = roi_x_r;
        staged.roi_y    = roi_y_r;
        staged.roi_w    = roi_w_r;
        staged.roi_h    = roi_h_r;
        staged.dec_log2 = dec_r;
        staged.roi_id   = roi_id_r;
        staged.frame_id = frame_id_r;
        staged.deadline = deadline_r;
        staged.t_submit = ts_now;
    end

    wire push_cmd = apb_wr && (addr == A_ROI_PUSH) && pwdata[0];
    wire w1c      = apb_wr && (addr == A_STATUS);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable_r   <= 1'b0;
            irq_en_r   <= '0;
            pad_val_r  <= PAD_VAL_DEFAULT;
            ibuf_n_r   <= 4'(IBUF_N_DEFAULT);
            roi_x_r    <= '0;
            roi_y_r    <= '0;
            roi_w_r    <= 8'(ROI_MIN_DIM);
            roi_h_r    <= 8'(ROI_MIN_DIM);
            roi_id_r   <= '0;
            frame_id_r <= '0;
            dec_r      <= '0;
            deadline_r <= '0;
            q_push     <= 1'b0;
            q_invalid  <= 1'b0;
            q_desc     <= '0;
            st_done    <= 1'b0;
            st_ovf     <= 1'b0;
            st_inv     <= 1'b0;
            ts_hi_snap <= '0;
        end else begin
            q_push    <= 1'b0;
            q_invalid <= 1'b0;

            if (apb_wr) begin
                case (addr)
                    A_CTRL:      enable_r   <= pwdata[0];
                    A_IRQ_EN:    irq_en_r   <= pwdata[5:0];
                    A_PAD_VAL:   pad_val_r  <= pwdata[7:0];
                    A_IBUF_N:    ibuf_n_r   <= pwdata[3:0];
                    A_ROI_POS: begin
                        roi_x_r <= pwdata[11:0];
                        roi_y_r <= pwdata[27:16];
                    end
                    A_ROI_DIM: begin
                        roi_w_r <= pwdata[7:0];
                        roi_h_r <= pwdata[23:16];
                    end
                    A_ROI_ID: begin
                        roi_id_r   <= pwdata[7:0];
                        frame_id_r <= pwdata[15:8];
                        dec_r      <= pwdata[17:16];
                    end
                    A_ROI_DL_LO: deadline_r[31:0]  <= pwdata;
                    A_ROI_DL_HI: deadline_r[47:32] <= pwdata[15:0];
                    default: ;
                endcase
            end

            // Push: chụp descriptor tại thời điểm ghi, kiểm tra hợp lệ
            if (push_cmd) begin
                q_desc <= staged;
                if (roi_desc_valid(staged)) q_push    <= 1'b1;
                else                        q_invalid <= 1'b1;
            end

            // Cờ sticky: sự kiện mới được ưu tiên hơn lệnh xóa W1C cùng chu kỳ
            st_done <= hw_done    | (st_done & ~(w1c & pwdata[1]));
            st_ovf  <= q_overflow | (st_ovf  & ~(w1c & pwdata[4]));
            st_inv  <= q_invalid  | (st_inv  & ~(w1c & pwdata[5]));

            // Đọc TS_LO chốt phần cao để đọc 48 bit nhất quán
            if (apb_rd && addr == A_TS_LO)
                ts_hi_snap <= ts_now[47:32];
        end
    end

    // ---------------- Đọc ----------------
    always_comb begin
        prdata = 32'h0;
        if (psel && !pwrite) begin
            case (addr)
                A_CTRL:      prdata = {31'b0, enable_r};
                A_STATUS:    prdata = {19'b0, (5)'(q_count), 2'b0, st_inv, st_ovf, q_empty, q_full, st_done, hw_busy};
                A_IRQ_EN:    prdata = {26'b0, irq_en_r};
                A_PAD_VAL:   prdata = {24'b0, pad_val_r};
                A_CAP:       prdata = {VERSION, 8'(ROI_QUEUE_DEPTH), 8'(ROI_MAX_DIM), 8'(ROI_MIN_DIM)};
                A_IBUF_N:    prdata = {28'b0, ibuf_n_r};
                A_ROI_POS:   prdata = {4'b0, roi_y_r, 4'b0, roi_x_r};
                A_ROI_DIM:   prdata = {8'b0, roi_h_r, 8'b0, roi_w_r};
                A_ROI_ID:    prdata = {14'b0, dec_r, frame_id_r, roi_id_r};
                A_ROI_DL_LO: prdata = deadline_r[31:0];
                A_ROI_DL_HI: prdata = {16'b0, deadline_r[47:32]};
                A_ROI_PUSH:  prdata = 32'h0;
                A_TS_LO:     prdata = ts_now[31:0];
                A_TS_HI:     prdata = {16'b0, ts_hi_snap};
                default:     prdata = 32'h0;
            endcase
        end
    end

    assign cfg_enable  = enable_r;
    assign cfg_pad_val = pad_val_r;
    assign cfg_ibuf_n  = ibuf_n_r;

    assign irq = (st_done & irq_en_r[1]) | (st_ovf & irq_en_r[4]) | (st_inv & irq_en_r[5]);

endmodule
