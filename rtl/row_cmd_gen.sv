`timescale 1ns / 1ps

// =====================================================================
// row_cmd_gen: với mỗi ROI, sinh cho TỪNG HÀNG ngõ ra:
//   - 1 lệnh đọc {addr, len} liền mạch (sang page_splitter) nếu hàng có
//     pixel nằm trong khung;
//   - 1 row_info {lead_pad, n_valid, trail_pad, ...} (sang axis_crop).
//
// Cột: pixel ngõ ra i lấy pixel nguồn x + i*dec. Các giá trị lead/n_valid/
// trail giống nhau cho mọi hàng nên chỉ tính 1 lần (trạng thái CALC).
// Hàng: nguồn y + r*dec; hàng ngoài khung -> toàn pad, không đọc.
// Đoạn đọc mỗi hàng = từ pixel hợp lệ đầu tới pixel hợp lệ cuối (kể cả
// các pixel bị bỏ qua khi dec > 1): ((n_valid-1)*dec + 1) * 3 byte.
// Không dùng bộ nhân: *1920 = <<11 - <<7, *3 = <<1 + x, *dec = dịch.
// =====================================================================
module row_cmd_gen
    import u02_pkg::*;
(
    input  logic          clk,
    input  logic          rst_n,

    // Điều khiển từ scheduler
    input  logic          start,
    input  u02_roi_desc_t desc,
    input  logic [31:0]   base,        // địa chỉ frame buffer
    output logic          busy,

    // Lệnh đọc hàng -> page_splitter
    output logic          rc_valid,
    input  logic          rc_ready,
    output logic [31:0]   rc_addr,
    output logic [11:0]   rc_len,
    output logic [7:0]    rc_roi_id,
    output logic [7:0]    rc_frame_id,
    output logic          rc_eol,
    output logic          rc_eof,

    // Thông tin hàng -> axis_crop
    output logic          ri_valid,
    input  logic          ri_ready,
    output u02_row_info_t ri
);

    localparam int CW = 16;
    localparam logic signed [CW-1:0] FW = CW'(FRAME_WIDTH);
    localparam logic signed [CW-1:0] FH = CW'(FRAME_HEIGHT);

    typedef enum logic [1:0] { IDLE, CALC, ROW } st_t;
    st_t st;

    // Tham số ROI chốt lúc start
    logic signed [CW-1:0] x_q, y_q;
    logic [7:0]           w_q, h_q;
    logic [1:0]           dl_q;
    logic [7:0]           roi_id_q, fid_q;
    logic [31:0]          base_q;
    u02_roi_desc_t        desc_q;

    // Kết quả CALC
    logic [7:0]           lead_q, nval_q, trail_q;
    logic [31:0]          col_off_q;       // byte offset của pixel hợp lệ đầu tiên
    logic [11:0]          span_q;          // số byte đọc mỗi hàng
    logic [31:0]          row_base_q;      // base + src_y*1920 (modulo 2^32)
    u02_header_t          hdr_q;

    // Bộ đếm hàng
    logic [7:0]           r;
    logic signed [CW-1:0] src_y;
    logic                 ri_done, rc_done;

    // ---------------- Tính tham số cột (tổ hợp, dùng trong CALC) ----------------
    logic [CW-1:0]        dec_m1;
    logic signed [CW-1:0] neg_x, lead_raw, imax, nhi_s, nval_s;
    logic [7:0]           lead_c, nval_c, trail_c;
    logic signed [CW-1:0] first_x;
    logic [CW-1:0]        span_px;         // (n_valid-1)*dec + 1
    logic                 has_pad_c;

    always_comb begin
        dec_m1   = (CW'(1) << dl_q) - 1'b1;
        neg_x    = -x_q;
        lead_raw = (x_q < 0) ? ((neg_x + $signed(dec_m1)) >>> dl_q) : '0;
        lead_c   = (lead_raw > $signed(CW'(w_q))) ? w_q : 8'(lead_raw);

        imax = '0;
        if (x_q >= FW) begin
            nhi_s = '0;
        end else begin
            imax  = (FW - 1 - x_q) >>> dl_q;                 // chỉ số ngõ ra lớn nhất còn trong khung
            nhi_s = (imax + 1 > $signed(CW'(w_q))) ? $signed(CW'(w_q)) : imax + 1;
        end
        nval_s  = (nhi_s > $signed(CW'(lead_c))) ? nhi_s - $signed(CW'(lead_c)) : '0;
        nval_c  = 8'(nval_s);
        trail_c = w_q - lead_c - nval_c;

        first_x = x_q + $signed(CW'(lead_c) << dl_q);
        span_px = (nval_c == 8'd0) ? '0 : (((CW'(nval_c) - 1'b1) << dl_q) + 1'b1);

        has_pad_c = (x_q < 0) || (y_q < 0) ||
                    (x_q + $signed(CW'(w_q) << dl_q) > FW) ||
                    (y_q + $signed(CW'(h_q) << dl_q) > FH);
    end

    // ---------------- Hàng hiện tại ----------------
    wire in_y     = (src_y >= 0) && (src_y < FH);
    wire has_read = in_y && (nval_q != 8'd0);
    wire last_row = (r == h_q - 1'b1);

    assign busy        = (st != IDLE);

    assign rc_valid    = (st == ROW) && has_read && !rc_done;
    assign rc_addr     = row_base_q + col_off_q;
    assign rc_len      = span_q;
    assign rc_roi_id   = roi_id_q;
    assign rc_frame_id = fid_q;
    assign rc_eol      = 1'b1;
    assign rc_eof      = last_row;

    assign ri_valid    = (st == ROW) && !ri_done;
    always_comb begin
        ri           = '0;
        ri.first     = (r == 8'd0);
        ri.last      = last_row;
        ri.dec_log2  = dl_q;
        ri.lead_pad  = in_y ? lead_q  : w_q;
        ri.n_valid   = in_y ? nval_q  : 8'd0;
        ri.trail_pad = in_y ? trail_q : 8'd0;
        ri.hdr       = hdr_q;
    end

    wire ri_ok   = ri_done || (ri_valid && ri_ready);
    wire rc_ok   = !has_read || rc_done || (rc_valid && rc_ready);
    wire advance = (st == ROW) && ri_ok && rc_ok;

    // ---------------- FSM ----------------
    wire [31:0] y32 = 32'(y_q);           // sign-extend
    wire [31:0] fx32 = 32'(first_x);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= IDLE;
            r       <= '0;
            ri_done <= 1'b0;
            rc_done <= 1'b0;
        end else begin
            case (st)
                IDLE: if (start) st <= CALC;
                CALC: begin
                    st      <= ROW;
                    r       <= '0;
                    ri_done <= 1'b0;
                    rc_done <= 1'b0;
                end
                ROW: begin
                    if (advance) begin
                        ri_done <= 1'b0;
                        rc_done <= 1'b0;
                        if (last_row) st <= IDLE;
                        else          r  <= r + 1'b1;
                    end else begin
                        if (ri_valid && ri_ready) ri_done <= 1'b1;
                        if (rc_valid && rc_ready) rc_done <= 1'b1;
                    end
                end
                default: st <= IDLE;
            endcase
        end
    end

    // Datapath (không reset)
    always_ff @(posedge clk) begin
        if (st == IDLE && start) begin
            desc_q   <= desc;
            x_q      <= CW'(desc.roi_x);
            y_q      <= CW'(desc.roi_y);
            w_q      <= desc.roi_w;
            h_q      <= desc.roi_h;
            dl_q     <= desc.dec_log2;
            roi_id_q <= desc.roi_id;
            fid_q    <= desc.frame_id;
            base_q   <= base;
        end
        if (st == CALC) begin
            lead_q     <= lead_c;
            nval_q     <= nval_c;
            trail_q    <= trail_c;
            col_off_q  <= (fx32 << 1) + fx32;                               // first_x * 3
            span_q     <= 12'((span_px << 1) + span_px);                    // span_px * 3
            row_base_q <= base_q + (y32 << 11) - (y32 << 7);                // base + y*1920
            src_y      <= y_q;
            hdr_q      <= make_header(desc_q, has_pad_c);
        end
        if (advance && !last_row) begin
            row_base_q <= row_base_q + (32'(FRAME_STRIDE_B) << dl_q);
            src_y      <= src_y + (CW'(1) << dl_q);
        end
    end

endmodule
