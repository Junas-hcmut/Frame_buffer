`timescale 1ns / 1ps

module addr_gen_2d
    import u02_pkg::*;
#(
    parameter ADDR_WIDTH  = 32,
    parameter COORD_WIDTH = 12
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Giao tiếp Điều khiển (từ Scheduler / Queue)
    input  logic                          start,
    output logic                          busy,
    output logic                          done,

    // Cấu hình ROI (được chốt tại start)
    input  logic [ADDR_WIDTH-1:0]         cfg_base_addr,
    input  logic signed [COORD_WIDTH-1:0] cfg_roi_x,
    input  logic signed [COORD_WIDTH-1:0] cfg_roi_y,
    input  logic [COORD_WIDTH-1:0]        cfg_roi_w,      // W ngõ ra (16..128)
    input  logic [COORD_WIDTH-1:0]        cfg_roi_h,      // H ngõ ra (16..128)
    input  logic [1:0]                    cfg_dec_log2,   // 0 -> 1x, 1 -> 2x, 2 -> 4x
    input  logic [7:0]                    cfg_roi_id,
    input  logic [7:0]                    cfg_frame_id,

    // Ready/Valid sang axis_crop: mỗi handshake = 1 pixel NGÕ RA
    output logic                          out_valid,
    input  logic                          out_ready,
    output logic [ADDR_WIDTH-1:0]         out_addr,       // Địa chỉ byte R của pixel nguồn
    output logic                          out_pad,
    output logic                          out_eol,
    output logic                          out_eof,

    // Metadata cho header beat
    output logic                          out_has_pad,
    output logic [COORD_WIDTH-1:0]        out_w,
    output logic [COORD_WIDTH-1:0]        out_h,
    output logic [1:0]                    out_dec_log2,
    output logic [7:0]                    out_roi_id,
    output logic [7:0]                    out_frame_id
);

    typedef enum logic [1:0] { IDLE = 2'b00, CALC = 2'b01, GEN = 2'b10 } state_t;
    state_t state_reg, state_next;

    // Tọa độ nội bộ dùng 16-bit có dấu để không tràn (x + (W-1)*4 tối đa ~2555)
    localparam int CW = 16;
    localparam logic signed [CW-1:0] FW = CW'(FRAME_WIDTH);
    localparam logic signed [CW-1:0] FH = CW'(FRAME_HEIGHT);

    logic [ADDR_WIDTH-1:0]         base_reg;
    logic signed [CW-1:0]          roi_x_reg, roi_y_reg;
    logic [COORD_WIDTH-1:0]        roi_w_reg, roi_h_reg;
    logic [1:0]                    dec_reg;
    logic [7:0]                    roi_id_reg, frame_id_reg;
    logic                          has_pad_reg;

    logic [COORD_WIDTH-1:0]        col_cnt;   // pixel NGÕ RA trong dòng: 0 .. W-1
    logic [COORD_WIDTH-1:0]        row_cnt;   // dòng NGÕ RA: 0 .. H-1

    // Địa chỉ: cộng dồn, không dùng bộ nhân. Giá trị ban đầu có thể "âm"
    // (base + y*1920 với y<0): số học modulo 2^32 vẫn đúng, và địa chỉ chỉ
    // được dùng khi pixel nằm trong biên.
    logic [ADDR_WIDTH-1:0]         row_base_addr;
    logic [ADDR_WIDTH-1:0]         col_base_off;    // x*3 (có dấu), dùng lại mỗi dòng
    logic [ADDR_WIDTH-1:0]         col_byte_off;
    logic [ADDR_WIDTH-1:0]         line_step_b;     // 1920 << dec
    logic [ADDR_WIDTH-1:0]         pix_step_b;      // 3 << dec

    assign line_step_b = ADDR_WIDTH'(FRAME_STRIDE_B) << dec_reg;
    assign pix_step_b  = ADDR_WIDTH'(3) << dec_reg;

    wire signed [CW-1:0] curr_src_x = roi_x_reg + $signed({4'b0, col_cnt} << dec_reg);
    wire signed [CW-1:0] curr_src_y = roi_y_reg + $signed({4'b0, row_cnt} << dec_reg);

    wire is_pad      = (curr_src_x < 0) || (curr_src_x >= FW) ||
                       (curr_src_y < 0) || (curr_src_y >= FH);
    wire is_last_col = (col_cnt == roi_w_reg - 1'b1);
    wire is_last_row = (row_cnt == roi_h_reg - 1'b1);
    wire hs          = out_valid && out_ready;

    // has_pad tính trên cấu hình đầu vào: vùng nguồn = (W<<dec) x (H<<dec)
    wire signed [CW-1:0] cfg_x   = CW'(cfg_roi_x);
    wire signed [CW-1:0] cfg_y   = CW'(cfg_roi_y);
    wire signed [CW-1:0] cfg_wsp = $signed({4'b0, cfg_roi_w} << cfg_dec_log2);
    wire signed [CW-1:0] cfg_hsp = $signed({4'b0, cfg_roi_h} << cfg_dec_log2);
    wire roi_exceeds_bounds = (cfg_x < 0) || (cfg_y < 0) ||
                              (cfg_x + cfg_wsp > FW) || (cfg_y + cfg_hsp > FH);

    // Sign-extend 32-bit cho phép tính địa chỉ khởi tạo
    wire [ADDR_WIDTH-1:0] y32 = ADDR_WIDTH'(roi_y_reg);   // roi_y_reg có dấu -> sign-extend
    wire [ADDR_WIDTH-1:0] x32 = ADDR_WIDTH'(roi_x_reg);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg     <= IDLE;
            base_reg      <= '0;
            roi_x_reg     <= '0;  roi_y_reg <= '0;
            roi_w_reg     <= '0;  roi_h_reg <= '0;
            dec_reg       <= '0;
            roi_id_reg    <= '0;  frame_id_reg <= '0;
            has_pad_reg   <= 1'b0;
            row_cnt       <= '0;  col_cnt <= '0;
            row_base_addr <= '0;  col_base_off <= '0; col_byte_off <= '0;
        end else begin
            state_reg <= state_next;
            case (state_reg)
                IDLE: if (start) begin
                    base_reg     <= cfg_base_addr;
                    roi_x_reg    <= cfg_x;
                    roi_y_reg    <= cfg_y;
                    roi_w_reg    <= cfg_roi_w;
                    roi_h_reg    <= cfg_roi_h;
                    dec_reg      <= cfg_dec_log2;
                    roi_id_reg   <= cfg_roi_id;
                    frame_id_reg <= cfg_frame_id;
                    has_pad_reg  <= roi_exceeds_bounds;
                end

                CALC: begin
                    row_cnt       <= '0;
                    col_cnt       <= '0;
                    // y*1920 = (y<<11) - (y<<7);  x*3 = (x<<1) + x  -- đúng cả khi âm
                    row_base_addr <= base_reg + (y32 << 11) - (y32 << 7);
                    col_base_off  <= (x32 << 1) + x32;
                    col_byte_off  <= (x32 << 1) + x32;
                end

                GEN: if (hs) begin
                    if (is_last_col) begin
                        col_cnt      <= '0;
                        col_byte_off <= col_base_off;
                        if (!is_last_row) begin
                            row_cnt       <= row_cnt + 1'b1;
                            row_base_addr <= row_base_addr + line_step_b;
                        end
                    end else begin
                        col_cnt      <= col_cnt + 1'b1;
                        col_byte_off <= col_byte_off + pix_step_b;
                    end
                end

                default: ;
            endcase
        end
    end

    always_comb begin
        state_next = state_reg;
        case (state_reg)
            IDLE:    if (start) state_next = CALC;
            CALC:    state_next = GEN;
            GEN:     if (hs && is_last_col && is_last_row) state_next = IDLE;
            default: state_next = IDLE;
        endcase
    end

    assign busy      = (state_reg != IDLE);
    assign done      = (state_reg == GEN) && hs && is_last_col && is_last_row;
    assign out_valid = (state_reg == GEN);
    assign out_addr  = is_pad ? '0 : (row_base_addr + col_byte_off);
    assign out_pad   = is_pad;
    assign out_eol   = is_last_col;
    assign out_eof   = is_last_col && is_last_row;

    assign out_has_pad  = has_pad_reg;
    assign out_w        = roi_w_reg;
    assign out_h        = roi_h_reg;
    assign out_dec_log2 = dec_reg;
    assign out_roi_id   = roi_id_reg;
    assign out_frame_id = frame_id_reg;

endmodule
