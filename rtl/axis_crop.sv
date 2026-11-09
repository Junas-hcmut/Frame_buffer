`timescale 1ns / 1ps

// =====================================================================
// axis_crop: ghép luồng ROI ra AXI4-Stream cho U01.
//
// Đầu vào (FWFT):
//   ri_* : 1 row_info cho mỗi hàng ngõ ra (row_cmd_gen, qua FIFO)
//          lead_pad + n_valid + trail_pad = W; hdr ở hàng first.
//   d_*  : beat 64 bit từ axi_master_rd. Byte hợp lệ: d_cnt byte bắt đầu
//          từ byte d_lo. Mỗi hàng có n_valid > 0 nhận đúng đoạn byte
//          ((n_valid-1)*dec + 1)*3 (beat cuối mang d_row_last); hàng có
//          n_valid = 0 không có beat nào.
//
// Đầu ra AXI4-Stream (64 bit):
//   beat 0          : header (u02_header_t), tuser = 1, tkeep = FF
//   beat 1 .. N     : payload W*H*3 byte RGB888, các hàng nối liền nhau,
//                     byte thấp trước; chỉ beat cuối có thể thiếu byte
//                     (tkeep = 2^k - 1) và mang tlast.
//
// Cấu trúc:
//   Tầng 1 (nguồn byte theo hàng): HDR -> LEAD (pad) -> DATA -> TRAIL (pad)
//     Mỗi chu kỳ sinh 1 "chunk" <= 8 byte.
//     DATA: lấy byte hợp lệ của beat, giữ 3 byte đầu của mỗi nhóm 3*dec
//     byte (decimation theo cột), nén lại liền nhau.
//   Tầng 2 (packer): bộ đệm 16 byte, xuất beat 8 byte; beat cuối của ROI
//     xuất khi đủ số byte còn lại. Qua skid_buffer trước khi ra cổng.
//
// roi_done: xung khi beat tlast được U01 nhận (handshake tại cổng ra).
// roi_err : cùng chu kỳ roi_done, = 1 nếu có beat nào của ROI mang d_err.
// Thông lượng: <= 8 byte / chu kỳ ở cả hai tầng; 1 chu kỳ trống / hàng.
// =====================================================================
module axis_crop
    import u02_pkg::*;
(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic [7:0]                 cfg_pad_val,

    // row_info (FWFT)
    input  logic                       ri_valid,
    output logic                       ri_ready,
    input  u02_row_info_t              ri,

    // Dữ liệu từ axi_master_rd (FWFT)
    input  logic                       d_valid,
    output logic                       d_ready,
    input  logic [63:0]                d_data,
    input  logic [2:0]                 d_lo,
    input  logic [3:0]                 d_cnt,
    input  logic                       d_row_last,
    input  logic                       d_err,

    // AXI4-Stream sang U01
    output logic                       m_axis_tvalid,
    input  logic                       m_axis_tready,
    output logic [AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                       m_axis_tlast,
    output logic [0:0]                 m_axis_tuser,

    output logic                       roi_done,
    output logic                       roi_err
);

    // =================================================================
    // Tầng 1: nguồn byte
    // =================================================================
    typedef enum logic [2:0] { S_START, S_HDR, S_LEAD, S_DATA, S_TRAIL } st_t;
    st_t st;

    logic [8:0] pad_rem;          // byte pad còn lại của đoạn hiện tại (<= 384)
    logic [3:0] ph;               // pha byte trong nhóm 3*dec (0..11)
    logic       err_q;            // ROI hiện tại có beat lỗi

    wire  [8:0] lead_b  = 9'(ri.lead_pad)  * 9'd3;
    wire  [8:0] trail_b = 9'(ri.trail_pad) * 9'd3;
    wire  [3:0] grp_m1  = (ri.dec_log2 == 2'd0) ? 4'd2 :
                          (ri.dec_log2 == 2'd1) ? 4'd5 : 4'd11;      // 3*dec - 1

    // ---- Nén byte của beat DATA ----
    logic [63:0] src;
    logic [63:0] cmp_data;
    logic [3:0]  cmp_cnt;
    logic [3:0]  ph_next;
    always_comb begin
        logic [3:0] p;
        logic [3:0] k;
        src      = d_data >> {d_lo, 3'b000};
        cmp_data = '0;
        p        = ph;
        k        = '0;
        for (int j = 0; j < 8; j++) begin
            if (4'(j) < d_cnt) begin
                if (p < 4'd3) begin
                    cmp_data[k*8 +: 8] = src[j*8 +: 8];
                    k = k + 1'b1;
                end
                p = (p == grp_m1) ? 4'd0 : p + 1'b1;
            end
        end
        cmp_cnt = k;
        ph_next = p;
    end

    // ---- Chunk sang packer ----
    logic        ck_valid, ck_ready;
    logic [63:0] ck_data;
    logic [3:0]  ck_cnt;
    wire  [3:0]  pad_cnt = (pad_rem >= 9'd8) ? 4'd8 : 4'(pad_rem);

    // chỉ giữ ck_cnt byte thấp (byte cao của chunk pad không được lọt vào packer)
    logic [63:0] ck_mask;
    always_comb for (int j = 0; j < 8; j++) ck_mask[j*8 +: 8] = (4'(j) < ck_cnt) ? 8'hFF : 8'h00;
    wire  [63:0] ck_data_m = ck_data & ck_mask;

    always_comb begin
        ck_valid = 1'b0;
        ck_data  = {8{cfg_pad_val}};
        ck_cnt   = pad_cnt;
        case (st)
            S_LEAD, S_TRAIL: ck_valid = (pad_rem != '0);
            S_DATA: begin
                ck_valid = d_valid && (cmp_cnt != '0);
                ck_data  = cmp_data;
                ck_cnt   = cmp_cnt;
            end
            default: ;
        endcase
    end

    wire ck_hs     = ck_valid && ck_ready;
    assign d_ready = (st == S_DATA) && (cmp_cnt == '0 || ck_ready);
    wire d_hs      = d_valid && d_ready;

    // header: tầng 2 phát trực tiếp khi packer rỗng
    logic hdr_hs;
    assign ri_ready = (st == S_TRAIL) && (pad_rem == '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_START;
            pad_rem <= '0;
            ph      <= '0;
        end else begin
            case (st)
                S_START: if (ri_valid) begin
                    if (ri.first) st <= S_HDR;
                    else begin st <= S_LEAD; pad_rem <= lead_b; end
                end
                S_HDR: if (hdr_hs) begin
                    st      <= S_LEAD;
                    pad_rem <= lead_b;
                end
                S_LEAD: begin
                    if (pad_rem == '0) begin
                        ph <= '0;
                        if (ri.n_valid != 8'd0) st <= S_DATA;
                        else begin st <= S_TRAIL; pad_rem <= trail_b; end
                    end else if (ck_hs) begin
                        pad_rem <= pad_rem - 9'(ck_cnt);
                    end
                end
                S_DATA: if (d_hs) begin
                    ph <= ph_next;
                    if (d_row_last) begin st <= S_TRAIL; pad_rem <= trail_b; end
                end
                S_TRAIL: begin
                    if (pad_rem == '0)      st <= S_START;      // ri_ready = 1 chu kỳ này
                    else if (ck_hs)         pad_rem <= pad_rem - 9'(ck_cnt);
                end
                default: st <= S_START;
            endcase
        end
    end

    // =================================================================
    // Tầng 2: packer
    // =================================================================
    logic [127:0] ob;
    logic [4:0]   oc;             // byte trong ob (0..16)
    logic [15:0]  rem;            // byte payload ROI chưa xuất (<= 49152)

    logic        p_valid, p_ready, p_last, p_user;
    logic [63:0] p_data;
    logic [7:0]  p_keep;

    wire  [15:0] roi_bytes = 16'(ri.hdr.w) * 16'(ri.hdr.h) * 16'd3;
    wire         hdr_req   = (st == S_HDR) && (oc == '0) && (rem == '0);
    wire         fin_beat  = (rem != '0) && (rem <= 16'd8) && (5'(rem) <= oc);
    wire         full_beat = (rem > 16'd8) && (oc >= 5'd8);

    always_comb begin
        p_valid = 1'b0;
        p_data  = ob[63:0];
        p_keep  = 8'hFF;
        p_last  = 1'b0;
        p_user  = 1'b0;
        if (hdr_req) begin
            p_valid = 1'b1;
            p_data  = ri.hdr;
            p_user  = 1'b1;
        end else if (fin_beat) begin
            p_valid = 1'b1;
            p_keep  = 8'((16'd1 << rem) - 1'b1);
            p_last  = 1'b1;
        end else if (full_beat) begin
            p_valid = 1'b1;
        end
    end

    wire        p_hs     = p_valid && p_ready;
    assign      hdr_hs   = p_hs && hdr_req;
    wire        pay_pop  = p_hs && !hdr_req;
    wire  [4:0] pop_n    = pay_pop ? (fin_beat ? 5'(rem) : 5'd8) : 5'd0;
    wire  [4:0] oc_after = oc - pop_n;
    assign      ck_ready = (oc_after <= 5'd8);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ob  <= '0;
            oc  <= '0;
            rem <= '0;
        end else begin
            logic [127:0] b;
            b = ob >> {pop_n, 3'b000};
            if (ck_hs) b = b | ({64'd0, ck_data_m} << {oc_after, 3'b000});
            ob <= b;
            oc <= oc_after + (ck_hs ? 5'(ck_cnt) : 5'd0);
            if (hdr_hs)       rem <= roi_bytes;
            else if (pay_pop) rem <= rem - 16'(pop_n);
        end
    end

    skid_buffer #(.DATA_WIDTH(64), .USER_WIDTH(1)) u_skid (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(p_valid), .s_axis_tready(p_ready), .s_axis_tdata(p_data),
        .s_axis_tkeep(p_keep), .s_axis_tlast(p_last), .s_axis_tuser(p_user),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready), .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(m_axis_tkeep), .m_axis_tlast(m_axis_tlast), .m_axis_tuser(m_axis_tuser)
    );

    // =================================================================
    // Kết thúc ROI
    // =================================================================
    wire out_last = m_axis_tvalid && m_axis_tready && m_axis_tlast;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err_q    <= 1'b0;
            roi_done <= 1'b0;
            roi_err  <= 1'b0;
        end else begin
            roi_done <= out_last;
            roi_err  <= out_last && (err_q || (d_hs && d_err));
            if (out_last)            err_q <= 1'b0;
            else if (d_hs && d_err)  err_q <= 1'b1;
        end
    end

    // synopsys translate_off
    always @(posedge clk) if (rst_n) begin
        if (st == S_DATA && d_valid && d_hs && ri.n_valid == 8'd0)
            $error("axis_crop: nhan beat du lieu cho hang khong co pixel hop le");
        if (ck_hs && (5'(oc_after) + 5'(ck_cnt) > 5'd16))
            $error("axis_crop: tran bo dem packer");
    end
    // synopsys translate_on

endmodule
