`timescale 1ns / 1ps

// =====================================================================
// axis_crop (bản đọc theo hàng): ghép pixel ROI và phát sang U01.
//
// Đầu vào:
//  - row_info từng hàng: lead_pad / n_valid / trail_pad (+ header ở hàng đầu)
//  - beat dữ liệu từ axi_master_rd, mỗi beat có {lo, cnt} byte hợp lệ;
//    dòng byte của 1 hàng = đúng đoạn từ pixel hợp lệ đầu tới cuối.
// Xử lý:
//  - Bộ đệm byte 24 B: nạp beat (tối đa 8 B), mỗi pixel hợp lệ lấy 3 B rồi
//    bỏ 3*(dec-1) B (trừ pixel hợp lệ cuối của hàng).
//  - Pixel pad = {PAD_VAL, PAD_VAL, PAD_VAL}.
//  - Bộ gom 16 B: mỗi chu kỳ thêm 1 pixel (3 B), đủ 8 B thì phát 1 beat.
// Đầu ra AXI4-Stream 64-bit theo đặc tả U01<->U02:
//  - beat header (tuser=1) rồi payload HWC nối liền; tlast ở beat cuối,
//    tkeep chỉ ở beat cuối có thể < 0xFF.
//  - roi_done: xung khi beat tlast được U01 nhận.
// =====================================================================
module axis_crop
    import u02_pkg::*;
(
    input  logic          clk,
    input  logic          rst_n,
    input  logic [7:0]    cfg_pad_val,

    // row_info (FWFT)
    input  logic          ri_valid,
    output logic          ri_ready,
    input  u02_row_info_t ri,

    // Beat dữ liệu từ axi_master_rd (FWFT)
    input  logic          d_valid,
    output logic          d_ready,
    input  logic [63:0]   d_data,
    input  logic [2:0]    d_lo,
    input  logic [3:0]    d_cnt,
    input  logic          d_row_last,
    input  logic          d_err,

    // AXI4-Stream sang U01
    output logic          m_axis_tvalid,
    input  logic          m_axis_tready,
    output logic [63:0]   m_axis_tdata,
    output logic [7:0]    m_axis_tkeep,
    output logic          m_axis_tlast,
    output logic [0:0]    m_axis_tuser,

    // Trạng thái
    output logic          roi_done,
    output logic          roi_err
);

    // ---------------- Trạng thái ----------------
    logic [191:0] bb;           // bộ đệm byte, byte 0 ở bit [7:0]
    logic [4:0]   bcnt;         // 0..24
    logic [127:0] acc;          // bộ gom đầu ra
    logic [4:0]   acnt;         // 0..16
    logic [7:0]   p;            // chỉ số pixel trong hàng
    logic         hdr_sent;
    logic         flush;        // đã thêm pixel cuối của ROI, đang xả bộ gom
    logic         err_q;

    // ---------------- Skid buffer đầu ra ----------------
    logic        s_valid, s_ready, s_last;
    logic [63:0] s_data;
    logic [7:0]  s_keep;
    logic [0:0]  s_user;

    skid_buffer #(.DATA_WIDTH(64), .KEEP_WIDTH(8), .USER_WIDTH(1)) u_skid (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(s_valid), .s_axis_tready(s_ready), .s_axis_tdata(s_data),
        .s_axis_tkeep(s_keep), .s_axis_tlast(s_last), .s_axis_tuser(s_user),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready), .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(m_axis_tkeep), .m_axis_tlast(m_axis_tlast), .m_axis_tuser(m_axis_tuser)
    );

    // ---------------- Header / beat payload ----------------
    wire hdr_emit   = ri_valid && ri.first && !hdr_sent && !flush && (acnt == 5'd0);
    wire pk_out_req = !hdr_emit && ((acnt >= 5'd8) || (flush && acnt != 5'd0));
    wire pk_out     = pk_out_req && s_ready;
    wire last_beat  = flush && (acnt <= 5'd8);

    assign s_valid = hdr_emit || pk_out_req;
    assign s_data  = hdr_emit ? 64'(ri.hdr) : acc[63:0];
    assign s_last  = !hdr_emit && last_beat;
    assign s_user  = hdr_emit;
    assign s_keep  = (!hdr_emit && last_beat) ? 8'((9'd1 << acnt) - 1'b1) : 8'hFF;

    wire hdr_hs  = hdr_emit && s_ready;
    wire last_hs = pk_out && last_beat;

    // ---------------- Pixel ----------------
    wire [7:0] lead     = ri.lead_pad;
    wire [7:0] nval     = ri.n_valid;
    wire [7:0] w        = ri.lead_pad + ri.n_valid + ri.trail_pad;
    wire       is_pad   = (p < lead) || (p >= lead + nval);
    wire [7:0] vi       = p - lead;
    wire       last_val = (vi == nval - 1'b1);
    wire [4:0] need     = last_val ? 5'd3 : (5'd3 << ri.dec_log2);    // 3, 6 hoặc 12 byte
    wire       bytes_ok = is_pad || (bcnt >= need);
    wire       can_add  = (acnt < 5'd8) || pk_out;
    wire       row_act  = ri_valid && (!ri.first || hdr_sent) && !flush;
    wire       add      = row_act && bytes_ok && can_add;
    wire       row_end  = add && (p == w - 1'b1);

    wire [23:0] pix = is_pad ? {3{cfg_pad_val}} : bb[23:0];

    assign ri_ready = row_end;

    // ---------------- Nạp byte ----------------
    assign d_ready = (bcnt <= 5'd16);
    wire   d_hs    = d_valid && d_ready;
    wire [4:0]  pop_n    = (add && !is_pad) ? need : 5'd0;
    wire [63:0] in_bytes = (d_data >> ({3'b0, d_lo} * 8)) & ((64'h1 << ({2'b0, d_cnt} * 8)) - 1'b1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bb       <= '0;
            bcnt     <= '0;
            acc      <= '0;
            acnt     <= '0;
            p        <= '0;
            hdr_sent <= 1'b0;
            flush    <= 1'b0;
            err_q    <= 1'b0;
            roi_done <= 1'b0;
            roi_err  <= 1'b0;
        end else begin
            // Bộ đệm byte
            begin : byte_buf
                logic [191:0] b;
                logic [4:0]   c;
                b = bb >> (pop_n * 8);
                c = bcnt - pop_n;
                if (d_hs) begin
                    b = b | (192'(in_bytes) << (c * 8));
                    c = c + ((d_cnt == 4'd8) ? 5'd8 : {1'b0, d_cnt});
                end
                bb   <= b;
                bcnt <= c;
            end

            // Bộ gom đầu ra
            begin : packer
                logic [127:0] a;
                logic [4:0]   c;
                a = acc;
                c = acnt;
                if (pk_out) begin
                    a = a >> 64;
                    c = last_beat ? 5'd0 : c - 5'd8;   // beat cuối có thể < 8 byte
                end
                if (add) begin
                    a = a | (128'(pix) << (c * 8));
                    c = c + 5'd3;
                end
                acc  <= a;
                acnt <= c;
            end

            // Tiến trình hàng / ROI
            if (hdr_hs) hdr_sent <= 1'b1;
            if (add) p <= row_end ? 8'd0 : p + 1'b1;
            if (row_end && ri.last) flush <= 1'b1;
            if (last_hs) begin
                flush    <= 1'b0;
                hdr_sent <= 1'b0;
            end

            // Lỗi bus trong ROI
            if (d_hs && d_err) err_q <= 1'b1;

            // Kết thúc ROI tại phía U01
            roi_done <= m_axis_tvalid && m_axis_tready && m_axis_tlast;
            roi_err  <= err_q;
            if (m_axis_tvalid && m_axis_tready && m_axis_tlast) err_q <= 1'b0;
        end
    end

    wire unused = d_row_last;

endmodule
