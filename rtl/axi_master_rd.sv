`timescale 1ns / 1ps

// =====================================================================
// axi_master_rd: phát burst đọc AXI4 (AR/R) theo lệnh từ page_splitter.
//  - Đặt trước chỗ trong bộ đệm dữ liệu (credit) cho cả burst rồi mới
//    phát AR -> RREADY luôn = 1, kênh R không bao giờ bị giữ.
//  - Tối đa MAX_OUTSTANDING burst đang chờ dữ liệu, 1 ID -> đúng thứ tự.
//  - Mỗi beat ra kèm vị trí byte hợp lệ đầu (lo) và số byte hợp lệ (cnt)
//    để axis_crop ghép byte chính xác; beat cuối của hàng có row_last.
//  - RRESP lỗi: beat vẫn được chuyển tiếp, gắn cờ err.
// =====================================================================
module axi_master_rd
    import u02_pkg::*;
#(
    parameter int RBUF_DEPTH = 64          // beat
)(
    input  logic        clk,
    input  logic        rst_n,

    // Từ page_splitter
    input  logic        c_valid,
    output logic        c_ready,
    input  logic [31:0] c_addr,
    input  logic [11:0] c_len,
    input  logic [7:0]  c_arlen,
    input  logic        c_row_last,         // burst con cuối của hàng

    // AXI4 AR
    output logic [AXI_ID_WIDTH-1:0]   arid,
    output logic [AXI_ADDR_WIDTH-1:0] araddr,
    output logic [7:0]                arlen,
    output logic [2:0]                arsize,
    output logic [1:0]                arburst,
    output logic [3:0]                arcache,
    output logic [2:0]                arprot,
    output logic                      arvalid,
    input  logic                      arready,
    // AXI4 R
    input  logic [AXI_ID_WIDTH-1:0]   rid,
    input  logic [AXI_DATA_WIDTH-1:0] rdata,
    input  logic [1:0]                rresp,
    input  logic                      rlast,
    input  logic                      rvalid,
    output logic                      rready,

    // Sang axis_crop (FWFT)
    output logic        d_valid,
    input  logic        d_ready,
    output logic [63:0] d_data,
    output logic [2:0]  d_lo,
    output logic [3:0]  d_cnt,
    output logic        d_row_last,
    output logic        d_err,

    // Thống kê
    input  logic        cnt_clear,
    output logic [19:0] beat_cnt,
    output logic        bus_err_evt
);

    localparam int CRW = $clog2(RBUF_DEPTH) + 1;
    localparam int OW  = $clog2(MAX_OUTSTANDING + 1);

    assign arid    = '0;
    assign arsize  = 3'($clog2(AXI_DATA_WIDTH / 8));
    assign arburst = 2'b01;
    assign arcache = 4'b0011;
    assign arprot  = 3'b000;

    // ---------------- Meta mỗi burst: {lo, len, row_last} ----------------
    logic        mq_push, mq_pop, mq_full, mq_empty;
    logic [15:0] mq_din, mq_dout;
    sync_fifo #(.DW(16), .DEPTH(4)) u_meta (
        .clk(clk), .rst_n(rst_n), .wr_en(mq_push), .wr_data(mq_din), .full(mq_full),
        .rd_en(mq_pop), .rd_data(mq_dout), .empty(mq_empty), .count()
    );

    // ---------------- Bộ đệm dữ liệu ----------------
    localparam int DDW = 64 + 3 + 4 + 1 + 1;
    logic           df_push, df_full, df_empty;
    logic [DDW-1:0] df_din, df_dout;
    sync_fifo #(.DW(DDW), .DEPTH(RBUF_DEPTH)) u_data (
        .clk(clk), .rst_n(rst_n), .wr_en(df_push), .wr_data(df_din), .full(df_full),
        .rd_en(d_ready), .rd_data(df_dout), .empty(df_empty), .count()
    );
    assign d_valid = !df_empty;
    assign {d_data, d_lo, d_cnt, d_row_last, d_err} = df_dout;

    // ---------------- Credit & outstanding ----------------
    logic [CRW-1:0] credit;        // số ô bộ đệm chưa bị đặt trước
    logic [OW-1:0]  ost;
    wire  [CRW-1:0] need    = CRW'(c_arlen) + 1'b1;
    wire            ar_free = !arvalid || arready;
    wire            d_pop   = d_valid && d_ready;

    assign c_ready = ar_free && (ost < OW'(MAX_OUTSTANDING)) && (credit >= need) && !mq_full;
    wire   c_hs    = c_valid && c_ready;

    assign mq_push = c_hs;
    assign mq_din  = {c_addr[2:0], c_len, c_row_last};

    // ---------------- Kênh R ----------------
    assign rready = 1'b1;                 // chỗ đã được đặt trước
    wire   r_hs   = rvalid && rready;

    logic        r_first;
    logic [11:0] r_rem;
    wire  [2:0]  m_lo      = mq_dout[15:13];
    wire  [11:0] m_len     = mq_dout[12:1];
    wire         m_rowlast = mq_dout[0];

    wire  [2:0]  b_lo   = r_first ? m_lo : 3'd0;
    wire  [11:0] b_rem  = r_first ? m_len : r_rem;
    wire  [3:0]  b_room = 4'd8 - {1'b0, b_lo};
    wire  [3:0]  b_cnt  = (b_rem < 12'(b_room)) ? b_rem[3:0] : b_room;

    assign df_push = r_hs;
    assign df_din  = {rdata, b_lo, b_cnt, (m_rowlast && rlast), (rresp != 2'b00)};
    assign mq_pop  = r_hs && rlast;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arvalid     <= 1'b0;
            araddr      <= '0;
            arlen       <= '0;
            credit      <= CRW'(RBUF_DEPTH);
            ost         <= '0;
            r_first     <= 1'b1;
            r_rem       <= '0;
            beat_cnt    <= '0;
            bus_err_evt <= 1'b0;
        end else begin
            // AR
            if (c_hs) begin
                arvalid <= 1'b1;
                araddr  <= c_addr;
                arlen   <= c_arlen;
            end else if (arready) begin
                arvalid <= 1'b0;
            end

            credit <= credit - (c_hs ? need : '0) + CRW'(d_pop);
            ost    <= ost + OW'(c_hs) - OW'(r_hs && rlast);

            // R
            if (r_hs) begin
                r_first <= rlast;
                r_rem   <= b_rem - 12'(b_cnt);
            end

            if (cnt_clear)  beat_cnt <= '0;
            else if (r_hs)  beat_cnt <= beat_cnt + 1'b1;

            bus_err_evt <= r_hs && (rresp != 2'b00);
        end
    end

    // rid không dùng (1 ID); df_full không bao giờ bật nhờ credit
    wire unused = ^rid ^ df_full ^ mq_empty;

endmodule
