`timescale 1ns / 1ps

// =====================================================================
// axi_master_wr: ghi frame vào DDR qua AXI4 (AW/W/B).
//  - Mỗi lệnh WR_DATA/WR_DATA_LAST = 1 burst 16 beat x 8 B, căn 128 B.
//  - Dữ liệu của burst đã nằm sẵn trong FIFO trước khi lệnh xuất hiện
//    (isp_ingress đẩy lệnh sau từ thứ 16), nên W không bao giờ bị rỗng
//    giữa burst -> không giữ kênh W của interconnect.
//  - W được phát ngay khi lệnh đã nạp vào thanh ghi AW, không chờ AWREADY
//    (AXI4 A3.3.1: master không được chờ AWREADY rồi mới bật WVALID).
//  - Tối đa MAX_OUTSTANDING burst chờ BRESP.
//  - Khung chỉ commit khi BRESP của burst cuối về và không có BRESP lỗi;
//    có lỗi -> abort. Lệnh WR_ABORT chờ mọi burst trước xong rồi mới hủy.
// =====================================================================
module axi_master_wr
    import u02_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // Từ isp_ingress (FWFT)
    input  logic        wd_valid,
    output logic        wd_ready,
    input  logic [63:0] wd_data,
    input  logic        wc_valid,
    output logic        wc_ready,
    input  u02_wr_cmd_t wc_cmd,

    // AXI4 AW
    output logic [AXI_ID_WIDTH-1:0]   awid,
    output logic [AXI_ADDR_WIDTH-1:0] awaddr,
    output logic [7:0]                awlen,
    output logic [2:0]                awsize,
    output logic [1:0]                awburst,
    output logic [3:0]                awcache,
    output logic [2:0]                awprot,
    output logic                      awvalid,
    input  logic                      awready,
    // AXI4 W
    output logic [AXI_DATA_WIDTH-1:0]   wdata,
    output logic [AXI_DATA_WIDTH/8-1:0] wstrb,
    output logic                        wlast,
    output logic                        wvalid,
    input  logic                        wready,
    // AXI4 B
    input  logic [AXI_ID_WIDTH-1:0]   bid,
    input  logic [1:0]                bresp,
    input  logic                      bvalid,
    output logic                      bready,

    // Sang frame_buf_mgr
    output logic        commit_evt,
    output logic [1:0]  commit_idx,
    output logic        abort_evt,
    output logic [1:0]  abort_idx,
    output logic        bus_err_evt
);

    localparam int OW = $clog2(MAX_OUTSTANDING + 1);

    assign awid    = '0;
    assign awlen   = 8'(MAX_BURST_BEATS - 1);
    assign awsize  = 3'($clog2(AXI_DATA_WIDTH / 8));
    assign awburst = 2'b01;      // INCR
    assign awcache = 4'b0011;    // normal, non-cacheable, bufferable
    assign awprot  = 3'b000;
    assign bready  = 1'b1;

    // ---------------- Hàng đợi meta chờ BRESP ----------------
    logic       mq_push, mq_pop, mq_full, mq_empty;
    logic [3:0] mq_din, mq_dout;             // {buf_idx, kind}

    sync_fifo #(.DW(4), .DEPTH(4)) u_meta (
        .clk(clk), .rst_n(rst_n), .wr_en(mq_push), .wr_data(mq_din), .full(mq_full),
        .rd_en(mq_pop), .rd_data(mq_dout), .empty(mq_empty), .count()
    );

    logic [OW-1:0] ost;          // burst đã nạp AW, chưa có BRESP
    logic [OW-1:0] wq;           // burst đã nạp AW, chưa gửi xong W
    logic [3:0]    wbeat;
    logic [2:0]    buf_err;      // đã có BRESP lỗi cho buffer i

    wire aw_free   = !awvalid || awready;
    wire is_abort  = (wc_cmd.kind == WR_ABORT);
    wire b_hs      = bvalid;     // bready = 1

    // Nạp AW / xử lý WR_ABORT
    wire aw_load   = wc_valid && !is_abort && aw_free && (ost < OW'(MAX_OUTSTANDING)) && !mq_full;
    wire abort_go  = wc_valid &&  is_abort && (ost == '0) && !awvalid && (wq == '0);

    assign wc_ready = aw_load || abort_go;
    assign mq_push  = aw_load;
    assign mq_din   = {wc_cmd.buf_idx, wc_cmd.kind};
    assign mq_pop   = b_hs;

    // W
    assign wvalid   = (wq != '0) && wd_valid;
    assign wdata    = wd_data;
    assign wstrb    = '1;
    assign wlast    = (wbeat == 4'(MAX_BURST_BEATS - 1));
    assign wd_ready = (wq != '0) && wready;
    wire   w_hs     = wvalid && wready;

    // B
    logic [1:0] b_buf;
    wr_kind_t   b_kind;
    logic       b_err;
    assign b_buf  = mq_dout[3:2];
    assign b_kind = wr_kind_t'(mq_dout[1:0]);
    assign b_err  = (bresp != 2'b00);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awvalid     <= 1'b0;
            awaddr      <= '0;
            ost         <= '0;
            wq          <= '0;
            wbeat       <= '0;
            buf_err     <= '0;
            commit_evt  <= 1'b0;
            commit_idx  <= '0;
            abort_evt   <= 1'b0;
            abort_idx   <= '0;
            bus_err_evt <= 1'b0;
        end else begin
            commit_evt  <= 1'b0;
            abort_evt   <= 1'b0;
            bus_err_evt <= b_hs && b_err;

            // AW
            if (aw_load) begin
                awvalid <= 1'b1;
                awaddr  <= wc_cmd.addr;
            end else if (awready) begin
                awvalid <= 1'b0;
            end

            ost <= ost + OW'(aw_load) - OW'(b_hs);
            wq  <= wq  + OW'(aw_load) - OW'(w_hs && wlast);   // W không chờ AWREADY (AXI4 A3.3.1)

            if (w_hs) wbeat <= wlast ? '0 : wbeat + 1'b1;

            // BRESP
            if (b_hs) begin
                if (b_kind == WR_DATA_LAST) begin
                    if (buf_err[b_buf] || b_err) begin
                        abort_evt <= 1'b1;
                        abort_idx <= b_buf;
                    end else begin
                        commit_evt <= 1'b1;
                        commit_idx <= b_buf;
                    end
                    buf_err[b_buf] <= 1'b0;
                end else if (b_err) begin
                    buf_err[b_buf] <= 1'b1;
                end
            end

            // WR_ABORT: mọi burst trước đã xong
            if (abort_go) begin
                abort_evt          <= 1'b1;
                abort_idx          <= wc_cmd.buf_idx;
                buf_err[wc_cmd.buf_idx] <= 1'b0;
            end
        end
    end

    // bid không dùng (1 ID)
    wire unused_bid = ^bid;

endmodule
