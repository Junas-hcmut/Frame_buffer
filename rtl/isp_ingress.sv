`timescale 1ns / 1ps

// =====================================================================
// isp_ingress: nhận luồng pixel RGB888 từ ISP (miền clk_isp), qua CDC
// sang clk_sys, đóng gói thành từ 64-bit và sinh lệnh burst ghi DDR.
//
// Giao thức ISP (không có backpressure):
//   isp_valid  : 1 pixel / chu kỳ khi = 1
//   isp_data   : [7:0] R, [15:8] G, [23:16] B  (R ở byte thấp trong DDR)
//   isp_sof    : pixel đầu tiên của khung
//   isp_eol    : pixel cuối của mỗi dòng
//
// Xử lý lỗi:
//  - Async FIFO đầy phía ISP: bỏ pixel tới SOF kế tiếp, gắn cờ err kèm SOF.
//  - Phía sys kiểm tra 640 pixel/dòng, 480 dòng/khung; sai hoặc SOF sớm
//    -> hủy khung: đệm 0 cho đủ burst dở dang, gửi lệnh WR_ABORT, bỏ tới SOF.
//  - Không xin được buffer: bỏ cả khung (cnt_denied trong frame_buf_mgr).
// =====================================================================
module isp_ingress
    import u02_pkg::*;
#(
    parameter int AFIFO_AW    = 4,     // async FIFO 16 pixel
    parameter int WFIFO_DEPTH = 64,    // 64 từ = 4 burst
    parameter int CFIFO_DEPTH = 8
)(
    // ---- Miền clk_isp ----
    input  logic        clk_isp,
    input  logic        rst_isp_n,
    input  logic        isp_valid,
    input  logic [23:0] isp_data,
    input  logic        isp_sof,
    input  logic        isp_eol,

    // ---- Miền clk_sys ----
    input  logic        clk,
    input  logic        rst_n,
    input  logic        cfg_cap_enable,

    // frame_buf_mgr
    output logic        alloc_req,
    input  logic        alloc_ok,
    input  logic [1:0]  alloc_idx,

    // Sang axi_master_wr (FWFT)
    output logic        wd_valid,
    input  logic        wd_ready,
    output logic [63:0] wd_data,
    output logic        wc_valid,
    input  logic        wc_ready,
    output u02_wr_cmd_t wc_cmd,

    // Sự kiện thống kê (xung, miền clk_sys)
    output logic        isp_ovf_evt,      // có pixel bị bỏ phía ISP vì FIFO đầy
    output logic        frame_err_evt     // khung bị hủy vì sai số pixel/dòng
);

    // =================================================================
    // 1. Miền clk_isp: đẩy vào async FIFO, bỏ pixel khi đầy
    // =================================================================
    logic        af_wfull, dropping;
    logic        af_winc;
    logic [26:0] af_wdata;

    assign af_winc  = isp_valid && !af_wfull && (!dropping || isp_sof);
    assign af_wdata = {dropping && isp_sof, isp_sof, isp_eol, isp_data};   // {err, sof, eol, pix}

    always_ff @(posedge clk_isp or negedge rst_isp_n) begin
        if (!rst_isp_n) begin
            dropping <= 1'b0;
        end else if (isp_valid) begin
            if (af_wfull)                    dropping <= 1'b1;
            else if (dropping && isp_sof)    dropping <= 1'b0;
        end
    end

    logic        af_rempty, af_rinc;
    logic [26:0] af_rdata;

    async_fifo #(.DW(27), .AW(AFIFO_AW)) u_afifo (
        .wclk(clk_isp), .wrst_n(rst_isp_n), .winc(af_winc), .wdata(af_wdata), .wfull(af_wfull),
        .rclk(clk),     .rrst_n(rst_n),     .rinc(af_rinc), .rdata(af_rdata), .rempty(af_rempty)
    );

    wire        it_err = af_rdata[26];
    wire        it_sof = af_rdata[25];
    wire        it_eol = af_rdata[24];
    wire [23:0] it_pix = af_rdata[23:0];

    // =================================================================
    // 2. Miền clk_sys
    // =================================================================
    // FIFO dữ liệu và lệnh sang axi_master_wr
    logic        wf_push, wf_full, wf_empty;
    logic [63:0] wf_din;
    logic        cf_push, cf_full, cf_empty;
    u02_wr_cmd_t cf_din;

    sync_fifo #(.DW(64), .DEPTH(WFIFO_DEPTH)) u_wfifo (
        .clk(clk), .rst_n(rst_n), .wr_en(wf_push), .wr_data(wf_din), .full(wf_full),
        .rd_en(wd_ready), .rd_data(wd_data), .empty(wf_empty), .count()
    );
    sync_fifo #(.DW($bits(u02_wr_cmd_t)), .DEPTH(CFIFO_DEPTH)) u_cfifo (
        .clk(clk), .rst_n(rst_n), .wr_en(cf_push), .wr_data(cf_din), .full(cf_full),
        .rd_en(wc_ready), .rd_data(wc_cmd), .empty(cf_empty), .count()
    );
    assign wd_valid = !wf_empty;
    assign wc_valid = !cf_empty;

    typedef enum logic [1:0] { S_WAIT_SOF, S_ACTIVE, S_ABORT, S_ABORT_CMD } st_t;
    st_t st;

    logic [79:0] acc;          // bộ gom byte (tối đa 10 byte)
    logic [3:0]  acc_cnt;
    logic [9:0]  x;            // 0..639
    logic [8:0]  y;            // 0..479
    logic        expect_sof;   // pixel SOF của chính khung vừa cấp buffer
    logic [31:0] base;
    logic [1:0]  buf_idx;
    logic [3:0]  wib;          // số từ trong burst hiện tại
    logic [12:0] burst_idx;    // 0..FRAME_BURSTS-1

    // ---- Đẩy từ 64-bit + sinh lệnh burst ----
    wire last_word_of_burst = (wib == 4'd15);
    wire can_push_word      = !wf_full && (!last_word_of_burst || !cf_full);
    wire push_word_req      = (st == S_ABORT) ? (wib != 4'd0) : (acc_cnt >= 4'd8);
    wire push_word          = push_word_req && can_push_word;

    // ---- Nhận pixel ----
    wire can_add    = (acc_cnt < 4'd8) || push_word;
    wire pix_avail  = !af_rempty;
    wire premature  = it_sof && !expect_sof;                       // SOF khi khung chưa xong
    wire bad_line   = (it_eol && x != 10'd639) || (!it_eol && x == 10'd639);
    wire frame_end  = it_eol && (x == 10'd639) && (y == 9'd479);

    logic add_pix, go_abort;

    always_comb begin
        af_rinc   = 1'b0;
        add_pix   = 1'b0;
        go_abort  = 1'b0;
        alloc_req = 1'b0;
        case (st)
            S_WAIT_SOF: begin
                if (pix_avail) begin
                    if (it_sof && cfg_cap_enable && acc_cnt == 4'd0 && wib == 4'd0) begin
                        alloc_req = 1'b1;               // cấp buffer; pixel SOF xử lý ở S_ACTIVE
                        if (!alloc_ok) af_rinc = 1'b1;  // không có buffer: bỏ khung
                    end else if (!it_sof || !cfg_cap_enable) begin
                        af_rinc = 1'b1;                 // bỏ pixel ngoài khung
                    end
                end
            end
            S_ACTIVE: begin
                if (pix_avail && can_add) begin
                    if (premature) begin
                        go_abort = 1'b1;                // giữ pixel SOF cho khung sau
                    end else if (bad_line) begin
                        af_rinc  = 1'b1;                // bỏ pixel lỗi
                        go_abort = 1'b1;
                    end else begin
                        af_rinc  = 1'b1;
                        add_pix  = 1'b1;
                    end
                end
            end
            default: ;
        endcase
    end

    // Dữ liệu từ và lệnh
    always_comb begin
        wf_push = push_word;
        wf_din  = (st == S_ABORT) ? 64'h0 : acc[63:0];
        cf_push = 1'b0;
        cf_din  = '0;
        if (push_word && last_word_of_burst) begin
            cf_push        = 1'b1;
            cf_din.addr    = base + 32'({burst_idx, 7'b0});
            cf_din.buf_idx = buf_idx;
            cf_din.kind    = (burst_idx == 13'(FRAME_BURSTS - 1) && st != S_ABORT) ? WR_DATA_LAST : WR_DATA;
        end else if (st == S_ABORT_CMD && !cf_full) begin
            cf_push        = 1'b1;
            cf_din.addr    = base;
            cf_din.buf_idx = buf_idx;
            cf_din.kind    = WR_ABORT;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st            <= S_WAIT_SOF;
            acc           <= '0;
            acc_cnt       <= '0;
            x             <= '0;
            y             <= '0;
            expect_sof    <= 1'b0;
            base          <= '0;
            buf_idx       <= '0;
            wib           <= '0;
            burst_idx     <= '0;
            isp_ovf_evt   <= 1'b0;
            frame_err_evt <= 1'b0;
        end else begin
            isp_ovf_evt   <= af_rinc && it_err && it_sof;
            frame_err_evt <= go_abort;

            // ---- Bộ gom byte ----
            begin : packer
                logic [79:0] a;
                logic [3:0]  c;
                a = acc;
                c = acc_cnt;
                if (push_word && st != S_ABORT) begin
                    a = a >> 64;
                    c = c - 4'd8;
                end
                if (add_pix) begin
                    a = a | (80'(it_pix) << (c * 8));
                    c = c + 4'd3;
                end
                if (go_abort) begin
                    a = '0;
                    c = '0;
                end
                acc     <= a;
                acc_cnt <= c;
            end

            // ---- Đếm từ / burst ----
            if (push_word) begin
                wib <= wib + 1'b1;
                if (last_word_of_burst)
                    burst_idx <= burst_idx + 1'b1;
            end

            // ---- FSM ----
            case (st)
                S_WAIT_SOF: begin
                    if (alloc_req && alloc_ok) begin
                        st         <= S_ACTIVE;
                        base       <= fb_base(alloc_idx);
                        buf_idx    <= alloc_idx;
                        burst_idx  <= '0;
                        x          <= '0;
                        y          <= '0;
                        expect_sof <= 1'b1;
                    end
                end
                S_ACTIVE: begin
                    if (go_abort) begin
                        st <= S_ABORT;
                    end else if (add_pix) begin
                        expect_sof <= 1'b0;
                        if (it_eol) begin
                            x <= '0;
                            y <= y + 1'b1;
                        end else begin
                            x <= x + 1'b1;
                        end
                        if (frame_end) st <= S_WAIT_SOF;   // từ cuối sẽ được đẩy nốt ở S_WAIT_SOF
                    end
                end
                S_ABORT: begin
                    // đệm 0 cho tới hết burst dở dang
                    if (wib == 4'd0 || (push_word && last_word_of_burst))
                        st <= S_ABORT_CMD;
                end
                S_ABORT_CMD: begin
                    if (!cf_full) st <= S_WAIT_SOF;
                end
                default: st <= S_WAIT_SOF;
            endcase
        end
    end

endmodule
