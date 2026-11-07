`timescale 1ns / 1ps

// =====================================================================
// u02_top: Frame buffer + 2D DMA cho CCTV (ISP -> DDR -> ROI -> U01)
//
//  Nhánh ghi : ISP --(clk_isp)--> isp_ingress --(CDC)--> axi_master_wr --> AXI AW/W/B
//                                     |                        |
//                                     +--- frame_buf_mgr <-----+ (commit / abort)
//  Nhánh đọc : regfile -> roi_desc_queue -> u02_scheduler -> row_cmd_gen
//              row_cmd_gen -> page_splitter -> axi_master_rd --> AXI AR/R
//              row_cmd_gen -> row_info FIFO --+
//              axi_master_rd ------------------+-> axis_crop -> AXIS -> U01
//              axis_crop (roi_done) -> u02_scheduler -> roi_cpl_queue -> regfile
//
//  Một cổng AXI4 master (ghi và đọc độc lập theo kênh), 1 ID.
//  Mọi khối chạy trên clk (clk_sys) trừ phần đầu của isp_ingress.
//
//  06/11/2026: nối giao thức giữ chỗ completion mới
//  (u02_scheduler.cpl_rsv -> roi_cpl_queue.rsv_a, roi_cpl_queue.a_room ->
//  u02_scheduler.cpl_room) thay cho cpl_full.
// =====================================================================
module u02_top
    import u02_pkg::*;
(
    input  logic                          clk,          // clk_sys 150 MHz
    input  logic                          rst_n,
    input  logic                          clk_isp,      // ~25,175 MHz

    // ---------------- APB slave ----------------
    input  logic [11:0]                   paddr,
    input  logic                          psel,
    input  logic                          penable,
    input  logic                          pwrite,
    input  logic [31:0]                   pwdata,
    output logic [31:0]                   prdata,
    output logic                          pready,
    output logic                          pslverr,
    output logic                          irq,

    // ---------------- ISP (miền clk_isp) ----------------
    input  logic                          isp_valid,
    input  logic [23:0]                   isp_data,
    input  logic                          isp_sof,
    input  logic                          isp_eol,

    // ---------------- AXI4 master tới DDR controller ----------------
    output logic [AXI_ID_WIDTH-1:0]       m_axi_awid,
    output logic [AXI_ADDR_WIDTH-1:0]     m_axi_awaddr,
    output logic [7:0]                    m_axi_awlen,
    output logic [2:0]                    m_axi_awsize,
    output logic [1:0]                    m_axi_awburst,
    output logic [3:0]                    m_axi_awcache,
    output logic [2:0]                    m_axi_awprot,
    output logic                          m_axi_awvalid,
    input  logic                          m_axi_awready,
    output logic [AXI_DATA_WIDTH-1:0]     m_axi_wdata,
    output logic [AXI_DATA_WIDTH/8-1:0]   m_axi_wstrb,
    output logic                          m_axi_wlast,
    output logic                          m_axi_wvalid,
    input  logic                          m_axi_wready,
    input  logic [AXI_ID_WIDTH-1:0]       m_axi_bid,
    input  logic [1:0]                    m_axi_bresp,
    input  logic                          m_axi_bvalid,
    output logic                          m_axi_bready,
    output logic [AXI_ID_WIDTH-1:0]       m_axi_arid,
    output logic [AXI_ADDR_WIDTH-1:0]     m_axi_araddr,
    output logic [7:0]                    m_axi_arlen,
    output logic [2:0]                    m_axi_arsize,
    output logic [1:0]                    m_axi_arburst,
    output logic [3:0]                    m_axi_arcache,
    output logic [2:0]                    m_axi_arprot,
    output logic                          m_axi_arvalid,
    input  logic                          m_axi_arready,
    input  logic [AXI_ID_WIDTH-1:0]       m_axi_rid,
    input  logic [AXI_DATA_WIDTH-1:0]     m_axi_rdata,
    input  logic [1:0]                    m_axi_rresp,
    input  logic                          m_axi_rlast,
    input  logic                          m_axi_rvalid,
    output logic                          m_axi_rready,

    // ---------------- AXI4-Stream sang U01 ----------------
    output logic                          m_axis_tvalid,
    input  logic                          m_axis_tready,
    output logic [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0]    m_axis_tkeep,
    output logic                          m_axis_tlast,
    output logic [0:0]                    m_axis_tuser,
    input  logic                          u01_ibuf_release,

    // ---------------- Timestamp chung ----------------
    output logic [TS_WIDTH-1:0]           ts_out
);

    // =================================================================
    // Timestamp & reset miền ISP
    // =================================================================
    logic [TS_WIDTH-1:0] ts;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ts <= '0;
        else        ts <= ts + 1'b1;
    end
    assign ts_out = ts;

    logic rst_isp_n;
    rst_sync u_rst_isp (.clk(clk_isp), .arst_n(rst_n), .rst_n(rst_isp_n));

    // =================================================================
    // Tín hiệu nội bộ
    // =================================================================
    // regfile
    logic                               cfg_enable, cfg_cap_enable;
    logic [7:0]                         cfg_pad_val;
    logic [3:0]                         cfg_ibuf_n;
    logic                               rf_q_push, rf_q_invalid;
    u02_roi_desc_t                      rf_q_desc;

    // roi_desc_queue
    logic                               q_push_ok, q_overflow, q_valid, q_empty, q_full, q_afull, q_pop;
    u02_roi_desc_t                      q_head;
    logic [$clog2(ROI_QUEUE_DEPTH):0]   q_count;

    // roi_cpl_queue
    logic                               sch_cpl_push, cpl_pop, cpl_valid, cpl_full, cpl_ovf;
    logic                               sch_cpl_rsv, cpl_room;
    u02_roi_cpl_t                       sch_cpl, inv_cpl, cpl_head;
    logic [$clog2(ROI_QUEUE_DEPTH):0]   cpl_count;

    // frame_buf_mgr
    logic                               fb_alloc_req, fb_alloc_ok;
    logic [1:0]                         fb_alloc_idx;
    logic [7:0]                         fb_alloc_fid;
    logic                               fb_commit, fb_abort;
    logic [1:0]                         fb_commit_idx, fb_abort_idx;
    logic [7:0]                         fb_lookup_fid, fb_latest_fid;
    logic                               fb_lookup_hit, fb_acq, fb_rel, fb_latest_valid;
    logic [1:0]                         fb_lookup_idx, fb_rel_idx;
    logic [15:0]                        fb_cnt_committed, fb_cnt_aborted, fb_cnt_recycled, fb_cnt_denied;

    // scheduler
    logic                               sch_busy, sch_done;
    logic                               rg_start;
    u02_roi_desc_t                      rg_desc;
    logic [31:0]                        rg_base;
    logic                               rd_cnt_clear;
    logic [19:0]                        rd_beat_cnt;

    // row_cmd_gen
    logic                               rg_busy;
    logic                               rc_valid, rc_ready, rc_eol, rc_eof;
    logic [31:0]                        rc_addr;
    logic [11:0]                        rc_len;
    logic [7:0]                         rc_roi_id, rc_frame_id;
    logic                               rg_ri_valid, rg_ri_ready;
    u02_row_info_t                      rg_ri;

    // row_info FIFO
    logic                               rif_empty, rif_full;
    logic                               ri_valid, ri_ready;
    u02_row_info_t                      ri_head;

    // page_splitter
    logic                               ps_valid, ps_ready;
    logic [31:0]                        ps_addr;
    logic [11:0]                        ps_len;
    logic [7:0]                         ps_arlen, ps_roi_id, ps_frame_id;
    logic                               ps_eol, ps_eof, ps_first, ps_last, ps_split, ps_err_zero;

    // axi_master_rd -> axis_crop
    logic                               d_valid, d_ready, d_row_last, d_err;
    logic [63:0]                        d_data;
    logic [2:0]                         d_lo;
    logic [3:0]                         d_cnt;
    logic                               rd_bus_err;

    // axis_crop
    logic                               roi_done, roi_err;

    // ingress -> axi_master_wr
    logic                               wd_valid, wd_ready, wc_valid, wc_ready;
    logic [63:0]                        wd_data;
    u02_wr_cmd_t                        wc_cmd;
    logic                               wr_bus_err, isp_ovf_evt, frame_err_evt;

    // =================================================================
    // Điều khiển
    // =================================================================
    regfile u_regfile (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite), .pwdata(pwdata),
        .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .cfg_enable(cfg_enable), .cfg_cap_enable(cfg_cap_enable),
        .cfg_pad_val(cfg_pad_val), .cfg_ibuf_n(cfg_ibuf_n),
        .q_push(rf_q_push), .q_desc(rf_q_desc), .q_overflow(q_overflow), .q_invalid(rf_q_invalid),
        .q_full(q_full), .q_empty(q_empty), .q_count(q_count),
        .hw_busy(sch_busy), .hw_done(sch_done), .ts_now(ts),
        .cpl_valid(cpl_valid), .cpl_head(cpl_head), .cpl_count(cpl_count), .cpl_pop(cpl_pop), .cpl_ovf(cpl_ovf),
        .fb_latest_fid(fb_latest_fid), .fb_latest_valid(fb_latest_valid),
        .fb_cnt_committed(fb_cnt_committed), .fb_cnt_aborted(fb_cnt_aborted),
        .fb_cnt_recycled(fb_cnt_recycled), .fb_cnt_denied(fb_cnt_denied),
        .evt_isp_ovf(isp_ovf_evt), .evt_frame_err(frame_err_evt), .evt_bus_err(rd_bus_err | wr_bus_err),
        .irq(irq)
    );

    roi_desc_queue u_roi_q (
        .clk(clk), .rst_n(rst_n),
        .push(rf_q_push), .in_desc(rf_q_desc), .push_ok(q_push_ok), .overflow(q_overflow),
        .pop(q_pop), .out_desc(q_head), .valid(q_valid),
        .empty(q_empty), .full(q_full), .almost_full(q_afull), .count(q_count)
    );

    // Completion cho descriptor sai (từ regfile)
    always_comb begin
        inv_cpl          = '0;
        inv_cpl.roi_id   = rf_q_desc.roi_id;
        inv_cpl.frame_id = rf_q_desc.frame_id;
        inv_cpl.status   = CPL_INVALID;
        inv_cpl.t_submit = rf_q_desc.t_submit;
        inv_cpl.t_start  = ts;
        inv_cpl.t_end    = ts;
    end

    roi_cpl_queue u_cpl_q (
        .clk(clk), .rst_n(rst_n),
        .push_a(sch_cpl_push), .din_a(sch_cpl), .rsv_a(sch_cpl_rsv), .a_room(cpl_room),
        .push_b(rf_q_invalid), .din_b(inv_cpl),
        .pop(cpl_pop), .dout(cpl_head), .valid(cpl_valid), .full(cpl_full), .count(cpl_count),
        .ovf_evt(cpl_ovf)
    );

    frame_buf_mgr u_fbm (
        .clk(clk), .rst_n(rst_n),
        .alloc_req(fb_alloc_req), .alloc_ok(fb_alloc_ok), .alloc_idx(fb_alloc_idx), .alloc_fid(fb_alloc_fid),
        .commit_evt(fb_commit), .commit_idx(fb_commit_idx), .abort_evt(fb_abort), .abort_idx(fb_abort_idx),
        .lookup_fid(fb_lookup_fid), .lookup_hit(fb_lookup_hit), .lookup_idx(fb_lookup_idx),
        .acq_req(fb_acq), .rel_evt(fb_rel), .rel_idx(fb_rel_idx),
        .latest_fid(fb_latest_fid), .latest_valid(fb_latest_valid),
        .cnt_committed(fb_cnt_committed), .cnt_aborted(fb_cnt_aborted),
        .cnt_recycled(fb_cnt_recycled), .cnt_denied(fb_cnt_denied)
    );

    u02_scheduler u_sched (
        .clk(clk), .rst_n(rst_n),
        .cfg_enable(cfg_enable), .cfg_ibuf_n(cfg_ibuf_n), .ts_now(ts),
        .q_valid(q_valid), .q_desc(q_head), .q_pop(q_pop),
        .lookup_fid(fb_lookup_fid), .lookup_hit(fb_lookup_hit), .lookup_idx(fb_lookup_idx),
        .acq_req(fb_acq), .rel_evt(fb_rel), .rel_idx(fb_rel_idx),
        .rg_start(rg_start), .rg_desc(rg_desc), .rg_base(rg_base),
        .roi_done(roi_done), .roi_err(roi_err),
        .rd_cnt_clear(rd_cnt_clear), .rd_beat_cnt(rd_beat_cnt),
        .ibuf_release(u01_ibuf_release),
        .cpl_push(sch_cpl_push), .cpl(sch_cpl), .cpl_room(cpl_room), .cpl_rsv(sch_cpl_rsv),
        .busy(sch_busy), .done_evt(sch_done)
    );

    // =================================================================
    // Nhánh đọc
    // =================================================================
    row_cmd_gen u_rowgen (
        .clk(clk), .rst_n(rst_n),
        .start(rg_start), .desc(rg_desc), .base(rg_base), .busy(rg_busy),
        .rc_valid(rc_valid), .rc_ready(rc_ready), .rc_addr(rc_addr), .rc_len(rc_len),
        .rc_roi_id(rc_roi_id), .rc_frame_id(rc_frame_id), .rc_eol(rc_eol), .rc_eof(rc_eof),
        .ri_valid(rg_ri_valid), .ri_ready(rg_ri_ready), .ri(rg_ri)
    );

    sync_fifo #(.DW($bits(u02_row_info_t)), .DEPTH(8)) u_rowinfo_fifo (
        .clk(clk), .rst_n(rst_n),
        .wr_en(rg_ri_valid), .wr_data(rg_ri), .full(rif_full),
        .rd_en(ri_ready), .rd_data(ri_head), .empty(rif_empty), .count()
    );
    assign rg_ri_ready = !rif_full;
    assign ri_valid    = !rif_empty;

    page_splitter u_splitter (
        .clk(clk), .rst_n(rst_n),
        .s_req_valid(rc_valid), .s_req_ready(rc_ready), .s_req_addr(rc_addr), .s_req_len(rc_len),
        .s_req_roi_id(rc_roi_id), .s_req_frame_id(rc_frame_id), .s_req_eol(rc_eol), .s_req_eof(rc_eof),
        .m_req_valid(ps_valid), .m_req_ready(ps_ready), .m_req_addr(ps_addr), .m_req_len(ps_len),
        .m_req_arlen(ps_arlen), .m_req_roi_id(ps_roi_id), .m_req_frame_id(ps_frame_id),
        .m_req_eol(ps_eol), .m_req_eof(ps_eof), .m_req_first(ps_first), .m_req_last(ps_last),
        .m_req_split(ps_split), .err_zero_len(ps_err_zero)
    );

    axi_master_rd u_rd (
        .clk(clk), .rst_n(rst_n),
        .c_valid(ps_valid), .c_ready(ps_ready), .c_addr(ps_addr), .c_len(ps_len),
        .c_arlen(ps_arlen), .c_row_last(ps_last),
        .arid(m_axi_arid), .araddr(m_axi_araddr), .arlen(m_axi_arlen), .arsize(m_axi_arsize),
        .arburst(m_axi_arburst), .arcache(m_axi_arcache), .arprot(m_axi_arprot),
        .arvalid(m_axi_arvalid), .arready(m_axi_arready),
        .rid(m_axi_rid), .rdata(m_axi_rdata), .rresp(m_axi_rresp), .rlast(m_axi_rlast),
        .rvalid(m_axi_rvalid), .rready(m_axi_rready),
        .d_valid(d_valid), .d_ready(d_ready), .d_data(d_data), .d_lo(d_lo), .d_cnt(d_cnt),
        .d_row_last(d_row_last), .d_err(d_err),
        .cnt_clear(rd_cnt_clear), .beat_cnt(rd_beat_cnt), .bus_err_evt(rd_bus_err)
    );

    axis_crop u_crop (
        .clk(clk), .rst_n(rst_n), .cfg_pad_val(cfg_pad_val),
        .ri_valid(ri_valid), .ri_ready(ri_ready), .ri(ri_head),
        .d_valid(d_valid), .d_ready(d_ready), .d_data(d_data), .d_lo(d_lo), .d_cnt(d_cnt),
        .d_row_last(d_row_last), .d_err(d_err),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready), .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(m_axis_tkeep), .m_axis_tlast(m_axis_tlast), .m_axis_tuser(m_axis_tuser),
        .roi_done(roi_done), .roi_err(roi_err)
    );

    // =================================================================
    // Nhánh ghi
    // =================================================================
    isp_ingress u_ingress (
        .clk_isp(clk_isp), .rst_isp_n(rst_isp_n),
        .isp_valid(isp_valid), .isp_data(isp_data), .isp_sof(isp_sof), .isp_eol(isp_eol),
        .clk(clk), .rst_n(rst_n), .cfg_cap_enable(cfg_cap_enable),
        .alloc_req(fb_alloc_req), .alloc_ok(fb_alloc_ok), .alloc_idx(fb_alloc_idx),
        .wd_valid(wd_valid), .wd_ready(wd_ready), .wd_data(wd_data),
        .wc_valid(wc_valid), .wc_ready(wc_ready), .wc_cmd(wc_cmd),
        .isp_ovf_evt(isp_ovf_evt), .frame_err_evt(frame_err_evt)
    );

    axi_master_wr u_wr (
        .clk(clk), .rst_n(rst_n),
        .wd_valid(wd_valid), .wd_ready(wd_ready), .wd_data(wd_data),
        .wc_valid(wc_valid), .wc_ready(wc_ready), .wc_cmd(wc_cmd),
        .awid(m_axi_awid), .awaddr(m_axi_awaddr), .awlen(m_axi_awlen), .awsize(m_axi_awsize),
        .awburst(m_axi_awburst), .awcache(m_axi_awcache), .awprot(m_axi_awprot),
        .awvalid(m_axi_awvalid), .awready(m_axi_awready),
        .wdata(m_axi_wdata), .wstrb(m_axi_wstrb), .wlast(m_axi_wlast),
        .wvalid(m_axi_wvalid), .wready(m_axi_wready),
        .bid(m_axi_bid), .bresp(m_axi_bresp), .bvalid(m_axi_bvalid), .bready(m_axi_bready),
        .commit_evt(fb_commit), .commit_idx(fb_commit_idx),
        .abort_evt(fb_abort), .abort_idx(fb_abort_idx), .bus_err_evt(wr_bus_err)
    );

    // Tín hiệu chỉ để quan sát / dành cho mở rộng
    wire unused = ^{q_push_ok, q_afull, fb_alloc_fid, cpl_full, rg_busy, ps_roi_id, ps_frame_id,
                    ps_eol, ps_eof, ps_first, ps_split, ps_err_zero};

endmodule
