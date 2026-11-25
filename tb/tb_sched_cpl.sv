`timescale 1ns / 1ps

// =====================================================================
// tb_sched_cpl: kiểm u02_scheduler + hàng completion (sync_fifo) của bản
// thu gọn ĐA2.
//
// DUT = u02_scheduler + sync_fifo DEPTH 16 (nối như u02_top). Các khối khác
// được thay bằng mô hình đơn giản:
//   - hàng descriptor: roi_desc_queue (RTL thật)
//   - lookup_hit = !frame_id[7]   (frame_id[7] = 1 -> ROI STALE)
//   - roi_done: xung sau 3..40 chu kỳ kể từ khi busy; roi_err ngẫu nhiên 5 %
//   - U01: ibuf_release sau 1..60 chu kỳ kể từ mỗi lần dispatch
//   - phần mềm: lấy completion ngẫu nhiên, có những đoạn dài không lấy để
//     hàng completion đầy.
//
// Tiêu chí:
//   (1) ROI STALE: đúng 1 xung stale_evt mang đúng roi_id/frame_id, không có completion.
//   (2) ROI hợp lệ: đúng 1 completion, đúng roi_id, frame_id, status, bytes_stream.
//   (3) Không bao giờ ghi vào hàng completion khi đầy; không dispatch khi đầy.
//   (4) Số credit đang dùng không vượt cfg_ibuf_n.
// =====================================================================
module tb_sched_cpl;
    import u02_pkg::*;

    localparam int DEPTH = ROI_QUEUE_DEPTH;
    localparam int N_ROI = 20000;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    logic                cfg_enable = 1'b0;
    logic [3:0]          cfg_ibuf_n = 4'd2;
    logic                q_valid, q_pop;
    u02_roi_desc_t       q_desc;
    logic [7:0]          lookup_fid;
    logic                lookup_hit;
    fb_idx_t             lookup_idx;
    logic                acq_req, rel_evt;
    fb_idx_t             rel_idx;
    logic                rg_start;
    u02_roi_desc_t       rg_desc;
    logic [31:0]         rg_base;
    logic                roi_done = 1'b0, roi_err = 1'b0;
    logic                rd_cnt_clear;
    logic                ibuf_release = 1'b0;
    logic                cpl_push, busy, done_evt;
    u02_roi_cpl_t        cpl;
    logic                stale_evt;
    logic [7:0]          stale_roi_id, stale_frame_id;

    logic                pop = 1'b0, cpl_full, cpl_empty;
    u02_roi_cpl_t        cpl_head;
    logic [$clog2(DEPTH):0] cpl_count;

    assign lookup_hit = !lookup_fid[7];
    assign lookup_idx = fb_idx_t'(lookup_fid[0]);

    u02_scheduler u_sched (
        .clk(clk), .rst_n(rst_n), .cfg_enable(cfg_enable), .cfg_ibuf_n(cfg_ibuf_n),
        .q_valid(q_valid), .q_desc(q_desc), .q_pop(q_pop),
        .lookup_fid(lookup_fid), .lookup_hit(lookup_hit), .lookup_idx(lookup_idx),
        .acq_req(acq_req), .rel_evt(rel_evt), .rel_idx(rel_idx),
        .rg_start(rg_start), .rg_desc(rg_desc), .rg_base(rg_base),
        .roi_done(roi_done), .roi_err(roi_err),
        .rd_cnt_clear(rd_cnt_clear), .rd_beat_cnt(20'd100),
        .ibuf_release(ibuf_release),
        .cpl_push(cpl_push), .cpl(cpl), .cpl_full(cpl_full),
        .stale_evt(stale_evt), .stale_roi_id(stale_roi_id), .stale_frame_id(stale_frame_id),
        .busy(busy), .done_evt(done_evt)
    );

    sync_fifo #(.DW($bits(u02_roi_cpl_t)), .DEPTH(DEPTH)) u_cpl_q (
        .clk(clk), .rst_n(rst_n), .wr_en(cpl_push), .wr_data(cpl), .full(cpl_full),
        .rd_en(pop), .rd_data(cpl_head), .empty(cpl_empty), .count(cpl_count)
    );

    // ---------------- Hàng descriptor (RTL) ----------------
    logic          dq_push = 1'b0, dq_full;
    u02_roi_desc_t dq_din = '0;
    roi_desc_queue u_dq (
        .clk(clk), .rst_n(rst_n), .push(dq_push), .in_desc(dq_din), .push_ok(), .overflow(),
        .pop(q_pop), .out_desc(q_desc), .valid(q_valid),
        .empty(), .full(dq_full), .almost_full(), .count()
    );

    // ---------------- Bảng kỳ vọng ----------------
    int  n_cpl_got [int], n_stale_got [int];
    bit  exp_err [int];
    u02_roi_desc_t sent [int];
    int  errors = 0, n_push = 0, n_ok = 0, n_bus = 0, n_stale = 0, max_count = 0, n_full_cyc = 0;
    int  used = 0;
    function automatic void err(input string s);
        errors++; if (errors <= 10) $display("[%0t] LOI %s", $time, s);
    endfunction

    // ---------------- Mô hình đường đọc (roi_done) ----------------
    int run_left = -1, cur_id = -1;
    always @(posedge clk) if (rst_n) begin
        roi_done <= 1'b0; roi_err <= 1'b0;
        if (rg_start) begin
            run_left <= $urandom_range(3, 40); cur_id <= int'(rg_desc.roi_id);
        end else if (run_left > 0) run_left <= run_left - 1;
        else if (run_left == 0) begin
            roi_done <= 1'b1;
            roi_err  <= exp_err[cur_id];
            run_left <= -1;
        end
    end

    // ---------------- Mô hình U01 (credit) ----------------
    int rel_due[$], cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;
    always @(posedge clk) if (rst_n) begin
        ibuf_release <= 1'b0;
        if (rg_start) rel_due.push_back(cyc + $urandom_range(1, 60));
        if (rel_due.size() > 0 && cyc >= rel_due[0]) begin void'(rel_due.pop_front()); ibuf_release <= 1'b1; end
    end

    // ---------------- Kiểm tra mỗi chu kỳ ----------------
    always @(posedge clk) if (rst_n) begin
        if (cpl_push && cpl_full) err("ghi completion khi hang day");
        if (rg_start && cpl_full) err("dispatch khi hang completion day");
        if (cpl_count > max_count) max_count = cpl_count;
        if (cpl_full) n_full_cyc++;
        if (rg_start) used++;
        if (ibuf_release && used > 0) used--;
        if (used > cfg_ibuf_n) err($sformatf("credit dang dung %0d > %0d", used, cfg_ibuf_n));
        if (stale_evt) begin
            int id; id = int'(stale_roi_id);
            if (!sent.exists(id)) err("stale_evt cua ROI la");
            else begin
                if (!sent[id].frame_id[7]) err($sformatf("ROI %0d hop le nhung bi STALE", id));
                if (stale_frame_id != sent[id].frame_id) err("stale_frame_id sai");
                n_stale_got[id]++;
                n_stale++;
            end
        end
    end

    // ---------------- Phần mềm lấy completion ----------------
    always @(posedge clk) if (rst_n) begin
        int id;
        pop <= 1'b0;
        if (!cpl_empty && !pop && (($urandom % 100) < (((cyc / 3000) % 3 == 0) ? 0 : 30))) begin
            pop <= 1'b1;
            id = int'(cpl_head.roi_id);
            if (!sent.exists(id)) err("completion cua ROI la");
            else begin
                if (sent[id].frame_id[7]) err($sformatf("ROI %0d STALE nhung co completion", id));
                if (cpl_head.frame_id != sent[id].frame_id) err("frame_id trong completion sai");
                if (cpl_head.status != (exp_err[id] ? CPL_BUS_ERR : CPL_OK)) err($sformatf("ROI %0d status sai", id));
                if (cpl_head.bytes_stream != 16'(sent[id].roi_w * sent[id].roi_h * 3)) err("bytes_stream sai");
                n_cpl_got[id]++;
                if (cpl_head.status == CPL_OK) n_ok++; else n_bus++;
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk); cfg_enable = 1'b1;
        for (int k = 0; k < N_ROI; k++) begin
            u02_roi_desc_t d;
            int id;
            id = k;                                         // roi_id duy nhất (dùng 8 bit thấp + bảng theo k % 256)
            while (dq_full) @(negedge clk);
            d = '0;
            d.roi_w = 8'($urandom_range(16, 128)); d.roi_h = 8'($urandom_range(16, 128));
            d.roi_id = 8'(k);
            d.frame_id = 8'($urandom_range(0, 255));
            // chờ ROI cũ cùng roi_id (k - 256) xong hẳn trước khi dùng lại id
            if (k >= 256) begin
                int old; old = k % 256;
                while (n_cpl_got[old] + n_stale_got[old] == 0) @(negedge clk);
                n_cpl_got.delete(old); n_stale_got.delete(old);
            end
            sent[k % 256]    = d;
            exp_err[k % 256] = (($urandom % 100) < 5);
            dq_din = d; dq_push = 1'b1;
            @(negedge clk); dq_push = 1'b0;
            n_push++;
            if (($urandom % 4) == 0) repeat ($urandom_range(1, 30)) @(negedge clk);
        end
        // xả hết
        repeat (200000) @(negedge clk);
        foreach (sent[i]) if (n_cpl_got.exists(i) || n_stale_got.exists(i)) begin
            int t; t = (n_cpl_got.exists(i) ? n_cpl_got[i] : 0) + (n_stale_got.exists(i) ? n_stale_got[i] : 0);
            if (t != 1) err($sformatf("ROI %0d co %0d ket qua", i, t));
        end else err($sformatf("ROI %0d khong co ket qua (mat)", i));
        if (n_ok + n_bus + n_stale != n_push) err($sformatf("tong ket qua %0d != so ROI %0d", n_ok + n_bus + n_stale, n_push));
        $display("tb_sched_cpl: ROI=%0d OK=%0d BUS_ERR=%0d STALE=%0d | hang completion day %0d chu ky, max count=%0d",
                 n_push, n_ok, n_bus, n_stale, n_full_cyc, max_count);
        if (errors == 0) $display("==== tb_sched_cpl: PASS ====");
        else             $display("==== tb_sched_cpl: FAIL (%0d loi) ====", errors);
        $finish;
    end
endmodule
