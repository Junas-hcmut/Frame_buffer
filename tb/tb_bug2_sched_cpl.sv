`timescale 1ns / 1ps

// =====================================================================
// tb_bug2_sched_cpl: tái hiện lỗi 2 - u02_scheduler đẩy completion lúc
// finish mà không kiểm hàng completion; CPL_INVALID đến trong lúc ROI
// đang chạy làm đầy hàng -> completion OK của ROI bị mất.
//
// DUT = u02_scheduler + roi_cpl_queue (nối như u02_top). frame_buf_mgr,
// roi_desc_queue, row_cmd_gen/axis_crop được thay bằng mô hình đơn giản:
//   - hàng descriptor: FWFT trong TB
//   - lookup_hit = !frame_id[7]   (frame_id[7] = 1 -> STALE)
//   - deadline = 1 -> luôn quá hạn (DROPPED_LATE); deadline = 0 -> không hạn
//   - roi_done: xung sau 3..32 chu kỳ kể từ khi busy
//
// Chạy: sim/run_all.do
// Kỳ vọng ở commit này (RTL gốc): FAIL. Commit sửa lỗi kế tiếp làm TB PASS.
//
// Tiêu chí:
//   (1) Mỗi ROI scheduler đã lấy khỏi hàng descriptor có ĐÚNG 1 completion
//       với status đúng (LATE / STALE / OK|BUS_ERR). Không được mất.
//   (2) Mỗi CPL_INVALID đã push: được pop ra hoặc được báo bằng ovf_evt.
// =====================================================================
module tb_bug2_sched_cpl;
    import u02_pkg::*;

    localparam int DEPTH = ROI_QUEUE_DEPTH;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    logic [TS_WIDTH-1:0] ts = '0;
    always @(posedge clk) ts <= ts + 1'b1;

    // ---------------- Tín hiệu DUT ----------------
    logic                cfg_enable = 1'b0;
    logic                q_valid, q_pop;
    u02_roi_desc_t       q_desc;
    logic [7:0]          lookup_fid;
    logic                lookup_hit;
    logic                acq_req, rel_evt;
    logic [1:0]          rel_idx, cur_idx_unused;
    logic                rg_start;
    u02_roi_desc_t       rg_desc;
    logic [31:0]         rg_base;
    logic                roi_done = 1'b0, roi_err = 1'b0;
    logic                rd_cnt_clear;
    logic                ibuf_release = 1'b0;
    logic                sch_push, busy, done_evt;
    u02_roi_cpl_t        sch_cpl;

    logic                push_b = 1'b0, pop = 1'b0;
    u02_roi_cpl_t        din_b = '0, dout;
    logic                valid, full, ovf_evt;
    logic [$clog2(DEPTH):0] count;

    assign lookup_hit = !lookup_fid[7];

    u02_scheduler u_sched (
        .clk(clk), .rst_n(rst_n),
        .cfg_enable(cfg_enable), .cfg_ibuf_n(4'd15), .ts_now(ts),
        .q_valid(q_valid), .q_desc(q_desc), .q_pop(q_pop),
        .lookup_fid(lookup_fid), .lookup_hit(lookup_hit), .lookup_idx(2'd0),
        .acq_req(acq_req), .rel_evt(rel_evt), .rel_idx(rel_idx),
        .rg_start(rg_start), .rg_desc(rg_desc), .rg_base(rg_base),
        .roi_done(roi_done), .roi_err(roi_err),
        .rd_cnt_clear(rd_cnt_clear), .rd_beat_cnt(20'd10),
        .ibuf_release(ibuf_release),
        .cpl_push(sch_push), .cpl(sch_cpl), .cpl_full(full),
        .busy(busy), .done_evt(done_evt)
    );

    roi_cpl_queue #(.DEPTH(DEPTH)) u_cpl (
        .clk(clk), .rst_n(rst_n),
        .push_a(sch_push), .din_a(sch_cpl),
        .push_b(push_b), .din_b(din_b),
        .pop(pop), .dout(dout), .valid(valid), .full(full), .count(count),
        .ovf_evt(ovf_evt)
    );

    // ---------------- Hàng descriptor (FWFT, dùng sync_fifo) ----------------
    logic          dq_wr = 1'b0, dq_empty;
    u02_roi_desc_t dq_din = '0;
    logic [4:0]    dq_count;
    sync_fifo #(.DW($bits(u02_roi_desc_t)), .DEPTH(16)) u_dq (
        .clk(clk), .rst_n(rst_n), .wr_en(dq_wr), .wr_data(dq_din), .full(),
        .rd_en(q_pop), .rd_data(q_desc), .empty(dq_empty), .count(dq_count)
    );
    assign q_valid = !dq_empty;

    // Trạng thái kỳ vọng theo roi_id (id dùng vòng 0..255, mỗi lúc < 64 ROI chưa xong)
    cpl_status_t exp_st  [256];
    int          exp_cnt [256];       // số ROI đã lấy khỏi hàng, chưa có completion
    int unsigned n_roi_taken, n_roi_cpl, n_bad_status, n_extra;
    int unsigned n_b_push, n_b_pop, n_ovf;
    int unsigned n_aa_ok = 0;      // số completion CPL_OK của ROI 0xAA (pha 1)
    bit          dbg = 0;
    logic [7:0]  next_id = 8'd0;

    task automatic enq_roi(input logic [7:0] id, input bit late, input bit stale);
        u02_roi_desc_t d;
        d          = '0;
        d.roi_w    = 8'd16;
        d.roi_h    = 8'd16;
        d.roi_id   = id;
        d.frame_id = stale ? 8'h80 : 8'h01;
        d.deadline = late ? 48'd1 : 48'd0;
        dq_din = d;
        dq_wr  = 1'b1;            // người gọi hạ dq_wr ở chu kỳ sau
        exp_st[id] = late ? CPL_DROPPED_LATE : (stale ? CPL_STALE : CPL_OK);
    endtask

    always @(posedge clk) if (rst_n) begin
        if (q_pop) begin
            exp_cnt[q_desc.roi_id]++;
            n_roi_taken++;
        end
        if (pop && valid) begin
            if (dbg) $display("  t=%0t pop roi_id=%h fid=%h st=%0d", $time, dout.roi_id, dout.frame_id, dout.status);
            if (dout.status == CPL_INVALID) begin
                n_b_pop++;
            end else begin
                n_roi_cpl++;
                if (exp_cnt[dout.roi_id] == 0) n_extra++;
                else exp_cnt[dout.roi_id]--;
                if ((exp_st[dout.roi_id] == CPL_OK) ? (dout.status != CPL_OK && dout.status != CPL_BUS_ERR)
                                                    : (dout.status != exp_st[dout.roi_id])) begin
                    n_bad_status++;
                    if (n_bad_status <= 5)
                        $display("  t=%0t sai status: roi_id=%0d nhan status %0d, ky vong %0d", $time,
                                 dout.roi_id, dout.status, exp_st[dout.roi_id]);
                end
                if (dout.roi_id == 8'hAA && dout.status == CPL_OK) n_aa_ok++;
            end
        end
        if (ovf_evt) n_ovf++;
    end

    // ---------------- Mô hình đầu đọc (roi_done) ----------------
    bit auto_done = 1'b0;
    bit armed     = 1'b0;
    int dly;
    always @(posedge clk) begin
        roi_done     <= 1'b0;
        ibuf_release <= roi_done;               // trả credit 1 chu kỳ sau
        if (rst_n && auto_done && busy && !roi_done) begin
            if (!armed) begin
                armed <= 1'b1;
                dly   <= 2 + ($urandom % 30);
            end else if (dly == 0) begin
                roi_done <= 1'b1;
                roi_err  <= (($urandom % 10) == 0);
                armed    <= 1'b0;
            end else begin
                dly <= dly - 1;
            end
        end
    end

    // ---------------- Tiện ích ----------------
    logic [15:0] b_tag = 16'h0;
    task automatic step(); @(posedge clk); #1; endtask

    task automatic push_inv();             // 1 CPL_INVALID, giữ 1 chu kỳ
        din_b          = '0;
        din_b.status   = CPL_INVALID;
        din_b.roi_id   = b_tag[7:0];
        din_b.frame_id = b_tag[15:8];
        b_tag++;
        push_b = 1'b1; n_b_push++;
    endtask

    int errors = 0;

    task automatic report(input string phase);
        int lost_roi;
        lost_roi = 0;
        for (int i = 0; i < 256; i++) lost_roi += exp_cnt[i];
        $display("[%s] ROI lay khoi hang=%0d, completion ROI=%0d, ROI MAT completion=%0d, sai status=%0d, thua=%0d",
                 phase, n_roi_taken, n_roi_cpl, lost_roi, n_bad_status, n_extra);
        $display("[%s] INVALID push=%0d, pop=%0d, ovf=%0d -> INVALID mat im lang=%0d",
                 phase, n_b_push, n_b_pop, n_ovf, int'(n_b_push) - int'(n_b_pop) - int'(n_ovf));
        if (lost_roi != 0 || n_bad_status != 0 || n_extra != 0) errors++;
        if (n_b_push != n_b_pop + n_ovf) errors++;
    endtask

    task automatic drain();
        int guard;
        push_b = 0; guard = 0;
        while ((q_valid || busy) && guard < 100000) begin
            pop = valid; step(); pop = 0; guard++;
        end
        repeat (5) step();
        while (valid) begin pop = 1; step(); end
        pop = 0;
        repeat (5) step();
    endtask


    initial begin
        for (int i = 0; i < 256; i++) begin exp_cnt[i] = 0; exp_st[i] = CPL_OK; end
        n_roi_taken = 0; n_roi_cpl = 0; n_bad_status = 0; n_extra = 0;
        n_b_push = 0; n_b_pop = 0; n_ovf = 0;
        repeat (3) step();
        rst_n = 1;
        step();

        // ================= Pha 1: định hướng =================
        // 1. 15 CPL_INVALID (cách nhau 2 chu kỳ như APB) -> còn 1 chỗ
        for (int i = 0; i < DEPTH - 1; i++) begin push_inv(); step(); push_b = 0; step(); end
        $display("Pha 1: sau 15 INVALID, count=%0d", count);

        // 2. ROI 0xAA vào hàng, bật scheduler -> dispatch (còn 1 chỗ)
        enq_roi(8'hAA, 1'b0, 1'b0);
        step(); dq_wr = 1'b0;
        cfg_enable = 1'b1;
        while (!busy) step();
        $display("Pha 1: ROI 0xAA dang chay (busy=1), count=%0d", count);

        // 3. 1 CPL_INVALID trong lúc ROI chạy
        push_inv(); step(); push_b = 0; repeat (4) step();
        $display("Pha 1: sau INVALID thu 16 trong luc ROI chay, count=%0d, ovf=%0d", count, n_ovf);

        // 4. ROI xong
        roi_done = 1'b1; step(); roi_done = 1'b0; ibuf_release = 1'b1; step(); ibuf_release = 1'b0;
        repeat (4) step();
        $display("Pha 1: sau roi_done, count=%0d, ovf=%0d", count, n_ovf);

        drain();
        if (n_aa_ok == 0) $display("Pha 1: KHONG THAY completion CPL_OK cua ROI 0xAA -> phan mem cho mai");
        else          $display("Pha 1: co completion CPL_OK cua ROI 0xAA");
        report("Pha 1");
        if (n_aa_ok != 1) errors++;

        // ================= Pha 2: ngẫu nhiên =================
        auto_done = 1'b1;
        begin
            int last_b;
            last_b = -10;
            next_id = 8'd0;
            for (int cyc = 0; cyc < 60000; cyc++) begin
                push_b = 0; pop = 0; dq_wr = 0;
                if (dq_count < 5'd8 && ($urandom % 100) < 20) begin
                    int r;
                    r = $urandom % 100;
                    enq_roi(next_id, r < 15, r >= 15 && r < 30);
                    next_id++;
                end
                if ((cyc - last_b) >= 2 && ($urandom % 100) < 20) begin push_inv(); last_b = cyc; end
                if (valid && ($urandom % 100) < 30) pop = 1;
                step();
            end
        end
        drain();
        report("Pha 2 (cong don)");

        if (errors == 0) $display("==== tb_bug2_sched_cpl: PASS ====");
        else             $display("==== tb_bug2_sched_cpl: FAIL (%0d loi) ====", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("==== tb_bug2_sched_cpl: FAIL (TIMEOUT) ====");
        $finish;
    end
endmodule
