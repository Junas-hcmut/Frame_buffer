`timescale 1ns / 1ps

// =====================================================================
// tb_bug3_fbm_age: tái hiện lỗi 3 - frame_buf_mgr dùng age 16 bit, sau
// 65.536 lần commit bộ đếm tràn vòng -> khung MỚI NHẤT bị thu hồi.
//
// Chạy: sim/run_all.do
// RTL trước commit "Fix ..." (lịch sử git) làm TB này FAIL; RTL hiện tại PASS.
//
// Mô hình tham chiếu trong TB giữ thứ tự commit chính xác (danh sách),
// không dùng bộ đếm hữu hạn. Mỗi lần alloc:
//   - còn buffer FREE  -> phải chọn FREE chỉ số nhỏ nhất
//   - không còn FREE   -> phải thu hồi buffer READY cũ nhất không bị đọc
// Pha 1: 70.000 khung liên tiếp (writer ghi, không có reader).
// Pha 2: reader giữ (READING) khung mới nhất trong 70.000 khung tiếp theo
//        rồi nhả; khung đó phải là khung bị thu hồi tiếp theo vì nó cũ
//        nhất. (Bản "so sánh age_cnt - age[i]" cũng sai ở pha này.)
// Pha 3: ngẫu nhiên alloc / commit / abort / acquire / release.
// =====================================================================
module tb_bug3_fbm_age;
    import u02_pkg::*;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    logic        alloc_req = 0, alloc_ok;
    logic [1:0]  alloc_idx;
    logic [7:0]  alloc_fid;
    logic        commit_evt = 0, abort_evt = 0;
    logic [1:0]  commit_idx = 0, abort_idx = 0;
    logic [7:0]  lookup_fid = 0;
    logic        lookup_hit;
    logic [1:0]  lookup_idx;
    logic        acq_req = 0, rel_evt = 0;
    logic [1:0]  rel_idx = 0;
    logic [7:0]  latest_fid;
    logic        latest_valid;
    logic [15:0] cnt_committed, cnt_aborted, cnt_recycled, cnt_denied;

    frame_buf_mgr dut (.*);

    // ---------------- Mô hình tham chiếu ----------------
    int m_st  [NUM_FB];          // 0 FREE, 1 WRITING, 2 READY, 3 READING
    int m_rc  [NUM_FB];
    longint m_seq [NUM_FB];      // số thứ tự commit (64 bit, không tràn)
    longint seq_cnt;

    int errors, n_alloc, first_err_frame;

    function automatic int exp_alloc();
        int best;
        best = -1;
        for (int i = 0; i < NUM_FB; i++) if (m_st[i] == 0) return i;
        for (int i = 0; i < NUM_FB; i++)
            if (m_st[i] == 2 && m_rc[i] == 0 && (best < 0 || m_seq[i] < m_seq[best])) best = i;
        return best;
    endfunction

    task automatic step(); @(posedge clk); #1; endtask

    // Cấp 1 buffer + kiểm tra lựa chọn. Trả về chỉ số được cấp (-1 nếu bị từ chối)
    task automatic do_alloc(output int idx);
        int e;
        e = exp_alloc();
        alloc_req = 1;
        #0;                                   // cho mạch tổ hợp ổn định
        #1;
        if ((e < 0) != !alloc_ok || (alloc_ok && int'(alloc_idx) != e)) begin
            errors++;
            if (first_err_frame < 0) first_err_frame = n_alloc;
            if (errors <= 5)
                $display("  LOI lan alloc thu %0d: DUT chon idx=%0d (ok=%0b), ky vong idx=%0d | seq=[%0d %0d %0d]",
                         n_alloc, alloc_idx, alloc_ok, e, m_seq[0], m_seq[1], m_seq[2]);
        end
        idx = alloc_ok ? int'(alloc_idx) : -1;
        n_alloc++;
        step();
        alloc_req = 0;
        if (idx >= 0) m_st[idx] = 1;
    endtask

    task automatic do_commit(input int idx);
        commit_evt = 1; commit_idx = 2'(idx);
        step();
        commit_evt = 0;
        m_st[idx]  = 2;
        m_seq[idx] = seq_cnt++;
    endtask

    task automatic do_abort(input int idx);
        abort_evt = 1; abort_idx = 2'(idx);
        step();
        abort_evt = 0;
        m_st[idx] = 0;
    endtask

    // Reader chiếm buffer có fid = f (nếu tra thấy)
    task automatic do_acquire(input logic [7:0] f, output int idx);
        lookup_fid = f;
        acq_req    = 1;
        #1;
        idx = lookup_hit ? int'(lookup_idx) : -1;
        step();
        acq_req = 0;
        if (idx >= 0) begin m_st[idx] = 3; m_rc[idx]++; end
    endtask

    task automatic do_release(input int idx);
        rel_evt = 1; rel_idx = 2'(idx);
        step();
        rel_evt = 0;
        if (m_rc[idx] > 0) begin
            m_rc[idx]--;
            if (m_rc[idx] == 0) m_st[idx] = 2;
        end
    endtask

    task automatic run_frames(input int n);
        int idx;
        for (int k = 0; k < n; k++) begin
            do_alloc(idx);
            if (idx >= 0) do_commit(idx);
        end
    endtask

    initial begin
        int idx, pin_idx, e0;
        logic [7:0] pin_fid;
        errors = 0; n_alloc = 0; seq_cnt = 0; first_err_frame = -1;
        for (int i = 0; i < NUM_FB; i++) begin m_st[i] = 0; m_rc[i] = 0; m_seq[i] = 0; end
        repeat (3) step();
        rst_n = 1;
        step();

        // ---------------- Pha 1 ----------------
        run_frames(70000);
        $display("Pha 1: %0d khung, cnt_committed=%0d (16 bit), loi=%0d, loi dau tien o lan alloc %0d",
                 seq_cnt, cnt_committed, errors, first_err_frame);

        // ---------------- Pha 2: reader giữ khung mới nhất ----------------
        e0 = errors;
        pin_fid = latest_fid;
        do_acquire(pin_fid, pin_idx);
        if (pin_idx < 0) begin $display("Pha 2: khong acquire duoc khung moi nhat"); errors++; end
        run_frames(70000);
        do_release(pin_idx);
        do_alloc(idx);                         // phải thu hồi chính pin_idx (cũ nhất)
        if (idx != pin_idx) $display("Pha 2: thu hoi idx=%0d, ky vong idx=%0d (khung bi giu lau nhat)", idx, pin_idx);
        if (idx >= 0) do_commit(idx);
        $display("Pha 2: loi them=%0d", errors - e0);

        // ---------------- Pha 3: ngẫu nhiên ----------------
        e0 = errors;
        begin
            int wr_idx, rd_list [$];
            wr_idx = -1;
            for (int k = 0; k < 50000; k++) begin
                int r;
                r = $urandom % 100;
                if (wr_idx < 0 && r < 30) begin
                    do_alloc(wr_idx);
                end else if (wr_idx >= 0 && r < 45) begin
                    if (($urandom % 10) == 0) do_abort(wr_idx); else do_commit(wr_idx);
                    wr_idx = -1;
                end else if (r < 70 && latest_valid && rd_list.size() < 3) begin
                    do_acquire(latest_fid, idx);
                    if (idx >= 0) rd_list.push_back(idx);
                end else if (rd_list.size() > 0) begin
                    do_release(rd_list.pop_front());
                end else begin
                    step();
                end
            end
        end
        $display("Pha 3: loi them=%0d", errors - e0);

        if (errors == 0) $display("==== tb_bug3_fbm_age: PASS ====");
        else             $display("==== tb_bug3_fbm_age: FAIL (%0d loi) ====", errors);
        $finish;
    end
endmodule
