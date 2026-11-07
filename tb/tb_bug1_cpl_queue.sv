`timescale 1ns / 1ps

// =====================================================================
// tb_bug1_cpl_queue: tái hiện lỗi 1 - roi_cpl_queue mất completion mà
// không phát ovf_evt khi ghi đúng chu kỳ hàng đầy và có pop.
//
// Chạy: sim/run_all.do
// Kỳ vọng ở commit này (RTL gốc): FAIL. Commit sửa lỗi kế tiếp làm TB PASS.
//
// Tiêu chí (bảo toàn): mỗi entry đã push phải hoặc được pop ra đúng 1 lần,
// hoặc được báo bằng đúng 1 xung ovf_evt. Không có trường hợp thứ ba.
//  - Pha 1 (định hướng): làm đầy hàng bằng B, rồi push_b + pop cùng chu
//    kỳ; sau đó push_a + pop cùng chu kỳ khi đầy.
//  - Pha 2 (ngẫu nhiên): 20.000 chu kỳ push_a/push_b/pop ngẫu nhiên.
// =====================================================================
module tb_bug1_cpl_queue;
    import u02_pkg::*;

    localparam int DEPTH = ROI_QUEUE_DEPTH;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    logic         push_a = 0, push_b = 0, pop = 0;
    u02_roi_cpl_t din_a = '0, din_b = '0, dout;
    logic         valid, full, ovf_evt;
    logic [$clog2(DEPTH):0] count;

    roi_cpl_queue #(.DEPTH(DEPTH)) dut (
        .clk(clk), .rst_n(rst_n),
        .push_a(push_a), .din_a(din_a),
        .push_b(push_b), .din_b(din_b),
        .pop(pop), .dout(dout), .valid(valid), .full(full), .count(count),
        .ovf_evt(ovf_evt)
    );

    // ---------------- Bảng điểm ----------------
    // Mỗi entry mang mã duy nhất {frame_id, roi_id} = tag 16 bit
    int unsigned n_push, n_pop, n_ovf, n_dup;
    bit          seen [65536];             // tag -> đã pop ra chưa
    logic [15:0] tag_cnt;

    function automatic u02_roi_cpl_t mk(input logic [15:0] tag, input logic is_b);
        u02_roi_cpl_t c;
        c          = '0;
        c.roi_id   = tag[7:0];
        c.frame_id = tag[15:8];
        c.status   = is_b ? CPL_INVALID : CPL_OK;
        return c;
    endfunction

    always @(posedge clk) if (rst_n) begin
        if (pop && valid) begin
            n_pop++;
            if (seen[{dout.frame_id, dout.roi_id}]) n_dup++;
            seen[{dout.frame_id, dout.roi_id}] = 1'b1;
        end
        if (ovf_evt) n_ovf++;
    end

    // Chu kỳ bắt tay: đặt tín hiệu sau cạnh lên 1 ns
    task automatic step();
        @(posedge clk); #1;
    endtask

    task automatic idle_inputs();
        push_a = 0; push_b = 0; pop = 0;
    endtask

    task automatic do_b();          // 1 push_b (người gọi tự giữ khoảng cách >= 2 chu kỳ)
        push_b = 1; din_b = mk(tag_cnt, 1'b1); tag_cnt++; n_push++;
    endtask

    task automatic do_a();
        push_a = 1; din_a = mk(tag_cnt, 1'b0); tag_cnt++; n_push++;
    endtask

    int errors;
    int last_b;      // chu kỳ của push_b gần nhất (giữ khoảng cách >= 2)
    int cyc;

    task automatic check_conservation(input string phase);
        int unsigned lost;
        lost = n_push - n_pop - n_ovf;
        $display("[%s] push=%0d  pop=%0d  ovf=%0d  -> mat im lang=%0d, pop trung=%0d",
                 phase, n_push, n_pop, n_ovf, lost, n_dup);
        if (n_push != n_pop + n_ovf || n_dup != 0) errors++;
    endtask

    task automatic drain();
        idle_inputs();
        repeat (3) step();
        while (valid) begin pop = 1; step(); end
        pop = 0;
        repeat (3) step();
    endtask

    initial begin
        errors = 0; tag_cnt = 16'h0100; n_push = 0; n_pop = 0; n_ovf = 0; n_dup = 0;
        repeat (3) step();
        rst_n = 1;
        step();

        // ---------------- Pha 1: định hướng ----------------
        // Làm đầy bằng B (cách nhau 2 chu kỳ như APB)
        for (int i = 0; i < DEPTH; i++) begin
            do_b(); step(); idle_inputs(); step();
        end
        if (count != DEPTH) begin $display("FAIL: count=%0d sau khi lam day", count); errors++; end

        // (a) push_b + pop cùng chu kỳ khi đầy
        do_b(); pop = 1; step(); idle_inputs(); step();
        $display("Pha 1a (full, push_b + pop): count=%0d (ky vong %0d), ovf_evt da thay=%0d",
                 count, DEPTH, n_ovf);

        // (b) push_a + pop cùng chu kỳ khi đầy
        if (count == DEPTH) begin
            do_a(); pop = 1; step(); idle_inputs(); step();
            $display("Pha 1b (full, push_a + pop): count=%0d (ky vong %0d), ovf_evt da thay=%0d",
                     count, DEPTH, n_ovf);
        end
        drain();
        check_conservation("Pha 1");

        // ---------------- Pha 2: ngẫu nhiên ----------------
        last_b = -10;
        for (cyc = 0; cyc < 20000; cyc++) begin
            idle_inputs();
            // A tuân thủ giao thức của từng bản: gốc dùng !full, bản sửa dùng a_room
            if (!full  && ($urandom % 100) < 30) do_a();
            if ((cyc - last_b) >= 2 && ($urandom % 100) < 35) begin do_b(); last_b = cyc; end
            if (valid && ($urandom % 100) < 45) pop = 1;
            step();
        end
        drain();
        check_conservation("Pha 2");

        if (errors == 0) $display("==== tb_bug1_cpl_queue: PASS ====");
        else             $display("==== tb_bug1_cpl_queue: FAIL (%0d loi) ====", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("==== tb_bug1_cpl_queue: FAIL (TIMEOUT) ====");
        $finish;
    end
endmodule
