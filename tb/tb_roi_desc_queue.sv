`timescale 1ns / 1ps
// Kiểm tra roi_desc_queue (bản dùng u02_roi_desc_t, sâu 16):
//  - so dữ liệu/thứ tự với mô hình FIFO tham chiếu
//  - push khi đầy + pop cùng chu kỳ phải được nhận
//  - push khi đầy và không pop phải bật xung overflow (không mất im lặng)
//  - count phải khớp số phần tử trong mô hình
module tb_roi_desc_queue;
    import u02_pkg::*;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    logic push = 0, pop = 0;
    u02_roi_desc_t in_desc, out_desc;
    logic push_ok, overflow, valid, empty, full, almost_full;
    logic [$clog2(ROI_QUEUE_DEPTH):0] count;

    roi_desc_queue dut (
        .clk(clk), .rst_n(rst_n),
        .push(push), .in_desc(in_desc), .push_ok(push_ok), .overflow(overflow),
        .pop(pop), .out_desc(out_desc), .valid(valid),
        .empty(empty), .full(full), .almost_full(almost_full), .count(count)
    );

    u02_roi_desc_t model[$];
    int err_data = 0, err_count = 0, err_ovf = 0;
    int pushes = 0, pops = 0, ovf_expected = 0, full_pop_accepted = 0;

    function automatic u02_roi_desc_t mk(input int s);
        u02_roi_desc_t d;
        d          = '0;
        d.roi_x    = 12'(s * 7 - 300);
        d.roi_y    = 12'(s * 3 - 100);
        d.roi_w    = 8'(16 + s % 113);
        d.roi_h    = 8'(16 + (s * 5) % 113);
        d.dec_log2 = 2'(s % 3);
        d.roi_id   = 8'(s);
        d.frame_id = 8'(s >> 3);
        d.deadline = 48'(s) * 48'd1000;
        d.t_submit = 48'(s);
        return d;
    endfunction

    // Kiểm tra tại cạnh lên, trước khi DUT cập nhật
    always @(posedge clk) begin
        if (rst_n) begin
            if (count != model.size()) err_count++;
            if ((model.size() == 0) != empty) err_count++;
            if (pop && !empty) begin
                if (out_desc !== model[0]) err_data++;
                void'(model.pop_front());
                pops++;
            end
            if (push) begin
                if (model.size() < ROI_QUEUE_DEPTH) begin      // còn chỗ (tính sau pop)
                    if (!push_ok || overflow) err_ovf++;
                    if (full) full_pop_accepted++;
                    model.push_back(in_desc);
                    pushes++;
                end else begin
                    ovf_expected++;
                    if (push_ok || !overflow) err_ovf++;
                end
            end
        end
    end

    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        for (int i = 0; i < 20000; i++) begin
            @(negedge clk);
            push    = ($urandom % 100) < 55;
            pop     = ($urandom % 100) < 45;
            in_desc = mk(i);
        end
        @(negedge clk); push = 0; pop = 0;
        repeat (2) @(posedge clk);
        $display("[tb_roi_desc_queue] push nhan %0d, pop %0d", pushes, pops);
        $display("[tb_roi_desc_queue] loi du lieu/thu tu %0d, loi count/empty %0d, loi co overflow %0d",
                 err_data, err_count, err_ovf);
        $display("[tb_roi_desc_queue] push khi day nhung co pop cung chu ky -> duoc nhan: %0d lan", full_pop_accepted);
        $display("[tb_roi_desc_queue] push bi tu choi (day, khong pop): %0d lan, deu co xung overflow", ovf_expected);
        $display("[tb_roi_desc_queue] %s", (err_data + err_count + err_ovf == 0) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
