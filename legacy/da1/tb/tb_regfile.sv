`timescale 1ns / 1ps
// Kiểm tra regfile + roi_desc_queue qua APB. Mỗi mục in PASS/FAIL.
module tb_regfile;
    import u02_pkg::*;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // Timestamp chạy tự do
    logic [TS_WIDTH-1:0] ts = '0;
    always @(posedge clk) ts <= ts + 1'b1;

    // APB
    logic [11:0] paddr = 0;
    logic [31:0] pwdata = 0, prdata;
    logic        psel = 0, penable = 0, pwrite = 0, pready, pslverr;

    // Nối regfile <-> queue
    logic          q_push, q_invalid, q_overflow, q_push_ok, q_full, q_empty, q_valid, q_afull;
    u02_roi_desc_t q_desc, q_out;
    logic [$clog2(ROI_QUEUE_DEPTH):0] q_count;
    logic          q_pop = 0;
    logic          cfg_enable, irq;
    logic [7:0]    cfg_pad_val;
    logic [3:0]    cfg_ibuf_n;
    logic          hw_busy = 0, hw_done = 0;

    regfile u_rf (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr), .psel(psel), .penable(penable), .pwrite(pwrite), .pwdata(pwdata),
        .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .cfg_enable(cfg_enable), .cfg_pad_val(cfg_pad_val), .cfg_ibuf_n(cfg_ibuf_n),
        .q_push(q_push), .q_desc(q_desc), .q_overflow(q_overflow), .q_invalid(q_invalid),
        .q_full(q_full), .q_empty(q_empty), .q_count(q_count),
        .hw_busy(hw_busy), .hw_done(hw_done), .ts_now(ts), .irq(irq)
    );

    roi_desc_queue u_q (
        .clk(clk), .rst_n(rst_n),
        .push(q_push), .in_desc(q_desc), .push_ok(q_push_ok), .overflow(q_overflow),
        .pop(q_pop), .out_desc(q_out), .valid(q_valid),
        .empty(q_empty), .full(q_full), .almost_full(q_afull), .count(q_count)
    );

    // ---------------- APB tasks ----------------
    logic last_slverr;
    task automatic apb_write(input logic [11:0] a, input logic [31:0] d);
        @(negedge clk); psel = 1; penable = 0; pwrite = 1; paddr = a; pwdata = d;
        @(negedge clk); penable = 1; #1 last_slverr = pslverr;
        @(negedge clk); psel = 0; penable = 0; pwrite = 0;
    endtask

    task automatic apb_read(input logic [11:0] a, output logic [31:0] d);
        @(negedge clk); psel = 1; penable = 0; pwrite = 0; paddr = a;
        @(negedge clk); penable = 1; #1 d = prdata; last_slverr = pslverr;
        @(negedge clk); psel = 0; penable = 0;
    endtask

    task automatic set_roi(input int x, y, w, h, id, fid, dec, input logic [47:0] dl);
        apb_write(12'h020, {4'b0, 12'(y), 4'b0, 12'(x)});
        apb_write(12'h024, {8'b0, 8'(h), 8'b0, 8'(w)});
        apb_write(12'h028, {14'b0, 2'(dec), 8'(fid), 8'(id)});
        apb_write(12'h02C, dl[31:0]);
        apb_write(12'h030, {16'b0, dl[47:32]});
    endtask

    int fails = 0;
    task automatic check(input string name, input logic ok, input string note);
        $display("[tb_regfile] %s  %s %s", ok ? "PASS" : "FAIL", name, ok ? "" : note);
        if (!ok) fails++;
    endtask

    logic [31:0] rd, lo, hi;
    logic [47:0] ts_before;
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;

        // 1. Giá trị mặc định
        apb_read(12'h00C, rd);  check("PAD_VAL mac dinh = 0x00", rd == 32'h0, $sformatf("(%h)", rd));
        apb_read(12'h014, rd);  check("IBUF_N mac dinh = 1", rd == 32'd1, $sformatf("(%0d)", rd));
        apb_read(12'h010, rd);  check("CAP = 0x01108010 (ver 1, depth 16, max 128, min 16)", rd == 32'h0110_8010, $sformatf("(%h)", rd));
        apb_read(12'h004, rd);  check("STATUS sau reset: q_empty=1, count=0, khong co loi", rd == 32'h0000_0008, $sformatf("(%h)", rd));

        // 2. Push ROI hợp lệ, tọa độ âm
        set_roi(-3, 20, 64, 32, 5, 9, 1, 48'h1234_5678_9ABC);
        ts_before = ts;
        apb_write(12'h034, 32'h1);
        repeat (2) @(posedge clk);
        check("ROI hop le vao hang doi (count = 1)", q_count == 1, $sformatf("(count=%0d)", q_count));
        check("descriptor dung (x=-3,y=20,w=64,h=32,id=5,frame=9,dec=1,deadline)",
              q_out.roi_x == -12'sd3 && q_out.roi_y == 12'sd20 && q_out.roi_w == 64 && q_out.roi_h == 32 &&
              q_out.roi_id == 5 && q_out.frame_id == 9 && q_out.dec_log2 == 1 && q_out.deadline == 48'h1234_5678_9ABC,
              $sformatf("(x=%0d y=%0d w=%0d h=%0d id=%0d f=%0d dec=%0d)", q_out.roi_x, q_out.roi_y,
                        q_out.roi_w, q_out.roi_h, q_out.roi_id, q_out.frame_id, q_out.dec_log2));
        check("t_submit nam trong khoang thoi gian push", q_out.t_submit >= ts_before && q_out.t_submit <= ts,
              $sformatf("(t_submit=%0d)", q_out.t_submit));
        apb_read(12'h034, rd);  check("ROI_PUSH doc ra 0 (tu xoa)", rd == 32'h0, $sformatf("(%h)", rd));

        // 3. Descriptor sai: W = 15, rồi dec = 3
        set_roi(0, 0, 15, 32, 6, 9, 0, 48'h0);
        apb_write(12'h034, 32'h1);
        repeat (2) @(posedge clk);
        apb_read(12'h004, rd);
        check("W=15 bi tu choi: err_invalid=1, count van = 1", rd[5] == 1'b1 && q_count == 1,
              $sformatf("(STATUS=%h count=%0d)", rd, q_count));
        apb_write(12'h004, 32'h20);                    // W1C err_invalid
        apb_read(12'h004, rd);
        check("W1C xoa err_invalid", rd[5] == 1'b0, $sformatf("(STATUS=%h)", rd));
        set_roi(0, 0, 32, 32, 7, 9, 3, 48'h0);
        apb_write(12'h034, 32'h1);
        repeat (2) @(posedge clk);
        apb_read(12'h004, rd);
        check("dec=3 bi tu choi", rd[5] == 1'b1 && q_count == 1, $sformatf("(STATUS=%h count=%0d)", rd, q_count));
        apb_write(12'h004, 32'h20);

        // 4. Làm đầy hàng đợi rồi push thêm -> err_overflow
        set_roi(10, 10, 16, 16, 8, 9, 0, 48'h0);
        for (int i = 0; i < ROI_QUEUE_DEPTH - 1; i++) apb_write(12'h034, 32'h1);
        repeat (2) @(posedge clk);
        check("hang doi day sau 16 ROI (q_full=1)", q_full == 1'b1 && q_count == 16, $sformatf("(count=%0d)", q_count));
        apb_write(12'h034, 32'h1);
        repeat (2) @(posedge clk);
        apb_read(12'h004, rd);
        check("push khi day -> err_overflow=1, count van 16", rd[4] == 1'b1 && q_count == 16 && rd[2] == 1'b1,
              $sformatf("(STATUS=%h count=%0d)", rd, q_count));
        check("STATUS.q_count = 16", rd[12:8] == 5'd16, $sformatf("(%0d)", rd[12:8]));
        apb_write(12'h004, 32'h10);

        // 5. done sticky + IRQ
        apb_write(12'h008, 32'h02);                    // bật IRQ done
        @(negedge clk); hw_done = 1; @(negedge clk); hw_done = 0;
        repeat (5) @(posedge clk);
        apb_read(12'h004, rd);
        check("STATUS.done giu lai sau xung hw_done", rd[1] == 1'b1, $sformatf("(STATUS=%h)", rd));
        check("irq = 1 khi done va IRQ_EN.done = 1", irq == 1'b1, "(irq = 0)");
        apb_write(12'h004, 32'h02);
        apb_read(12'h004, rd);
        check("W1C xoa done va irq ve 0", rd[1] == 1'b0 && irq == 1'b0, $sformatf("(STATUS=%h irq=%b)", rd, irq));

        // 6. Sự kiện trùng chu kỳ với lệnh xóa: sự kiện phải thắng
        fork
            apb_write(12'h004, 32'h02);
            begin @(negedge clk); @(negedge clk); hw_done = 1; @(negedge clk); hw_done = 0; end
        join
        apb_read(12'h004, rd);
        check("hw_done trung chu ky voi W1C khong bi mat", rd[1] == 1'b1, $sformatf("(STATUS=%h)", rd));
        apb_write(12'h004, 32'h02);

        // 7. Địa chỉ ngoài bản đồ
        apb_read(12'h110, rd);
        check("0x110 ngoai ban do -> PSLVERR=1, PRDATA=0", last_slverr == 1'b1 && rd == 32'h0,
              $sformatf("(pslverr=%b prdata=%h)", last_slverr, rd));
        apb_read(12'h010, rd);
        check("0x010 hop le -> PSLVERR=0", last_slverr == 1'b0, "(pslverr=1)");

        // 8. Đọc timestamp 48 bit nhất quán
        apb_read(12'h038, lo);
        apb_read(12'h03C, hi);
        check("TS_HI:TS_LO doc nhat quan", {hi[15:0], lo} <= ts && {hi[15:0], lo} + 48'd100 > ts,
              $sformatf("(doc=%0d ts=%0d)", {hi[15:0], lo}, ts));

        // 9. Pop hết hàng đợi -> q_empty
        @(negedge clk); q_pop = 1;
        repeat (16) @(negedge clk);
        q_pop = 0;
        apb_read(12'h004, rd);
        check("pop het -> q_empty=1, count=0", rd[3] == 1'b1 && rd[12:8] == 0, $sformatf("(STATUS=%h)", rd));

        $display("[tb_regfile] tong FAIL: %0d", fails);
        $finish;
    end
endmodule
