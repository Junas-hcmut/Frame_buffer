`timescale 1ns / 1ps
// Hồi quy addr_gen_2d: 6 ca cố định + 400 ROI ngẫu nhiên, so với golden model tọa độ.
module tb_addr_gen_2d;
    import u02_pkg::*;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    logic start = 0, busy, done, ov, orr, pad, eol, eof, hp;
    logic [31:0] addr;
    logic signed [11:0] x, y;
    logic [11:0] w, h, ow, oh;
    logic [1:0]  dec, odec;
    logic [7:0]  rid, fid;

    addr_gen_2d dut (
        .clk(clk), .rst_n(rst_n), .start(start), .busy(busy), .done(done),
        .cfg_base_addr(FB0_BASE_ADDR), .cfg_roi_x(x), .cfg_roi_y(y), .cfg_roi_w(w), .cfg_roi_h(h),
        .cfg_dec_log2(dec), .cfg_roi_id(8'd7), .cfg_frame_id(8'd3),
        .out_valid(ov), .out_ready(orr), .out_addr(addr), .out_pad(pad), .out_eol(eol), .out_eof(eof),
        .out_has_pad(hp), .out_w(ow), .out_h(oh), .out_dec_log2(odec), .out_roi_id(rid), .out_frame_id(fid)
    );

    always @(posedge clk) orr <= ($urandom % 4) != 0;   // backpressure ngẫu nhiên 25%

    int k, n_err;
    task automatic run(input int X, Y, WW, HH, DL, input bit verbose, input string name);
        int sx, sy, d; logic epad, ehp; logic [31:0] eaddr;
        x = X; y = Y; w = WW; h = HH; dec = DL; d = 1 << DL;
        k = 0; n_err = 0;
        @(negedge clk); start = 1; @(negedge clk); start = 0;
        forever begin
            @(posedge clk);
            if (ov && orr) begin
                sx = X + (k % WW) * d;  sy = Y + (k / WW) * d;
                epad  = (sx < 0 || sy < 0 || sx >= FRAME_WIDTH || sy >= FRAME_HEIGHT);
                eaddr = FB0_BASE_ADDR + sy * FRAME_STRIDE_B + sx * PIXEL_BYTES;
                if (pad !== epad) n_err++;
                if (!epad && addr !== eaddr) n_err++;
                if (eol !== ((k % WW) == WW - 1) || eof !== (k == WW * HH - 1)) n_err++;
                k++;
                if (eof) break;
            end
            if (k > 4 * WW * HH + 10) break;
        end
        ehp = (X < 0 || Y < 0 || X + (WW << DL) > FRAME_WIDTH || Y + (HH << DL) > FRAME_HEIGHT);
        if (hp !== ehp) n_err++;
        if (k != WW * HH) n_err++;
        if (verbose)
            $display("[tb_addr_gen_2d] %-30s pixel ra %0d (ky vong %0d), loi %0d", name, k, WW * HH, n_err);
        repeat (2) @(posedge clk);
    endtask

    int tot_err = 0, tot_pix = 0;
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        run(100,  50, 16, 16, 0, 1, "trong bien, dec=1");        tot_err += n_err;
        run( -3,  -2, 16, 16, 0, 1, "tran trai+tren, dec=1");    tot_err += n_err;
        run(630, 470, 16, 16, 0, 1, "tran phai+duoi, dec=1");    tot_err += n_err;
        run(100,  50, 16, 16, 1, 1, "trong bien, dec=2");        tot_err += n_err;
        run(200, 100, 32, 64, 2, 1, "trong bien, dec=4");        tot_err += n_err;
        run(-10, 300, 64,128, 1, 1, "tran trai, dec=2");         tot_err += n_err;
        for (int r = 0; r < 400; r++) begin
            run(-300 + int'($urandom % 1000), -300 + int'($urandom % 800),
                16 + int'($urandom % 113), 16 + int'($urandom % 113), int'($urandom % 3), 0, "rand");
            tot_err += n_err; tot_pix += k;
        end
        $display("[tb_addr_gen_2d] 400 ROI ngau nhien: %0d pixel, tong loi (ca 6 ca co dinh) %0d", tot_pix, tot_err);
        $finish;
    end
endmodule
