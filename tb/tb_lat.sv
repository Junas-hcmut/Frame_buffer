`timescale 1ns/1ps
// tb_lat: đo độ trễ U02 trong điều kiện lý tưởng, xác định
//  - DDR: ARREADY = AWREADY = WREADY = 1, R về sau đúng RLAT chu kỳ, 1 beat/chu kỳ; B sau BLAT chu kỳ
//  - U01: TREADY = 1, ibuf_release ngay sau TLAST
//  - ISP: 1 khung VGA (để có khung commit), rồi các ROI có hướng, mỗi lần 1 ROI
//  Tham số: +RLAT=n (trễ đọc DDR, mặc định 30)  +BLAT=n (trễ B, mặc định 20)
//           +FASTISP : clk_isp 75 MHz, không có vùng xóa -> mô phỏng nhanh ~4 lần
//                      (chỉ dùng khi quan tâm nhánh đọc; số đo nhánh ghi khi đó không phải VGA)
//  ModelSim/Questa: do run_lat.do     Verilator: xem run_lat_verilator.sh
module tb_lat;
    import u02_pkg::*;
    logic clk = 0, clk_isp = 0, rst_n = 0;
    always #3.333 clk = ~clk;
    real isp_half = 19.861;                 // 25,175 MHz
    bit  FASTISP = 0;
    always #(isp_half) clk_isp = ~clk_isp;
    int RLAT = 30, BLAT = 20;

    logic [11:0] paddr = 0; logic psel = 0, penable = 0, pwrite = 0; logic [31:0] pwdata = 0, prdata; logic pready, pslverr, irq;
    logic isp_valid = 0, isp_sof = 0, isp_eol = 0; logic [23:0] isp_data = 0;
    logic [0:0] awid, bid, arid, rid; logic [31:0] awaddr, araddr; logic [7:0] awlen, arlen;
    logic [2:0] awsize, arsize, awprot, arprot; logic [1:0] awburst, arburst, bresp, rresp; logic [3:0] awcache, arcache;
    logic awvalid, wvalid, wlast, bvalid, bready, arvalid, rvalid, rready, rlast;
    logic awready = 1, wready = 1, arready = 1;
    logic [63:0] wdata, rdata; logic [7:0] wstrb;
    logic tvalid, tlast; logic tready = 1; logic [63:0] tdata; logic [7:0] tkeep; logic [0:0] tuser;
    logic ibuf_release = 0; logic [47:0] ts_out;

    u02_top dut (.clk, .rst_n, .clk_isp,
        .paddr, .psel, .penable, .pwrite, .pwdata, .prdata, .pready, .pslverr, .irq,
        .isp_valid, .isp_data, .isp_sof, .isp_eol,
        .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
        .m_axi_awcache(awcache), .m_axi_awprot(awprot), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bid(bid), .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize), .m_axi_arburst(arburst),
        .m_axi_arcache(arcache), .m_axi_arprot(arprot), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rid(rid), .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast), .m_axi_rvalid(rvalid), .m_axi_rready(rready),
        .m_axis_tvalid(tvalid), .m_axis_tready(tready), .m_axis_tdata(tdata), .m_axis_tkeep(tkeep),
        .m_axis_tlast(tlast), .m_axis_tuser(tuser), .u01_ibuf_release(ibuf_release), .ts_out(ts_out));

    int cyc = 0; always @(posedge clk) cyc <= cyc + 1;

    // ---------------- DDR ----------------
    int b_due[$], wb = 0, aw_cnt = 0, b_cnt = 0;
    always @(posedge clk) if (!rst_n) begin bvalid <= 0; bresp <= 0; bid <= 0; end else begin
        if (awvalid && awready) aw_cnt++;
        if (wvalid && wready) begin wb++; if (wlast) begin b_due.push_back(cyc + BLAT); end end
        if (bvalid) begin bvalid <= 0; b_cnt++; end
        else if (b_due.size() > 0 && cyc >= b_due[0]) begin void'(b_due.pop_front()); bvalid <= 1; end
    end
    typedef struct { int len; int due; } ar_t;
    ar_t arq[$]; int rbeat = 0, r_free = 0;
    always @(posedge clk) if (!rst_n) rvalid <= 0; else begin
        if (arvalid && arready) begin
            ar_t t; t.len = arlen + 1; t.due = cyc + RLAT; arq.push_back(t);
        end
        if (arq.size() > 0 && cyc >= arq[0].due) begin
            rvalid <= 1; rdata <= 64'(cyc); rresp <= 0; rid <= 0; rlast <= (rbeat == arq[0].len - 1);
            if (rbeat == arq[0].len - 1) begin void'(arq.pop_front()); rbeat = 0; end else rbeat++;
        end else rvalid <= 0;
    end

    // ---------------- U01 ----------------
    always @(posedge clk) begin
        ibuf_release <= 0;
        if (tvalid && tready && tlast) ibuf_release <= 1;
    end

    // ---------------- ISP ----------------
    int sof_cyc = -1, lastpix_cyc = -1, commit_cyc = -1;
    int first_aw_cyc = -1, first_b_cyc = -1, burst0_last_pix_cyc = -1;
    task automatic isp_frame_send();
        for (int y = 0; y < 480; y++) begin
            for (int x = 0; x < 640; x++) begin
                @(negedge clk_isp);
                isp_valid = 1; isp_data = 24'(x + y); isp_sof = (x == 0 && y == 0); isp_eol = (x == 639);
                if (x == 0 && y == 0) sof_cyc = cyc;
                if (x == 42 && y == 0) burst0_last_pix_cyc = cyc;   // pixel 42 chứa byte 127 (byte 126..128)
                if (x == 639 && y == 479) lastpix_cyc = cyc;
            end
            @(negedge clk_isp); isp_valid = 0; isp_sof = 0; isp_eol = 0;
            if (!FASTISP) repeat (159) @(negedge clk_isp);
        end
        repeat (10) @(negedge clk_isp);
    endtask
    always @(posedge clk) begin
        if (awvalid && awready && first_aw_cyc < 0) first_aw_cyc = cyc;
        if (bvalid && first_b_cyc < 0) first_b_cyc = cyc;
        if (dut.u_fbm.commit_evt && commit_cyc < 0) commit_cyc = cyc;
    end

    // ---------------- APB ----------------
    int push_cyc;
    task automatic apb_wr(input logic [11:0] a, input logic [31:0] d);
        @(negedge clk); paddr = a; pwdata = d; pwrite = 1; psel = 1; penable = 0;
        @(negedge clk); penable = 1;
        @(posedge clk); if (a == 12'h034) push_cyc = cyc;
        @(negedge clk); psel = 0; penable = 0; pwrite = 0;
    endtask
    task automatic apb_rd(input logic [11:0] a, output logic [31:0] d);
        @(negedge clk); paddr = a; pwrite = 0; psel = 1; penable = 0;
        @(negedge clk); penable = 1;
        @(posedge clk); d = prdata;
        @(negedge clk); psel = 0; penable = 0;
    endtask

    // ---------------- Đo 1 ROI ----------------
    int c_disp, c_ar, c_r, c_hdr, c_pay, n_beats_axis, n_ar, n_r;
    int p_cyc, p_rv, p_dstall, p_dempty, p_credit, p_ost, p_cvalid0, p_crop_add, p_crop_noadd_bytes;
    bit armed = 0;
    always @(posedge clk) if (armed) begin
        if (dut.u_sched.rg_start && c_disp < 0) c_disp = cyc;
        if (c_disp >= 0 && dut.u_sched.busy) begin
            p_cyc++;
            if (rvalid) p_rv++;
            if (dut.u_rd.d_valid && !dut.u_rd.d_ready) p_dstall++;
            if (!dut.u_rd.d_valid) p_dempty++;
            if (dut.u_rd.c_valid && !dut.u_rd.c_ready && dut.u_rd.credit < dut.u_rd.need) p_credit++;
            if (dut.u_rd.c_valid && !dut.u_rd.c_ready && dut.u_rd.ost == 4) p_ost++;
            if (!dut.u_rd.c_valid) p_cvalid0++;
            if (dut.u_crop.ck_hs) p_crop_add += dut.u_crop.ck_cnt;   // byte axis_crop nhận vào packer
        end
        if (arvalid && arready) begin n_ar++; if (c_ar < 0) c_ar = cyc; end
        if (rvalid && rready) begin n_r++; if (c_r < 0) c_r = cyc; end
        if (tvalid && tready) begin
            n_beats_axis++;
            if (tuser[0] && c_hdr < 0) c_hdr = cyc;
            else if (!tuser[0] && c_pay < 0) c_pay = cyc;
        end
    end
    task automatic run_roi(input int x, input int y, input int w, input int h, input int dl, input int fid, input string name);
        logic [31:0] id, a, b, c, d, e, f, bd, bs; int k;
        p_cyc = 0; p_rv = 0; p_dstall = 0; p_dempty = 0; p_credit = 0; p_ost = 0; p_cvalid0 = 0; p_crop_add = 0;
        c_disp = -1; c_ar = -1; c_r = -1; c_hdr = -1; c_pay = -1; n_beats_axis = 0; n_ar = 0; n_r = 0;
        armed = 1;
        apb_wr(12'h020, {4'b0, 12'(y), 4'b0, 12'(x)});
        apb_wr(12'h024, {8'b0, 8'(h), 8'b0, 8'(w)});
        apb_wr(12'h028, {14'b0, 2'(dl), 8'(fid), 8'h11});
        apb_wr(12'h02C, 0); apb_wr(12'h030, 0);
        apb_wr(12'h034, 1);
        k = 0;
        do begin apb_rd(12'h040, id); k++; end while (!id[31] && k < 100000);
        apb_rd(12'h044, a); apb_rd(12'h048, b); apb_rd(12'h04C, c); apb_rd(12'h050, d);
        apb_rd(12'h054, e); apb_rd(12'h058, f); apb_rd(12'h05C, bd); apb_rd(12'h060, bs);
        apb_wr(12'h064, 1);
        armed = 0;
        $display("%s | st=%0d push->disp=%0d disp->AR=%0d AR->R=%0d disp->hdr=%0d disp->pay1=%0d (-1 = không có) | tstart-tsub=%0d tend-tstart=%0d (%.2f us) | AR=%0d Rbeats=%0d AXISbeats=%0d W*H=%0d bytes_ddr=%0d",
                 name, id[18:16], c_disp - push_cyc, (c_ar < 0) ? -1 : c_ar - c_disp, (c_ar < 0) ? -1 : c_r - c_ar, c_hdr - c_disp, c_pay - c_disp,
                 c - a, e - c, (e - c) * 6.6667 / 1000.0, n_ar, n_r, n_beats_axis, w * h, bd);
        $display("      prof: busy=%0d rvalid=%0d crop_full(d_valid&!d_ready)=%0d rd_buf_empty=%0d AR_wait_credit=%0d AR_wait_ost=%0d no_cmd_from_splitter=%0d crop_bytes=%0d",
                 p_cyc, p_rv, p_dstall, p_dempty, p_credit, p_ost, p_cvalid0, p_crop_add);
        repeat (50) @(negedge clk);
    endtask

    initial begin
        logic [31:0] st; int fid;
        if ($value$plusargs("RLAT=%d", RLAT)) ;
        if ($value$plusargs("BLAT=%d", BLAT)) ;
        FASTISP = $test$plusargs("FASTISP");
        if (FASTISP) isp_half = 6.667;      // 75 MHz
        repeat (10) @(negedge clk); rst_n = 1; repeat (10) @(negedge clk);
        apb_wr(12'h014, 4); apb_wr(12'h000, 3);
        isp_frame_send();
        repeat (2000) @(negedge clk);
        if (FASTISP) $display("=== NHÁNH GHI (RLAT=%0d, BLAT=%0d, FASTISP: thời gian khung KHÔNG phải VGA) ===", RLAT, BLAT);
        else         $display("=== NHÁNH GHI (RLAT=%0d, BLAT=%0d) ===", RLAT, BLAT);
        $display("SOF -> pixel cuối khung     : %0d chu kỳ clk_sys = %.3f ms", lastpix_cyc - sof_cyc, (lastpix_cyc - sof_cyc) * 6.666 / 1e6);
        $display("pixel cuối -> commit_evt    : %0d chu kỳ", commit_cyc - lastpix_cyc);
        $display("SOF -> commit               : %.4f ms", (commit_cyc - sof_cyc) * 6.666 / 1e6);
        $display("pixel 42 (byte 127) -> AW đầu: %0d chu kỳ ; AW đầu -> B đầu: %0d chu kỳ", first_aw_cyc - burst0_last_pix_cyc, first_b_cyc - first_aw_cyc);
        apb_rd(12'h070, st); fid = int'(st[7:0]);
        $display("=== NHÁNH ĐỌC ===");
        run_roi(100, 100, 128, 128, 0, fid, "128x128 dec1 trong");
        run_roi(96, 96, 128, 128, 0, fid, "128x128 dec1 x%8=0");
        run_roi(100, 100, 128, 128, 1, fid, "128x128 dec2");
        run_roi(50, 10, 128, 112, 2, fid, "128x112 dec4");
        run_roi(100, 100, 64, 64, 0, fid, "64x64 dec1");
        run_roi(100, 100, 16, 16, 0, fid, "16x16 dec1");
        run_roi(-5, 100, 16, 16, 1, fid, "16x16 dec2 x=-5");
        run_roi(-200, 100, 16, 16, 0, fid, "16x16 toàn pad");
        $finish;
    end
endmodule
