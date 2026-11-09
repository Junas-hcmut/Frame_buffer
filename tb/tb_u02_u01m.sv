`timescale 1ns/1ps
// =====================================================================
// tb_u02_u01m: testbench toàn hệ thống U02 dùng u01_model.sv thay cho U01 thật
//  (bản tách từ tb_u02_top: phần U01 nay là module riêng, TB chỉ còn scoreboard thụ động)
//  - Nguồn ISP 640x480 có xóa ngang/dọc kiểu VGA; pixel = hàm(khung, x, y)
//  - Mô hình DDR AXI4 slave: 1 bộ nhớ chung, trễ 20-40 chu kỳ, READY ngẫu nhiên
//      +AW_WAIT_W : slave chỉ bật AWREADY khi đã thấy WVALID (hợp lệ theo AXI4 A3.3.1)
//  - U01 = u01_model (TREADY, credit, thời gian tính, kiểm giao thức)
//  - Scoreboard: nghe trộm AXIS, so từng byte với ảnh gốc pix(khung, x, y)
//  - CPU qua APB: đẩy ROI ngẫu nhiên (tràn biên, dec 1/2/4, ROI sai, frame cũ, deadline)
//    và đọc completion; đối chiếu trạng thái với dữ liệu nhận được.
//  - Kiểm tra giao thức: AW/AR/W/AXIS giữ ổn định khi bị chặn, 4 KB, WLAST, TKEEP, TUSER.
//  Tham số: +FRAMES=n  +AW_WAIT_W  +PADVAL=hh  + các +U01_* của u01_model (xem u01_model.sv)
// =====================================================================
module tb_u02_u01m;
    import u02_pkg::*;
    logic clk = 0, clk_isp = 0, rst_n = 0;
    always #3.333 clk = ~clk;          // 150 MHz
    always #19.861 clk_isp = ~clk_isp; // 25,175 MHz

    // ---------------- DUT ----------------
    logic [11:0] paddr; logic psel = 0, penable = 0, pwrite = 0; logic [31:0] pwdata = 0, prdata; logic pready, pslverr, irq;
    logic isp_valid = 0, isp_sof = 0, isp_eol = 0; logic [23:0] isp_data = 0;
    logic [0:0] awid, bid, arid, rid; logic [31:0] awaddr, araddr; logic [7:0] awlen, arlen;
    logic [2:0] awsize, arsize, awprot, arprot; logic [1:0] awburst, arburst, bresp, rresp; logic [3:0] awcache, arcache;
    logic awvalid, awready, wvalid, wready, wlast, bvalid, bready, arvalid, arready, rvalid, rready, rlast;
    logic [63:0] wdata, rdata; logic [7:0] wstrb;
    logic tvalid, tready, tlast; logic [63:0] tdata; logic [7:0] tkeep; logic [0:0] tuser;
    logic ibuf_release; logic [47:0] ts_out;

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

    // ---------------- U01 (mô hình) ----------------
    //   Khi có U01 thật: thay "u01_model u01" bằng module U01 thật, giữ nguyên các nối dây
    int u01_recv, u01_done, u01_err;
    u01_model u01 (.clk, .rst_n,
        .s_axis_tvalid(tvalid), .s_axis_tready(tready), .s_axis_tdata(tdata), .s_axis_tkeep(tkeep),
        .s_axis_tlast(tlast), .s_axis_tuser(tuser), .u01_ibuf_release(ibuf_release),
        .roi_recv(u01_recv), .roi_done(u01_done), .errors(u01_err));

    // ---------------- Cấu hình ----------------
    int  N_FRAMES = 3, IBUF_N = 1; bit AW_WAIT_W = 0; logic [7:0] PADV = 8'h00;
    int  errors = 0, cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;
    function automatic void err(input string s);
        errors++; if (errors <= 25) $display("[%0t] ERR %s", $time, s);
    endfunction

    // ---------------- Nội dung ảnh ----------------
    function automatic logic [23:0] pix(input int f, input int x, input int y);
        logic [7:0] r, g, b;
        r = 8'(x + f*3); g = 8'(y ^ (x >> 2)); b = 8'(f*37 + (y >> 1) + (x >> 8));
        return {b, g, r};
    endfunction

    int isp_frame = -1;                 // khung ISP đang phát
    int fid2frame [int];                // frame_id -> số khung ISP
    always @(posedge clk) if (dut.u_fbm.alloc_req && dut.u_fbm.alloc_ok)
        fid2frame[int'(dut.u_fbm.alloc_fid)] = isp_frame;

    // ---------------- Bộ nhớ DDR (AXI4 slave) ----------------
    logic [63:0] mem [logic [28:0]];
    function automatic logic [63:0] rdw(input logic [28:0] a);
        return mem.exists(a) ? mem[a] : 64'h0;
    endfunction

    // --- Ghi ---
    typedef struct { logic [31:0] addr; int beats; } aw_t;
    aw_t awq[$]; logic [63:0] wbq[$]; bit wlq[$];
    int b_due[$];
    logic aw_stall = 0, w_stall = 0; logic [31:0] aw_h; logic [63:0] w_h; logic wl_h;
    logic rnd_aw, rnd_w, rnd_ar, rnd_r, rnd_t;
    always @(posedge clk) begin
        rnd_aw <= ($urandom_range(0,99) < 60); rnd_w <= ($urandom_range(0,99) < 75);
        rnd_ar <= ($urandom_range(0,99) < 60); rnd_r <= ($urandom_range(0,99) < 80);
        rnd_t  <= ($urandom_range(0,99) < 70);
    end
    assign awready = AW_WAIT_W ? (rnd_aw && wvalid) : rnd_aw;
    assign wready  = rnd_w;

    always @(posedge clk) if (!rst_n) begin bvalid <= 0; bresp <= 0; bid <= 0; end else begin
        // ổn định khi bị chặn
        if (aw_stall && (!awvalid || awaddr != aw_h)) err("AW thay đổi khi bị chặn");
        if (w_stall  && (!wvalid || wdata != w_h || wlast != wl_h)) err("W thay đổi khi bị chặn");
        aw_stall <= awvalid && !awready; aw_h <= awaddr;
        w_stall  <= wvalid && !wready;   w_h  <= wdata; wl_h <= wlast;
        if (awvalid && awready) begin
            aw_t t;
            if (awlen != 15 || awaddr[6:0] != 0) err($sformatf("AW lạ addr=%h len=%0d", awaddr, awlen));
            t.addr = awaddr; t.beats = 0; awq.push_back(t);
        end
        if (wvalid && wready) begin wbq.push_back(wdata); wlq.push_back(wlast); end
        // ghép W với AW theo thứ tự
        if (awq.size() > 0 && wbq.size() > 0) begin
            logic [63:0] d; bit l;
            d = wbq.pop_front(); l = wlq.pop_front();
            mem[29'((awq[0].addr >> 3) + awq[0].beats)] = d;
            awq[0].beats++;
            if (l != (awq[0].beats == 16)) err("WLAST sai vị trí");
            if (awq[0].beats == 16) begin void'(awq.pop_front()); b_due.push_back(cyc + 15 + $urandom_range(0, 25)); end
        end
        // B (bready = 1 trong DUT)
        if (bvalid && bready) bvalid <= 0;
        else if (!bvalid && b_due.size() > 0 && cyc >= b_due[0]) begin
            void'(b_due.pop_front()); bvalid <= 1; bresp <= 0; bid <= 0;
        end
    end

    // --- Đọc ---
    typedef struct { logic [31:0] addr; int len; int due; } ar_t;
    ar_t arq[$]; int rbeat = 0;
    logic ar_stall = 0; logic [31:0] ar_h;
    assign arready = rnd_ar;
    always @(posedge clk) if (!rst_n) begin rvalid <= 0; end else begin
        if (ar_stall && (!arvalid || araddr != ar_h)) err("AR thay đổi khi bị chặn");
        ar_stall <= arvalid && !arready; ar_h <= araddr;
        if (arvalid && arready) begin
            ar_t t;
            if (arlen > 15) err("ARLEN > 15");
            if (({20'b0, araddr[11:0]} & ~32'h7) + (arlen + 1) * 8 > 4096) err("AR vượt 4KB");
            t.addr = araddr; t.len = arlen + 1; t.due = cyc + 20 + $urandom_range(0, 20); arq.push_back(t);
        end
        if (rvalid && rready) begin
            rbeat++;
            if (rlast) begin void'(arq.pop_front()); rbeat = 0; end
        end
        if (arq.size() > 0 && cyc >= arq[0].due && rnd_r) begin
            rdata <= rdw(29'((arq[0].addr >> 3) + rbeat)); rvalid <= 1; rlast <= (rbeat == arq[0].len - 1);
            rresp <= 0; rid <= 0;
        end else rvalid <= 0;
    end

    // ---------------- Nguồn ISP ----------------
    task automatic isp_frame_send(input int f);
        isp_frame = f;
        for (int y = 0; y < 480; y++) begin
            for (int x = 0; x < 640; x++) begin
                @(negedge clk_isp);
                isp_valid = 1; isp_data = pix(f, x, y); isp_sof = (x == 0 && y == 0); isp_eol = (x == 639);
            end
            @(negedge clk_isp); isp_valid = 0; isp_sof = 0; isp_eol = 0;
            repeat (159) @(negedge clk_isp);
        end
        repeat (45 * 800) @(negedge clk_isp);
    endtask
    bit isp_done = 0;

    // ---------------- APB ----------------
    task automatic apb_wr(input logic [11:0] a, input logic [31:0] d);
        @(negedge clk); paddr = a; pwdata = d; pwrite = 1; psel = 1; penable = 0;
        @(negedge clk); penable = 1;
        @(posedge clk); #0.1;
        if (pslverr) err($sformatf("PSLVERR khi ghi %h", a));
        @(negedge clk); psel = 0; penable = 0; pwrite = 0;
    endtask
    task automatic apb_rd(input logic [11:0] a, output logic [31:0] d);
        @(negedge clk); paddr = a; pwrite = 0; psel = 1; penable = 0;
        @(negedge clk); penable = 1;
        @(posedge clk); d = prdata;
        @(negedge clk); psel = 0; penable = 0;
    endtask

    // ---------------- Bảng ROI đã gửi ----------------
    typedef struct { int x, y, w, h, dl, fid; logic [47:0] deadline; int kind; bit got_pkt, pkt_ok, done; } roi_t;
    // kind: 0 = bình thường, 1 = sai (W/dec), 2 = frame cũ (STALE chắc chắn), 3 = deadline gấp
    roi_t rois [int];
    int n_sent = 0, n_cpl = 0, st_cnt [8], stale_normal = 0;

    // ---------------- Scoreboard U01 (thụ động: chỉ nghe AXIS, không lái tín hiệu nào) ----------------
    logic [7:0] pkt[$]; logic [63:0] hdr; bit in_pkt = 0; int beat_i = 0;
    logic t_stall = 0; logic [63:0] t_h; logic [7:0] k_h; logic l_h, u_h;
    always @(posedge clk) if (rst_n) begin
        if (t_stall && (!tvalid || tdata != t_h || tkeep != k_h || tlast != l_h || tuser != u_h)) err("AXIS thay đổi khi bị chặn");
        t_stall <= tvalid && !tready; t_h <= tdata; k_h <= tkeep; l_h <= tlast; u_h <= tuser;
        if (tvalid && tready) begin
            if (!in_pkt) begin
                if (!tuser[0]) begin err("beat đầu gói không có TUSER"); if ($test$plusargs("DBG")) $display("   tdata=%h tkeep=%h tlast=%b | crop: st=%0d oc=%0d rem=%0d pad_rem=%0d ri_valid=%b first=%b last=%b lead=%0d n=%0d trail=%0d | sched st=%0d", tdata, tkeep, tlast, dut.u_crop.st, dut.u_crop.oc, dut.u_crop.rem, dut.u_crop.pad_rem, dut.u_crop.ri_valid, dut.u_crop.ri.first, dut.u_crop.ri.last, dut.u_crop.ri.lead_pad, dut.u_crop.ri.n_valid, dut.u_crop.ri.trail_pad, dut.u_sched.st); end
                if (tkeep != 8'hFF || tlast) err("header sai TKEEP/TLAST");
                hdr = tdata; in_pkt = 1; pkt.delete();
            end else begin
                if (tuser[0]) err("TUSER ở beat payload");
                if (!tlast && tkeep != 8'hFF) err("TKEEP != FF ở beat giữa");
                for (int k = 0; k < 8; k++) if (tkeep[k]) pkt.push_back(tdata[k*8 +: 8]);
                if (tlast) begin if ($test$plusargs("DBG")) $display("[%0t] pkt roi=%0d bytes=%0d", $time, hdr[39:32], pkt.size()); check_pkt(); in_pkt = 0; end
            end
        end
    end

    task automatic check_pkt();
        int rid_, fr, idx, bad, dec, hp;
        roi_t r;
        rid_ = int'(hdr[39:32]);
        if (hdr[63:56] != 8'hA5) err("magic sai");
        if (!rois.exists(rid_)) begin err($sformatf("gói của roi_id %0d không tồn tại", rid_)); return; end
        r = rois[rid_];
        if (r.got_pkt) err($sformatf("roi_id %0d nhận 2 gói", rid_));
        r.got_pkt = 1;
        if (hdr[15:0] != 16'(r.w) || hdr[31:16] != 16'(r.h) || hdr[47:40] != 8'(r.fid) || hdr[50:49] != 2'(r.dl))
            err($sformatf("header roi %0d sai: %h", rid_, hdr));
        if (pkt.size() != r.w * r.h * 3) err($sformatf("roi %0d: %0d byte, mong đợi %0d", rid_, pkt.size(), r.w*r.h*3));
        if (!fid2frame.exists(r.fid)) begin err("fid không rõ khung"); return; end
        fr = fid2frame[r.fid]; dec = 1 << r.dl; bad = 0; idx = 0; hp = 0;
        for (int yy = 0; yy < r.h; yy++)
            for (int xx = 0; xx < r.w; xx++) begin
                int sx, sy; logic [23:0] e;
                sx = r.x + xx*dec; sy = r.y + yy*dec;
                if (sx < 0 || sx >= 640 || sy < 0 || sy >= 480) begin e = {3{PADV}}; hp = 1; end
                else e = pix(fr, sx, sy);
                for (int c = 0; c < 3; c++) begin
                    if (idx < pkt.size() && pkt[idx] != e[c*8 +: 8]) begin
                        if (bad == 0) err($sformatf("roi %0d (x=%0d y=%0d W=%0d H=%0d dec=%0d, khung %0d) sai byte tại pixel (%0d,%0d): %h != %h",
                                                     rid_, r.x, r.y, r.w, r.h, dec, fr, xx, yy, pkt[idx], e[c*8 +: 8]));
                        bad++;
                    end
                    idx++;
                end
            end
        if (hdr[48] != 1'(hp)) err($sformatf("roi %0d has_pad=%0b mong đợi %0b", rid_, hdr[48], hp));
        r.pkt_ok = (bad == 0);
        rois[rid_] = r;
    endtask

    // ---------------- CPU ----------------
    task automatic push_roi(input int fid, input int kind, input int rid_);
        roi_t r; logic [31:0] lo, hi; int w, h;
        r.kind = kind; r.got_pkt = 0; r.pkt_ok = 0; r.done = 0;
        case ($urandom_range(0, 3))
            0: begin r.x = $urandom_range(0, 600); r.y = $urandom_range(0, 440); end
            1: begin r.x = -$urandom_range(0, 300); r.y = -$urandom_range(0, 300); end
            2: begin r.x = 640 - $urandom_range(0, 400); r.y = 480 - $urandom_range(0, 400); end
            default: begin r.x = $urandom_range(0, 1000) - 300; r.y = $urandom_range(0, 800) - 200; end
        endcase
        r.w = $urandom_range(16, 128); r.h = $urandom_range(16, 128); r.dl = $urandom_range(0, 2);
        r.fid = fid; r.deadline = 0;
        if (kind == 1) begin if ($urandom_range(0,1)) r.w = 15; else r.dl = 3; end
        if (kind == 2) r.fid = (fid + 256 - 5) % 256;
        if (kind == 3) begin
            apb_rd(12'h038, lo); apb_rd(12'h03C, hi);
            r.deadline = {hi[15:0], lo} + 48'($urandom_range(200, 20000));
        end
        rois[rid_] = r;
        if ($test$plusargs("DBG")) $display("[%0t] push roi=%0d kind=%0d x=%0d y=%0d W=%0d H=%0d dec=%0d fid=%0d", $time, rid_, kind, r.x, r.y, r.w, r.h, 1<<r.dl, r.fid);
        apb_wr(12'h020, {4'b0, 12'(r.y), 4'b0, 12'(r.x)});
        apb_wr(12'h024, {8'b0, 8'(r.h), 8'b0, 8'(r.w)});
        apb_wr(12'h028, {14'b0, 2'(r.dl), 8'(r.fid), 8'(rid_)});
        apb_wr(12'h02C, r.deadline[31:0]);
        apb_wr(12'h030, {16'b0, r.deadline[47:32]});
        apb_wr(12'h034, 32'h1);
        n_sent++;
    endtask

    task automatic drain_cpl();
        logic [31:0] id, a, b, c, d, e, f, bd, bs; int rid_, st; roi_t r;
        apb_rd(12'h040, id);
        while (id[31]) begin
            rid_ = int'(id[7:0]); st = int'(id[18:16]);
            apb_rd(12'h044, a); apb_rd(12'h048, b); apb_rd(12'h04C, c); apb_rd(12'h050, d);
            apb_rd(12'h054, e); apb_rd(12'h058, f); apb_rd(12'h05C, bd); apb_rd(12'h060, bs);
            apb_wr(12'h064, 32'h1);
            n_cpl++; st_cnt[st]++;
            if (!rois.exists(rid_)) err($sformatf("completion của roi %0d lạ", rid_));
            else begin
                r = rois[rid_];
                if (r.done) err($sformatf("roi %0d có 2 completion", rid_));
                r.done = 1;
                case (st)
                    0: begin   // OK
                        if (r.kind == 1 || r.kind == 2) err($sformatf("roi %0d (kind %0d) lẽ ra không OK", rid_, r.kind));
                        if (!r.got_pkt) err($sformatf("roi %0d OK nhưng U01 chưa nhận gói", rid_));
                        if (bs != 32'(r.w*r.h*3)) err($sformatf("roi %0d bytes_stream=%0d", rid_, bs));
                        if (r.deadline != 0 && {d[15:0], c} > r.deadline) err("OK nhưng bắt đầu sau deadline");
                    end
                    1: begin   // DROPPED_LATE
                        if (r.deadline == 0) err($sformatf("roi %0d LATE nhưng không có deadline", rid_));
                        if ({d[15:0], c} <= r.deadline) err("LATE nhưng t_start chưa quá hạn");
                    end
                    2: if (r.kind != 2) begin
                           // STALE của ROI bình thường: chấp nhận nhưng đếm riêng
                           stale_normal++;
                       end
                    3: if (r.kind != 1) err($sformatf("roi %0d INVALID nhưng descriptor đúng", rid_));
                    default: err($sformatf("roi %0d status %0d", rid_, st));
                endcase
                if (r.kind == 1 && st != 3) err($sformatf("roi %0d sai mà status=%0d", rid_, st));
                if (r.kind == 2 && st != 2) err($sformatf("roi %0d frame cũ mà status=%0d", rid_, st));
                if (st != 0 && r.got_pkt) err($sformatf("roi %0d status %0d nhưng U01 lại nhận gói", rid_, st));
                if (!({b[15:0], a} <= {d[15:0], c} && {d[15:0], c} <= {f[15:0], e})) err("thứ tự timestamp sai");
                rois[rid_] = r;
            end
            apb_rd(12'h040, id);
        end
    endtask
    initial begin
        logic [31:0] st, v;
        int rid_ = 0, fid, kind, nb;
        if ($value$plusargs("FRAMES=%d", N_FRAMES)) ;
        if ($value$plusargs("U01_IBUF_N=%d", IBUF_N)) ;   // cùng plusarg với u01_model
        if ($value$plusargs("NEG_IBUF_N=%d", IBUF_N)) ;   // thử âm: cố tình ghi IBUF_N của U02 khác U01 → u01_model phải báo lỗi credit
        if ($value$plusargs("PADVAL=%h", PADV)) ;
        AW_WAIT_W = $test$plusargs("AW_WAIT_W");
        paddr = 0;
        repeat (10) @(negedge clk); rst_n = 1; repeat (10) @(negedge clk);
        apb_rd(12'h010, v); if (v != 32'h0110_8010) err($sformatf("CAP = %h", v));
        apb_wr(12'h00C, {24'b0, PADV});
        apb_wr(12'h014, 32'(IBUF_N));
        apb_wr(12'h004, 32'h7F);
        apb_wr(12'h000, 32'h3);
        fork
            begin for (int f = 0; f < N_FRAMES; f++) isp_frame_send(f); isp_done = 1; end
            begin
                // chờ khung đầu tiên commit
                st = 0;
                while (!st[8] && !isp_done) apb_rd(12'h070, st);
                while (!isp_done) begin
                    apb_rd(12'h070, st); fid = int'(st[7:0]);
                    nb = $urandom_range(1, 6);
                    for (int k = 0; k < nb; k++) begin
                        int p; p = $urandom_range(0, 99);
                        kind = (p < 70) ? 0 : (p < 80) ? 1 : (p < 88) ? 2 : 3;
                        apb_rd(12'h004, v);
                        if (!v[2]) begin push_roi(fid, kind, rid_ % 256); rid_++; end
                    end
                    repeat ($urandom_range(500, 20000)) @(negedge clk);
                    drain_cpl();
                end
                // xả hết
                // U01 mô hình tính chậm (≈0,76 ms / ROI 128x128) → chờ tới khi đủ completion (tối đa ~40 ms)
                for (int t = 0; t < 30 && n_cpl < n_sent; t++) begin
                    repeat (200000) @(negedge clk);
                    drain_cpl();
                end
            end
        join
        begin
            int lost; lost = 0;
            foreach (rois[i]) if (!rois[i].done) lost++;
            if (lost) err($sformatf("%0d ROI không có completion", lost));
        end
        apb_rd(12'h074, v);
        $display("------------------------------------------------------------");
        $display("tb_u02_u01m: khung ISP=%0d commit=%0d abort=%0d | ROI gửi=%0d completion=%0d",
                 N_FRAMES, v[15:0], v[31:16], n_sent, n_cpl);
        $display("  OK=%0d LATE=%0d STALE=%0d (trong đó ROI thường bị STALE=%0d) INVALID=%0d BUS_ERR=%0d",
                 st_cnt[0], st_cnt[1], st_cnt[2], stale_normal, st_cnt[3], st_cnt[4]);
        $display("  U01_MODEL: nhận %0d ROI, tính xong %0d, lỗi giao thức %0d", u01_recv, u01_done, u01_err);
        errors += u01_err;
        $display("  LỖI = %0d %s", errors, errors == 0 ? "-> PASS" : "-> FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

    // Watchdog: khung không commit được -> treo
    int last_commit_cyc = 0;
    always @(posedge clk) if (dut.u_fbm.commit_evt) last_commit_cyc <= cyc;
    always @(posedge clk) if (rst_n && cyc - last_commit_cyc > 6_000_000) begin
        $display("[%0t] TREO: không có khung nào commit trong 40 ms. awvalid=%0b awready=%0b wvalid=%0b ost=%0d wq=%0d",
                 $time, awvalid, awready, wvalid, dut.u_wr.ost, dut.u_wr.wq);
        $display("  LỖI = %0d -> FAIL (deadlock)", errors + 1);
        $finish;
    end
endmodule
