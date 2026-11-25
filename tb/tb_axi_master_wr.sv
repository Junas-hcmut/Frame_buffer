`timescale 1ns / 1ps

// =====================================================================
// tb_axi_master_wr: kiểm axi_master_wr với slave AXI ngẫu nhiên.
//
// Nguồn: mô hình isp_ingress đơn giản - mỗi khung gồm N burst (16 từ),
// kết thúc bằng WR_DATA_LAST (khung đủ) hoặc WR_ABORT (khung hủy, có thể
// sau 0 burst). Dữ liệu được đẩy vào FIFO dữ liệu TRƯỚC lệnh của burst,
// giống isp_ingress.
// Slave: AWREADY/WREADY ngẫu nhiên, BRESP trễ ngẫu nhiên (đúng thứ tự),
// tiêm SLVERR vào một số burst. Từng giai đoạn 3000 chu kỳ, slave chuyển
// sang chế độ chỉ bật AWREADY khi đã thấy WVALID (AXI4 cho phép slave chờ
// WVALID trước AWREADY; master chờ AWREADY rồi mới phát W sẽ deadlock).
//
// Kiểm tra:
//  - Bộ nhớ: mọi byte ghi đúng địa chỉ và giá trị (W được ghép với AW theo
//    thứ tự, W có thể đến trước AW như AXI4 cho phép).
//  - Giao thức: AW/W giữ ổn định khi valid & !ready; 16 beat / burst,
//    WLAST đúng beat 16; số burst chưa có BRESP <= MAX_OUTSTANDING;
//    BRESP chỉ được phát sau khi đã có cả AW và beat W cuối của burst.
//  - Sự kiện: mỗi khung sinh đúng 1 commit (khung đủ, mọi BRESP OKAY)
//    hoặc 1 abort (khung hủy hoặc có BRESP lỗi), đúng buf_idx, đúng thứ tự,
//    và chỉ sau khi mọi BRESP của khung đã về.
// =====================================================================
module tb_axi_master_wr;
    import u02_pkg::*;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    // ---------------- Nguồn: FIFO lệnh / dữ liệu ----------------
    logic        wd_valid, wd_ready, wc_valid, wc_ready;
    logic [63:0] wd_data;
    u02_wr_cmd_t wc_cmd;
    logic        df_wr = 0, cf_wr = 0, df_full, cf_full, df_empty, cf_empty;
    logic [63:0] df_din = '0;
    u02_wr_cmd_t cf_din = '0;

    sync_fifo #(.DW(64), .DEPTH(64)) u_df (
        .clk(clk), .rst_n(rst_n), .wr_en(df_wr), .wr_data(df_din), .full(df_full),
        .rd_en(wd_ready), .rd_data(wd_data), .empty(df_empty), .count());
    sync_fifo #(.DW($bits(u02_wr_cmd_t)), .DEPTH(8)) u_cf (
        .clk(clk), .rst_n(rst_n), .wr_en(cf_wr), .wr_data(cf_din), .full(cf_full),
        .rd_en(wc_ready), .rd_data(wc_cmd), .empty(cf_empty), .count());
    assign wd_valid = !df_empty;
    assign wc_valid = !cf_empty;

    // ---------------- AXI ----------------
    logic [0:0]  awid, bid = '0;
    logic [31:0] awaddr;
    logic [7:0]  awlen;
    logic [2:0]  awsize, awprot;
    logic [1:0]  awburst, bresp = 2'b00;
    logic [3:0]  awcache;
    logic        awvalid, awready = 0, wvalid, wready = 0, wlast, bvalid = 0, bready;
    logic [63:0] wdata;
    logic [7:0]  wstrb;
    logic        commit_evt, abort_evt, bus_err_evt;
    fb_idx_t     commit_idx, abort_idx;

    axi_master_wr dut (.*);

    // ---------------- Kịch bản ----------------
    localparam int NFRAMES = 300;
    int  fr_nb   [NFRAMES];       // số burst dữ liệu
    bit  fr_abt  [NFRAMES];       // khung kết thúc bằng WR_ABORT
    int  fr_idx  [NFRAMES];
    bit  fr_berr [NFRAMES];       // có ít nhất 1 BRESP lỗi được tiêm
    int  fr_bdone[NFRAMES];       // số BRESP đã về
    int  total_bursts;

    // Bộ nhớ mong đợi và thực tế (theo từ 64 bit, địa chỉ / 8)
    localparam int MW = 3 * 131072;          // 3 khe x 1 MB, theo từ 64 bit
    logic [63:0] mem_exp [MW];
    logic [63:0] mem_act [MW];
    bit          exp_v   [MW];
    bit          act_v   [MW];
    function automatic int wi(input logic [31:0] a);
        return int'((a - FB0_BASE_ADDR) >> 3);
    endfunction

    // Burst trong thứ tự phát: frame của mỗi burst
    int  burst_frame [$];
    bit  burst_err   [$];

    task automatic step(); @(posedge clk); #1; endtask

    initial begin
        int b;
        for (int f = 0; f < NFRAMES; f++) begin
            int r;
            r          = $urandom % 100;
            fr_abt[f]  = (r < 20);
            fr_nb[f]   = fr_abt[f] ? ($urandom % 4) : 1 + ($urandom % 12);
            fr_idx[f]  = $urandom % NUM_FB;
            fr_berr[f] = 1'b0;
            fr_bdone[f]= 0;
        end
        // vài khung cuối dài để có nhiều outstanding
        fr_abt[5] = 0; fr_nb[5] = 40;
    end

    // Nguồn: đẩy dữ liệu 16 từ rồi lệnh
    int src_done = 0;
    initial begin
        logic [31:0] base;
        @(posedge rst_n); step();
        for (int f = 0; f < NFRAMES; f++) begin
            base = fb_base(fb_idx_t'(fr_idx[f]));
            for (int k = 0; k < fr_nb[f]; k++) begin
                for (int w = 0; w < 16; w++) begin
                    logic [63:0] d;
                    d = {$urandom, $urandom};
                    while (df_full) step();
                    df_wr = 1; df_din = d;
                    mem_exp[wi(base + k*128 + w*8)] = d;
                    exp_v[wi(base + k*128 + w*8)]   = 1'b1;
                    step(); df_wr = 0;
                    if (($urandom % 4) == 0) step();
                end
                while (cf_full) step();
                cf_wr = 1;
                cf_din.addr    = base + k*128;
                cf_din.buf_idx = fb_idx_t'(fr_idx[f]);
                cf_din.kind    = (!fr_abt[f] && k == fr_nb[f]-1) ? WR_DATA_LAST : WR_DATA;
                burst_frame.push_back(f);
                step(); cf_wr = 0;
            end
            if (fr_abt[f]) begin
                while (cf_full) step();
                cf_wr = 1;
                cf_din.addr = base; cf_din.buf_idx = fb_idx_t'(fr_idx[f]); cf_din.kind = WR_ABORT;
                step(); cf_wr = 0;
            end
            if (($urandom % 3) == 0) repeat ($urandom % 20) step();
        end
        src_done = 1;
    end

    // ---------------- Slave AXI ----------------
    int  aw_q_addr [$];
    int  aw_cnt = 0, w_burst_cnt = 0, b_cnt = 0, w_beat = 0;
    int  b_pending_t [$];        // thời điểm (chu kỳ) BRESP sẵn sàng
    int  cyc = 0;
    int  errors = 0;
    int  max_ost = 0;
    always @(posedge clk) cyc++;

    // ready ngẫu nhiên. aw_wait_w: slave chỉ nhận AW khi dữ liệu W của chính
    // burst đó đã bắt đầu đến (WVALID của beat đầu, hoặc đã nhận beat đầu).
    bit aw_wait_w = 0;
    int n_aw_wait_w = 0;
    int w_beats_rx = 0;
    always @(posedge clk) if (wvalid && wready) w_beats_rx++;
    always @(posedge clk) begin
        if (cyc % 3000 == 0) aw_wait_w <= ~aw_wait_w;
        awready <= (($urandom % 100) < 60) &&
                   (!aw_wait_w || (w_beats_rx > aw_cnt*16) || (wvalid && w_beats_rx == aw_cnt*16));
        wready  <= ($urandom % 100) < 70;
        if (aw_wait_w && awvalid && awready) n_aw_wait_w++;
    end

    // Phát hiện deadlock: slave chờ WVALID, master giữ AWVALID mà không phát W
    int stuck = 0;
    always @(posedge clk) if (rst_n) begin
        if (aw_wait_w && awvalid && !wvalid) stuck++;
        else stuck = 0;
        if (stuck == 200) begin
            errors++;
            $display("LOI: deadlock t=%0t - AWVALID cho AWREADY nhung master khong phat WVALID (vi pham AXI4 A3.3.1)", $time);
        end
    end

    // W đến trước AW: giữ dữ liệu theo burst, ghép với AW khi có đủ
    logic [63:0] w_buf [$];       // beat W chưa ghép AW
    int          w_full_bursts = 0, applied = 0;

    // Ổn định khi valid & !ready
    logic        aw_hold = 0, w_hold = 0;
    logic [31:0] aw_hold_addr;
    logic [63:0] w_hold_data;
    logic        w_hold_last;

    always @(posedge clk) if (rst_n) begin
        // ---- AW ----
        if (aw_hold && !(awvalid && awaddr == aw_hold_addr)) begin
            errors++; $display("LOI: AW thay doi/ha khi chua handshake t=%0t", $time);
        end
        aw_hold      <= awvalid && !awready;
        aw_hold_addr <= awaddr;
        if (awvalid && awready) begin
            if (awlen != 8'd15 || awsize != 3'd3 || awburst != 2'b01) begin
                errors++; $display("LOI: AW len/size/burst sai");
            end
            if (awaddr[6:0] != 0) begin errors++; $display("LOI: awaddr khong can le 128 B"); end
            aw_q_addr.push_back(awaddr);
            aw_cnt++;
        end
        // ---- W ----
        if (w_hold && !(wvalid && wdata == w_hold_data && wlast == w_hold_last)) begin
            errors++; $display("LOI: W thay doi/ha khi chua handshake t=%0t", $time);
        end
        w_hold      <= wvalid && !wready;
        w_hold_data <= wdata;
        w_hold_last <= wlast;
        if (wvalid && wready) begin
            if (wstrb != 8'hFF) begin errors++; $display("LOI: wstrb"); end
            w_buf.push_back(wdata);
            if (wlast != (w_beat == 15)) begin errors++; $display("LOI: WLAST sai o beat %0d", w_beat); end
            if (w_beat == 15) begin w_beat = 0; w_full_bursts++; end
            else w_beat++;
        end
        // ghép burst đã đủ cả AW và W -> ghi bộ nhớ, hẹn BRESP
        while (applied < aw_cnt && applied < w_full_bursts) begin
            for (int k = 0; k < 16; k++) begin
                int a;
                a = aw_q_addr[applied] + k*8;
                mem_act[wi(a)] = w_buf.pop_front();
                act_v[wi(a)]   = 1'b1;
            end
            applied++;
            w_burst_cnt++;
            b_pending_t.push_back(cyc + 1 + ($urandom % 25));
        end
        // ---- outstanding ----
        if (aw_cnt - b_cnt > max_ost) max_ost = aw_cnt - b_cnt;
        if (aw_cnt - b_cnt > MAX_OUTSTANDING) begin
            errors++; $display("LOI: outstanding %0d > %0d", aw_cnt - b_cnt, MAX_OUTSTANDING);
        end
    end

    // BRESP đúng thứ tự, tối đa 1 BRESP mỗi 2 chu kỳ
    always @(posedge clk) if (rst_n) begin
        if (bvalid && bready) begin
            fr_bdone[burst_frame[b_cnt]]++;
            b_cnt++;
            bvalid <= 1'b0;
        end else if (!bvalid && b_pending_t.size() > 0 && b_pending_t[0] <= cyc) begin
            void'(b_pending_t.pop_front());
            bvalid <= 1'b1;
            if (($urandom % 100) < 4) begin            // tiêm lỗi ~4% burst
                bresp <= 2'b10;
                fr_berr[burst_frame[b_cnt]] = 1'b1;
            end else begin
                bresp <= 2'b00;
            end
        end
    end

    // ---------------- Kiểm sự kiện khung ----------------
    int ev_frame = 0;            // khung kế tiếp phải kết thúc
    int n_commit = 0, n_abort = 0, n_buserr = 0;
    always @(posedge clk) if (rst_n) begin
        if (bus_err_evt) n_buserr++;
        if (commit_evt || abort_evt) begin
            if (commit_evt && abort_evt) begin errors++; $display("LOI: commit va abort cung chu ky"); end
            // bỏ qua khung 0 burst mà không abort (không tồn tại theo kịch bản)
            if (ev_frame >= NFRAMES) begin errors++; $display("LOI: su kien thua"); end
            else begin
                bit exp_commit;
                exp_commit = !fr_abt[ev_frame] && !fr_berr[ev_frame];
                if (commit_evt != exp_commit) begin
                    errors++;
                    $display("LOI khung %0d: %s, ky vong %s (abt=%0d berr=%0d)", ev_frame,
                             commit_evt ? "commit" : "abort", exp_commit ? "commit" : "abort",
                             fr_abt[ev_frame], fr_berr[ev_frame]);
                end
                if ((commit_evt ? commit_idx : abort_idx) != fb_idx_t'(fr_idx[ev_frame])) begin
                    errors++; $display("LOI khung %0d: buf_idx sai", ev_frame);
                end
                if (fr_bdone[ev_frame] != fr_nb[ev_frame]) begin
                    errors++; $display("LOI khung %0d: su kien truoc khi du BRESP (%0d/%0d)",
                                       ev_frame, fr_bdone[ev_frame], fr_nb[ev_frame]);
                end
                if (commit_evt) n_commit++; else n_abort++;
            end
            ev_frame++;
        end
    end

    initial begin
        int guard;
        repeat (3) step();
        rst_n = 1;
        guard = 0;
        while (!(src_done && ev_frame == NFRAMES) && guard < 2_000_000) begin step(); guard++; end
        repeat (50) step();

        // so bộ nhớ
        begin
            int nbad, nw;
            nbad = 0; nw = 0;
            for (int a = 0; a < MW; a++) if (exp_v[a]) begin
                nw++;
                if (!act_v[a] || mem_act[a] !== mem_exp[a]) begin
                    nbad++;
                    if (nbad <= 3) $display("LOI bo nho @%h", FB0_BASE_ADDR + a*8);
                end
            end
            // Ghi chú: các khung cùng buf_idx ghi đè nhau -> so với lần ghi cuối,
            // mem_exp cũng giữ lần ghi cuối nên phép so vẫn đúng.
            if (nbad) errors++;
            $display("Bo nho: %0d tu, sai %0d", nw, nbad);
        end
        $display("Khung: %0d, commit %0d, abort %0d, BRESP loi %0d, burst %0d, outstanding max %0d, AW nhan o che do cho WVALID %0d",
                 ev_frame, n_commit, n_abort, n_buserr, b_cnt, max_ost, n_aw_wait_w);
        if (n_aw_wait_w == 0) begin errors++; $display("LOI: khong co AW nao o che do cho WVALID"); end
        if (ev_frame != NFRAMES) begin errors++; $display("LOI: chi co %0d/%0d su kien", ev_frame, NFRAMES); end
        if (errors == 0) $display("==== tb_axi_master_wr: PASS ====");
        else             $display("==== tb_axi_master_wr: FAIL (%0d loi) ====", errors);
        $finish;
    end
endmodule
