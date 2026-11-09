`timescale 1ns / 1ps

// =====================================================================
// tb_read_path: kiểm nhánh đọc row_cmd_gen -> page_splitter ->
// axi_master_rd -> axis_crop (kèm FIFO row_info như u02_top) với:
//   - DDR mô hình: 1 khung 640x480 RGB888 ngẫu nhiên tại FB1_BASE,
//     ARREADY / khoảng trống R ngẫu nhiên, tiêm RRESP lỗi cho vài ROI.
//   - U01 mô hình: TREADY ngẫu nhiên.
//   - Mô hình vàng tính trực tiếp từ (x, y, W, H, dec): pixel ngõ ra
//     (r, c) = ảnh[y + r*dec][x + c*dec] nếu trong khung, ngược lại pad.
// Kiểm: header (magic, W, H, ids, dec, has_pad), toàn bộ payload theo byte,
// tkeep / tlast / tuser, roi_done & roi_err, số beat R = dự đoán từ địa chỉ.
// =====================================================================
module tb_read_path;
    import u02_pkg::*;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ---------------- DUT ----------------
    logic          start = 0;
    u02_roi_desc_t desc = '0;
    logic [31:0]   base = FB1_BASE_ADDR;
    logic [7:0]    pad_val = 8'h00;
    logic          rg_busy;

    logic          rc_valid, rc_ready, rc_eol, rc_eof;
    logic [31:0]   rc_addr;
    logic [11:0]   rc_len;
    logic [7:0]    rc_roi_id, rc_frame_id;
    logic          rg_ri_valid, rg_ri_ready, rif_full, rif_empty, ri_valid, ri_ready;
    u02_row_info_t rg_ri, ri_head;

    logic          ps_valid, ps_ready, ps_eol, ps_eof, ps_first, ps_last, ps_split, ps_err_zero;
    logic [31:0]   ps_addr;
    logic [11:0]   ps_len;
    logic [7:0]    ps_arlen, ps_roi_id, ps_frame_id;

    logic [0:0]    arid, rid = '0;
    logic [31:0]   araddr;
    logic [7:0]    arlen;
    logic [2:0]    arsize, arprot;
    logic [1:0]    arburst, rresp = 2'b00;
    logic [3:0]    arcache;
    logic          arvalid, arready = 0, rvalid = 0, rready, rlast = 0;
    logic [63:0]   rdata = '0;

    logic          d_valid, d_ready, d_row_last, d_err, rd_bus_err;
    logic [63:0]   d_data;
    logic [2:0]    d_lo;
    logic [3:0]    d_cnt;
    logic [19:0]   beat_cnt;
    logic          cnt_clear = 0;

    logic          tvalid, tready = 0, tlast;
    logic [63:0]   tdata;
    logic [7:0]    tkeep;
    logic [0:0]    tuser;
    logic          roi_done, roi_err;

    row_cmd_gen u_rowgen (
        .clk(clk), .rst_n(rst_n), .start(start), .desc(desc), .base(base), .busy(rg_busy),
        .rc_valid(rc_valid), .rc_ready(rc_ready), .rc_addr(rc_addr), .rc_len(rc_len),
        .rc_roi_id(rc_roi_id), .rc_frame_id(rc_frame_id), .rc_eol(rc_eol), .rc_eof(rc_eof),
        .ri_valid(rg_ri_valid), .ri_ready(rg_ri_ready), .ri(rg_ri));

    sync_fifo #(.DW($bits(u02_row_info_t)), .DEPTH(8)) u_rif (
        .clk(clk), .rst_n(rst_n), .wr_en(rg_ri_valid), .wr_data(rg_ri), .full(rif_full),
        .rd_en(ri_ready), .rd_data(ri_head), .empty(rif_empty), .count());
    assign rg_ri_ready = !rif_full;
    assign ri_valid    = !rif_empty;

    page_splitter u_ps (
        .clk(clk), .rst_n(rst_n),
        .s_req_valid(rc_valid), .s_req_ready(rc_ready), .s_req_addr(rc_addr), .s_req_len(rc_len),
        .s_req_roi_id(rc_roi_id), .s_req_frame_id(rc_frame_id), .s_req_eol(rc_eol), .s_req_eof(rc_eof),
        .m_req_valid(ps_valid), .m_req_ready(ps_ready), .m_req_addr(ps_addr), .m_req_len(ps_len),
        .m_req_arlen(ps_arlen), .m_req_roi_id(ps_roi_id), .m_req_frame_id(ps_frame_id),
        .m_req_eol(ps_eol), .m_req_eof(ps_eof), .m_req_first(ps_first), .m_req_last(ps_last),
        .m_req_split(ps_split), .err_zero_len(ps_err_zero));

    axi_master_rd u_rd (
        .clk(clk), .rst_n(rst_n),
        .c_valid(ps_valid), .c_ready(ps_ready), .c_addr(ps_addr), .c_len(ps_len),
        .c_arlen(ps_arlen), .c_row_last(ps_last),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
        .arcache(arcache), .arprot(arprot), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready),
        .d_valid(d_valid), .d_ready(d_ready), .d_data(d_data), .d_lo(d_lo), .d_cnt(d_cnt),
        .d_row_last(d_row_last), .d_err(d_err),
        .cnt_clear(cnt_clear), .beat_cnt(beat_cnt), .bus_err_evt(rd_bus_err));

    axis_crop u_crop (
        .clk(clk), .rst_n(rst_n), .cfg_pad_val(pad_val),
        .ri_valid(ri_valid), .ri_ready(ri_ready), .ri(ri_head),
        .d_valid(d_valid), .d_ready(d_ready), .d_data(d_data), .d_lo(d_lo), .d_cnt(d_cnt),
        .d_row_last(d_row_last), .d_err(d_err),
        .m_axis_tvalid(tvalid), .m_axis_tready(tready), .m_axis_tdata(tdata),
        .m_axis_tkeep(tkeep), .m_axis_tlast(tlast), .m_axis_tuser(tuser),
        .roi_done(roi_done), .roi_err(roi_err));

    int  errors = 0;

    // ---------------- DDR ----------------
    localparam int FSZ = FRAME_SIZE_B;
    logic [7:0] img [0:FSZ-1];
    initial for (int i = 0; i < FSZ; i++) img[i] = 8'($urandom);

    int  ar_q_addr [$];
    int  ar_q_len  [$];
    int  r_beat = 0;
    bit  inject_err = 0;          // ROI hiện tại: tiêm RRESP lỗi vào 1 beat
    bit  err_done   = 0;
    int  n_ar = 0, n_rbeats = 0;

    always @(posedge clk) begin
        arready <= ($urandom % 100) < 70;
        if (rst_n && arvalid && arready) begin
            if (araddr < base || (araddr & ~7) + (arlen+1)*8 > base + FSZ) begin
                errors++; $display("LOI: AR ngoai khung %h", araddr);
            end
            if (((araddr & ~7) & 32'hFFF) + (arlen+1)*8 > 4096) begin errors++; $display("LOI: AR vuot 4KB %h", araddr); end
            ar_q_addr.push_back(araddr);
            ar_q_len.push_back(arlen + 1);
            n_ar++;
        end
    end

    always @(posedge clk) begin
        if (rvalid && rready) begin
            n_rbeats++;
            rvalid <= 0;
        end
        if (rst_n && (!rvalid || rready) && ar_q_addr.size() > 0 && ($urandom % 100) < 75) begin
            int a;
            a = (ar_q_addr[0] & ~7) - base + r_beat*8;
            for (int b = 0; b < 8; b++) rdata[b*8 +: 8] <= img[a + b];
            rvalid <= 1;
            rlast  <= (r_beat == ar_q_len[0] - 1);
            if (inject_err && !err_done && ($urandom % 4) == 0) begin rresp <= 2'b10; err_done = 1; end
            else rresp <= 2'b00;
            if (r_beat == ar_q_len[0] - 1) begin
                r_beat = 0;
                void'(ar_q_addr.pop_front());
                void'(ar_q_len.pop_front());
            end else r_beat++;
        end
    end

    // ---------------- U01 ----------------
    int tready_pct = 70;
    always @(posedge clk) tready <= ($urandom % 100) < tready_pct;

    // ---------------- Mô hình vàng & so sánh ----------------
    logic [7:0] exp_bytes [$];
    logic [63:0] exp_hdr;
    int  got_beats, got_bytes;
    bit  seen_hdr, seen_last;
    int  done_cnt = 0;
    bit  done_err;

    function automatic int fl_div(input int a, input int b);   // chia làm tròn xuống
        return (a >= 0) ? a / b : -((-a + b - 1) / b);
    endfunction

    task automatic build_expected(input int x, y, w, h, dl);
        int dec;
        bit has_pad;
        dec = 1 << dl;
        has_pad = 0;
        exp_bytes.delete();
        for (int r = 0; r < h; r++)
            for (int c = 0; c < w; c++) begin
                int sx, sy;
                sx = x + c*dec; sy = y + r*dec;
                for (int b = 0; b < 3; b++)
                    if (sx >= 0 && sx < FRAME_WIDTH && sy >= 0 && sy < FRAME_HEIGHT)
                        exp_bytes.push_back(img[sy*FRAME_STRIDE_B + sx*3 + b]);
                    else begin
                        exp_bytes.push_back(pad_val);
                        has_pad = 1;                 // có ít nhất 1 pixel ngõ ra là pad
                    end
            end
        exp_hdr = {HEADER_MAGIC, 5'b0, 2'(dl), has_pad, desc.frame_id, desc.roi_id, 16'(h), 16'(w)};
    endtask

    always @(posedge clk) if (rst_n) begin
        if (tvalid && tready) begin
            if (!seen_hdr) begin
                if (!tuser[0] || tkeep != 8'hFF || tlast) begin errors++; $display("LOI: beat header sai co"); end
                if (tdata != exp_hdr) begin errors++; $display("LOI: header %h, ky vong %h", tdata, exp_hdr); end
                seen_hdr = 1;
            end else begin
                if (tuser[0]) begin errors++; $display("LOI: tuser o payload"); end
                if (seen_last) begin errors++; $display("LOI: beat sau tlast"); end
                for (int b = 0; b < 8; b++) begin
                    if (tkeep[b]) begin
                        if (got_bytes >= exp_bytes.size()) begin errors++; $display("LOI: thua byte"); end
                        else if (tdata[b*8 +: 8] !== exp_bytes[got_bytes]) begin
                            if (errors < 10) $display("LOI: byte %0d = %h, ky vong %h", got_bytes,
                                                      tdata[b*8 +: 8], exp_bytes[got_bytes]);
                            errors++;
                        end
                        got_bytes++;
                    end
                end
                if (!tlast && tkeep != 8'hFF) begin errors++; $display("LOI: tkeep thieu o beat giua"); end
                if (tlast && tkeep != 8'(2**((exp_bytes.size()-1)%8+1)-1)) begin errors++; $display("LOI: tkeep beat cuoi %h", tkeep); end
                if (tlast) seen_last = 1;
                got_beats++;
            end
        end
        if (roi_done) begin done_cnt++; done_err = roi_err; end
    end

    task automatic step(); @(posedge clk); #1; endtask

    int n_roi = 0;
    task automatic run_roi(input int x, y, w, h, dl, input bit err = 0);
        int guard, exp_rbeats;
        desc          = '0;
        desc.roi_x    = 12'(x);
        desc.roi_y    = 12'(y);
        desc.roi_w    = 8'(w);
        desc.roi_h    = 8'(h);
        desc.dec_log2 = 2'(dl);
        desc.roi_id   = 8'(n_roi);
        desc.frame_id = 8'(n_roi >> 8);
        build_expected(x, y, w, h, dl);
        // số beat R dự đoán: mỗi hàng trong khung đọc [first, last] pixel hợp lệ
        exp_rbeats = 0;
        begin
            int dec, c0, c1;
            dec = 1 << dl;
            c0 = -1; c1 = -1;
            for (int c = 0; c < w; c++)
                if (x + c*dec >= 0 && x + c*dec < FRAME_WIDTH) begin if (c0 < 0) c0 = c; c1 = c; end
            for (int r = 0; r < h; r++) begin
                int sy;
                sy = y + r*dec;
                if (c0 >= 0 && sy >= 0 && sy < FRAME_HEIGHT) begin
                    int a0, a1;
                    a0 = sy*FRAME_STRIDE_B + (x + c0*dec)*3;
                    a1 = sy*FRAME_STRIDE_B + (x + c1*dec)*3 + 2;
                    exp_rbeats += (a1 >> 3) - (a0 >> 3) + 1;
                end
            end
        end
        seen_hdr = 0; seen_last = 0; got_beats = 0; got_bytes = 0;
        inject_err = err; err_done = 0;
        n_rbeats = 0;
        start = 1; step(); start = 0;
        guard = 0;
        while (done_cnt == n_roi && guard < 200000) begin step(); guard++; end
        repeat (2) step();
        if (guard >= 200000) begin errors++; $display("LOI: ROI %0d treo (x=%0d y=%0d %0dx%0d dec=%0d)", n_roi, x, y, w, h, 1<<dl); end
        if (got_bytes != exp_bytes.size() || !seen_last) begin
            errors++; $display("LOI ROI %0d: %0d/%0d byte, tlast=%0d", n_roi, got_bytes, exp_bytes.size(), seen_last);
        end
        if (n_rbeats != exp_rbeats) begin errors++; $display("LOI ROI %0d: %0d beat R, ky vong %0d", n_roi, n_rbeats, exp_rbeats); end
        if (done_err != (err && err_done)) begin errors++; $display("LOI ROI %0d: roi_err=%0d ky vong %0d", n_roi, done_err, err && err_done); end
        n_roi++;
        if (($urandom % 3) == 0) repeat ($urandom % 10) step();
    endtask

    initial begin
        int e0;
        repeat (3) step();
        rst_n = 1;
        repeat (2) step();

        // ---------- Định hướng ----------
        run_roi(0, 0, 16, 16, 0);
        run_roi(100, 50, 128, 128, 0);
        run_roi(-5, 7, 33, 17, 1);               // ví dụ tài liệu (x = -5, dec = 2)
        run_roi(-200, 0, 16, 16, 0);              // hoàn toàn bên trái
        run_roi(700, 10, 16, 20, 2);              // hoàn toàn bên phải
        run_roi(10, -300, 20, 16, 1);             // hoàn toàn phía trên
        run_roi(10, 470, 20, 30, 0);              // tràn dưới
        run_roi(577, 3, 16, 16, 2);               // không có pad dù x + W*dec > 640 (review mục 4)
        run_roi(-37, -41, 128, 128, 2);           // tràn trái + trên, dec 4
        run_roi(630, 475, 128, 128, 0);           // góc phải dưới
        run_roi(1, 1, 17, 19, 0);                 // W*3 lẻ, beat cuối thiếu
        pad_val = 8'h80;
        run_roi(-3, -3, 21, 16, 1);               // pad khác 0
        tready_pct = 100;
        run_roi(0, 0, 128, 128, 0);               // không backpressure
        tready_pct = 15;
        run_roi(5, 5, 64, 32, 1);                 // backpressure nặng
        tready_pct = 70;
        run_roi(200, 200, 40, 40, 0, 1);          // RRESP lỗi
        $display("Dinh huong: %0d ROI, loi %0d", n_roi, errors);

        // ---------- Ngẫu nhiên ----------
        e0 = errors;
        for (int i = 0; i < 400; i++) begin
            int w, h, dl, x, y;
            w  = 16 + $urandom % 113;
            h  = 16 + $urandom % 113;
            dl = $urandom % 3;
            x  = int'($urandom % 1000) - 300;
            y  = int'($urandom % 800) - 200;
            pad_val    = 8'($urandom);
            tready_pct = 20 + $urandom % 81;
            run_roi(x, y, w, h, dl, ($urandom % 20) == 0);
        end
        $display("Ngau nhien: 400 ROI, loi %0d", errors - e0);

        if (errors == 0) $display("==== tb_read_path: PASS ====");
        else             $display("==== tb_read_path: FAIL (%0d loi) ====", errors);
        $finish;
    end
endmodule
