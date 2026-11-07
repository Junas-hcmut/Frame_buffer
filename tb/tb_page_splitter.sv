`timescale 1ns / 1ps

// =====================================================================
// tb_page_splitter
//  - Golden model: bẻ mỗi yêu cầu tại mọi ranh giới 128 B.
//  - Kiểm tra mọi burst con: addr, len, arlen, roi/frame id, eol/eof,
//    first/last/split.
//  - Bất biến AXI: 1 <= len <= 128, không vượt khối 128 B, không vượt
//    4 KB, arlen <= 15, arlen khớp công thức theo byte lệch đầu.
//  - Giao thức: khi valid=1 và ready=0 thì valid và mọi trường phải giữ
//    nguyên ở chu kỳ sau; không có burst "ma" khi không có yêu cầu.
//  - Thông lượng: sau khi 1 burst con (chưa phải cuối) được nhận, burst
//    con kế tiếp phải valid ngay chu kỳ sau (không có bọt).
//  - Ca hiếm: len = 0, 1 byte ở 8 vị trí lệch, byte cuối trang 4 KB,
//    4095 B, đỉnh không gian địa chỉ, stall dài, ready bật/tắt xen kẽ,
//    reset giữa chừng, dòng RGB thực tế cho cả 480 dòng khung hình.
// =====================================================================
module tb_page_splitter;
    import u02_pkg::*;

    localparam int BLK = MAX_BURST_BEATS * (AXI_DATA_WIDTH / 8);   // 128
    parameter  int N_RAND = 10000;   // đổi bằng: vsim -gN_RAND=20000 ...

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    int unsigned cyc = 0;
    always @(posedge clk) cyc++;

    // ---------------- DUT ----------------
    logic        s_req_valid = 0, s_req_ready;
    logic [31:0] s_req_addr = 0;
    logic [11:0] s_req_len = 0;
    logic [7:0]  s_req_roi_id = 0, s_req_frame_id = 0;
    logic        s_req_eol = 0, s_req_eof = 0;

    logic        m_req_valid, m_req_ready = 1;
    logic [31:0] m_req_addr;
    logic [11:0] m_req_len;
    logic [7:0]  m_req_arlen, m_req_roi_id, m_req_frame_id;
    logic        m_req_eol, m_req_eof, m_req_first, m_req_last, m_req_split;
    logic        err_zero_len;

    page_splitter dut (
        .clk(clk), .rst_n(rst_n),
        .s_req_valid(s_req_valid), .s_req_ready(s_req_ready), .s_req_addr(s_req_addr),
        .s_req_len(s_req_len), .s_req_roi_id(s_req_roi_id), .s_req_frame_id(s_req_frame_id),
        .s_req_eol(s_req_eol), .s_req_eof(s_req_eof),
        .m_req_valid(m_req_valid), .m_req_ready(m_req_ready), .m_req_addr(m_req_addr),
        .m_req_len(m_req_len), .m_req_arlen(m_req_arlen), .m_req_roi_id(m_req_roi_id),
        .m_req_frame_id(m_req_frame_id), .m_req_eol(m_req_eol), .m_req_eof(m_req_eof),
        .m_req_first(m_req_first), .m_req_last(m_req_last), .m_req_split(m_req_split),
        .err_zero_len(err_zero_len)
    );

    // ---------------- Điều khiển ready phía downstream ----------------
    // 0: luôn 1 | 1: ngẫu nhiên theo ready_pct | 2: bật/tắt xen kẽ | 3: luôn 0 | 4: stall theo cụm
    int ready_mode = 0, ready_pct = 100, burst_left = 0;
    always @(posedge clk) begin
        case (ready_mode)
            0: m_req_ready <= 1'b1;
            1: m_req_ready <= ($urandom % 100) < ready_pct;
            2: m_req_ready <= ~m_req_ready;
            3: m_req_ready <= 1'b0;
            4: begin
                   if (burst_left == 0) begin
                       burst_left  = 1 + $urandom % 20;
                       m_req_ready <= ($urandom % 2);
                   end else begin
                       burst_left--;
                   end
               end
            default: m_req_ready <= 1'b1;
        endcase
    end

    // ---------------- Golden model ----------------
    typedef struct {
        logic [31:0] addr;
        int          len;
        int          arlen;
        logic [7:0]  roi, fid;
        bit          eol, eof, first, last, split;
    } exp_t;

    exp_t expq[$];
    int   exp_zero = 0;

    // Thống kê / coverage
    int n_req = 0, n_chunk = 0, n_split_req = 0, max_chunks = 0, n_page_cross = 0;
    int n_1byte = 0, n_full_blk = 0, n_stall = 0, n_zero_seen = 0, n_len4095 = 0, n_arlen0_unaligned = 0;

    function automatic void model(input logic [31:0] addr, input int len,
                                  input logic [7:0] roi, fid, input bit eol, eof);
        longint unsigned a = addr;
        int rem = len, k = 0, chunk, to_blk;
        exp_t e;
        if (len == 0) begin exp_zero++; return; end
        n_req++;
        if (len == 4095) n_len4095++;
        if ((addr % 4096) + len > 4096) n_page_cross++;
        while (rem > 0) begin
            to_blk  = BLK - int'(a % BLK);
            chunk   = (rem < to_blk) ? rem : to_blk;
            e.addr  = a[31:0];
            e.len   = chunk;
            e.arlen = (int'(a % 8) + chunk + 7) / 8 - 1;
            e.roi   = roi;  e.fid = fid;
            e.first = (k == 0);
            e.last  = (rem == chunk);
            e.split = !(e.first && e.last);
            e.eol   = eol && e.last;
            e.eof   = eof && e.last;
            expq.push_back(e);
            a   += chunk;
            rem -= chunk;
            k++;
        end
        if (k > 1) n_split_req++;
        if (k > max_chunks) max_chunks = k;
    endfunction

    // ---------------- Monitor + scoreboard + kiểm tra giao thức ----------------
    int err = 0, err_shown = 0;
    task automatic report(input string msg);
        err++;
        if (err_shown < 20) begin
            $display("[tb_page_splitter][ERR @%0t] %s", $time, msg);
            err_shown++;
        end
    endtask

    bit          prev_stall = 0;
    logic [31:0] p_addr;  logic [11:0] p_len;  logic [7:0] p_arlen, p_roi, p_fid;
    logic        p_eol, p_eof, p_first, p_last, p_split;
    int unsigned last_hs_cyc = 0;
    bit          last_hs_was_not_last = 0;
    exp_t        e;

    always @(posedge clk) begin
        if (!rst_n) begin
            expq.delete();
            prev_stall = 0;
            last_hs_was_not_last = 0;
        end else begin
            // Upstream handshake -> sinh kỳ vọng
            if (s_req_valid && s_req_ready)
                model(s_req_addr, int'(s_req_len), s_req_roi_id, s_req_frame_id, s_req_eol, s_req_eof);

            if (err_zero_len) n_zero_seen++;

            // Giữ ổn định khi bị stall
            if (prev_stall) begin
                if (!m_req_valid)
                    report("valid rot xuong khi dang stall (vi pham AXI)");
                else if (m_req_addr !== p_addr || m_req_len !== p_len || m_req_arlen !== p_arlen ||
                         m_req_roi_id !== p_roi || m_req_frame_id !== p_fid || m_req_eol !== p_eol ||
                         m_req_eof !== p_eof || m_req_first !== p_first || m_req_last !== p_last ||
                         m_req_split !== p_split)
                    report("du lieu dau ra thay doi khi dang stall (vi pham AXI)");
            end

            if (m_req_valid && !m_req_ready) n_stall++;

            // Downstream handshake -> so với kỳ vọng
            if (m_req_valid && m_req_ready) begin
                n_chunk++;
                // Bất biến
                if (m_req_len == 0 || m_req_len > BLK)
                    report($sformatf("len=%0d ngoai [1,%0d]", m_req_len, BLK));
                if (int'(m_req_addr % BLK) + int'(m_req_len) > BLK)
                    report($sformatf("vuot khoi %0d B: addr=%h len=%0d", BLK, m_req_addr, m_req_len));
                if (int'(m_req_addr % 4096) + int'(m_req_len) > 4096)
                    report($sformatf("VUOT 4KB: addr=%h len=%0d", m_req_addr, m_req_len));
                if (m_req_arlen > MAX_BURST_BEATS - 1)
                    report($sformatf("arlen=%0d > %0d", m_req_arlen, MAX_BURST_BEATS - 1));
                if (m_req_len == 1) n_1byte++;
                if (m_req_len == BLK) n_full_blk++;
                if (m_req_arlen == 0 && m_req_addr[2:0] != 0) n_arlen0_unaligned++;

                if (expq.size() == 0) begin
                    report($sformatf("burst ngoai mong doi: addr=%h len=%0d", m_req_addr, m_req_len));
                end else begin
                    e = expq.pop_front();
                    if (m_req_addr !== e.addr || int'(m_req_len) != e.len || int'(m_req_arlen) != e.arlen ||
                        m_req_roi_id !== e.roi || m_req_frame_id !== e.fid || m_req_eol !== e.eol ||
                        m_req_eof !== e.eof || m_req_first !== e.first || m_req_last !== e.last ||
                        m_req_split !== e.split)
                        report($sformatf("sai burst: ky vong addr=%h len=%0d arlen=%0d eol=%0b eof=%0b f/l/s=%0b%0b%0b | thuc te addr=%h len=%0d arlen=%0d eol=%0b eof=%0b f/l/s=%0b%0b%0b",
                               e.addr, e.len, e.arlen, e.eol, e.eof, e.first, e.last, e.split,
                               m_req_addr, m_req_len, m_req_arlen, m_req_eol, m_req_eof,
                               m_req_first, m_req_last, m_req_split));
                end

                last_hs_cyc          = cyc;
                last_hs_was_not_last = !m_req_last;
            end

            // Thông lượng: ngay sau khi 1 burst con (chưa phải cuối) được nhận,
            // burst con kế tiếp phải valid ở chu kỳ liền sau (không có bọt)
            if (last_hs_was_not_last && cyc == last_hs_cyc + 1 && !m_req_valid)
                report("co bot giua 2 burst con cua cung yeu cau");

            prev_stall = m_req_valid && !m_req_ready;
            p_addr = m_req_addr;  p_len = m_req_len;  p_arlen = m_req_arlen;
            p_roi = m_req_roi_id; p_fid = m_req_frame_id; p_eol = m_req_eol; p_eof = m_req_eof;
            p_first = m_req_first; p_last = m_req_last; p_split = m_req_split;
        end
    end

    // ---------------- Driver (tuân thủ AXI: giữ valid tới khi bắt tay) ----------------
    task automatic send(input logic [31:0] addr, input int len, input logic [7:0] roi, fid,
                        input bit eol, eof, input int gap = 0);
        repeat (gap) @(negedge clk);
        @(negedge clk);
        s_req_valid    = 1'b1;
        s_req_addr     = addr;
        s_req_len      = 12'(len);
        s_req_roi_id   = roi;
        s_req_frame_id = fid;
        s_req_eol      = eol;
        s_req_eof      = eof;
        do @(posedge clk); while (!s_req_ready);
        @(negedge clk);
        s_req_valid    = 1'b0;
        s_req_addr     = $urandom;          // rác khi valid = 0, DUT không được dùng
        s_req_len      = 12'($urandom);
    endtask

    task automatic drain(input string name);
        int t = 0;
        while ((expq.size() > 0 || m_req_valid) && t < 200000) begin @(posedge clk); t++; end
        repeat (3) @(posedge clk);
        if (expq.size() != 0) report($sformatf("%s: TIMEOUT, con %0d burst chua ra", name, expq.size()));
        $display("[tb_page_splitter] %-44s xong (loi tich luy: %0d)", name, err);
    endtask

    // ---------------- Kịch bản ----------------
    int n_req_before;
    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        $display("[tb_page_splitter] ===== BAT DAU =====");

        // D1: 1 byte tại 8 vị trí lệch trong 1 beat
        ready_mode = 0;
        for (int o = 0; o < 8; o++) send(32'h8000_0100 + o, 1, 8'd1, 8'd1, 1, 0);
        drain("D1  1 byte o 8 vi tri lech");

        // D2: vắt qua biên beat
        send(32'h8000_0105, 4, 8'd2, 8'd1, 1, 0);          // 5..8 -> 2 beat
        send(32'h8000_0101, 8, 8'd2, 8'd1, 1, 0);          // 2 beat
        send(32'h8000_0107, 2, 8'd2, 8'd1, 1, 0);          // 2 beat
        drain("D2  vat qua bien beat (arlen=1)");

        // D3..D6: quanh biên khối 128 B
        send(32'h8000_0200, 128, 8'd3, 8'd1, 1, 0);        // đúng 1 khối, arlen=15
        send(32'h8000_0281, 127, 8'd4, 8'd1, 1, 0);        // lấp đúng tới cuối khối
        send(32'h8000_0301, 128, 8'd5, 8'd1, 1, 0);        // 127 + 1
        send(32'h8000_0380, 129, 8'd6, 8'd1, 1, 0);        // 128 + 1
        send(32'h8000_047F, 2, 8'd6, 8'd1, 1, 0);          // 1 + 1 qua biên khối
        drain("D3-6 quanh bien khoi 128 B");

        // D7..D8: byte cuối trang 4 KB
        send(32'h8000_0FFF, 1, 8'd7, 8'd1, 1, 1);          // 1 byte cuối trang
        send(32'h8000_1FFF, 2, 8'd8, 8'd1, 1, 1);          // 1 + 1, burst 2 ở đầu trang mới
        send(32'h8000_2F81, 200, 8'd8, 8'd1, 1, 0);        // qua trang, lệch lẻ
        drain("D7-8 byte cuoi trang 4 KB");

        // D9..D11: độ dài cực đại 4095 B
        send(32'h8000_0000, 4095, 8'd9,  8'd2, 1, 1);      // 32 burst con
        send(32'h8000_0FFF, 4095, 8'd10, 8'd2, 1, 1);      // bắt đầu tại byte cuối trang
        send(32'h8000_0F81, 4095, 8'd11, 8'd2, 1, 1);      // lệch lẻ
        drain("D9-11 do dai cuc dai 4095 B");

        // D12..D13: len = 0 (phải bỏ, báo lỗi, không làm hỏng yêu cầu kế tiếp)
        send(32'h8000_0500, 0, 8'd12, 8'd3, 1, 1);
        send(32'h8000_0510, 64, 8'd12, 8'd3, 1, 1);
        send(32'h8000_0600, 0, 8'd13, 8'd3, 0, 0);
        send(32'h8000_0600, 0, 8'd13, 8'd3, 0, 0);
        send(32'h8000_0700, 3, 8'd13, 8'd3, 1, 0);
        drain("D12-13 len = 0");

        // D14: đỉnh không gian địa chỉ (byte cuối cùng = 0xFFFF_FFFF)
        send(32'hFFFF_FF00, 256, 8'd14, 8'd4, 1, 1);
        send(32'hFFFF_FFFF, 1, 8'd14, 8'd4, 1, 1);
        drain("D14 dinh khong gian dia chi");

        // D15: stall dài khi đầu ra đang valid, rồi ready bật/tắt xen kẽ
        ready_mode = 3;
        send(32'h8000_3000, 1000, 8'd15, 8'd5, 1, 0);
        repeat (60) @(posedge clk);
        ready_mode = 2;
        send(32'h8000_4003, 1500, 8'd15, 8'd5, 1, 1);
        drain("D15 stall 60 chu ky + ready xen ke");

        // D16: reset giữa chừng
        ready_mode = 1; ready_pct = 50;
        send(32'h8000_5000, 4095, 8'd16, 8'd6, 1, 1);
        repeat (7) @(posedge clk);
        @(negedge clk); rst_n = 0;
        #1;
        if (m_req_valid !== 1'b0) report("m_req_valid khong ve 0 ngay khi reset");
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);
        if (m_req_valid !== 1'b0) report("co burst sot lai sau reset");
        send(32'h8000_6001, 300, 8'd17, 8'd6, 1, 1);
        drain("D16 reset giua chung roi chay tiep");

        // D17: dòng RGB thực tế, ROI 128 px, dec=1 và dec=4, cả 480 dòng
        ready_mode = 0;
        for (int y = 0; y < FRAME_HEIGHT; y++)
            send(FB0_BASE_ADDR + y * FRAME_STRIDE_B + 213 * PIXEL_BYTES, 128 * PIXEL_BYTES,
                 8'd18, 8'd7, 1, (y == FRAME_HEIGHT - 1));
        for (int y = 0; y < FRAME_HEIGHT; y += 4)
            send(FB1_BASE_ADDR + y * FRAME_STRIDE_B + 1 * PIXEL_BYTES, 4 * 128 * PIXEL_BYTES,
                 8'd19, 8'd8, 1, (y == FRAME_HEIGHT - 4));
        drain("D17 dong RGB thuc te (480 dong)");

        // R: N_RAND yêu cầu ngẫu nhiên, phân bố dồn về các biên
        n_req_before = n_req;
        for (int i = 0; i < N_RAND; i++) begin
            logic [31:0] a;
            int len, sel;
            if (i % (N_RAND / 10) == 0) begin
                ready_mode = $urandom % 5;
                ready_pct  = 10 + $urandom % 91;
                if (ready_mode == 3) ready_mode = 4;      // tránh treo vĩnh viễn
            end
            sel = $urandom % 10;
            case (sel)
                0:       a = $urandom | 32'h0000_0FF8;                           // 8 byte cuối trang
                1:       a = ($urandom & 32'hFFFF_F000) | (12'hFFF - ($urandom % 8));
                2:       a = ($urandom & 32'hFFFF_FF80) | (7'h7F - ($urandom % 4)); // gần cuối khối
                3:       a = $urandom & 32'hFFFF_FF80;                           // căn khối
                default: a = $urandom;
            endcase
            sel = $urandom % 10;
            case (sel)
                0:       len = 1 + $urandom % 8;
                1:       len = 126 + $urandom % 5;             // 126..130
                2:       len = 4090 + $urandom % 6;            // 4090..4095
                3:       len = ($urandom % 50 == 0) ? 0 : 384; // dòng 128 px, thỉnh thoảng 0
                4:       len = 1536;
                default: len = 1 + $urandom % 4095;
            endcase
            if (longint'(a) + len > 64'h1_0000_0000) a = a - len;  // không vượt đỉnh 4 GB
            send(a, len, 8'(i), 8'(i >> 8), $urandom % 2, (i % 97 == 0), ($urandom % 4 == 0) ? $urandom % 5 : 0);
        end
        ready_mode = 0;
        drain($sformatf("R   %0d yeu cau ngau nhien", N_RAND));

        // ---------------- Tổng kết ----------------
        if (n_zero_seen != exp_zero)
            report($sformatf("so xung err_zero_len %0d != so yeu cau len=0 %0d", n_zero_seen, exp_zero));
        $display("[tb_page_splitter] ----- COVERAGE -----");
        $display("[tb_page_splitter] yeu cau hop le %0d, len=0 %0d (err_zero_len %0d)", n_req, exp_zero, n_zero_seen);
        $display("[tb_page_splitter] burst con %0d, yeu cau bi be %0d, toi da %0d burst/yeu cau", n_chunk, n_split_req, max_chunks);
        $display("[tb_page_splitter] yeu cau vat qua 4 KB %0d, len=4095 %0d", n_page_cross, n_len4095);
        $display("[tb_page_splitter] burst 1 byte %0d, burst du 128 B %0d, arlen=0 lech le %0d", n_1byte, n_full_blk, n_arlen0_unaligned);
        $display("[tb_page_splitter] chu ky stall (valid & !ready) %0d", n_stall);
        $display("[tb_page_splitter] ===== TONG LOI: %0d -> %s =====", err, (err == 0) ? "PASS" : "FAIL");
        $finish;
    end

    // Watchdog
    initial begin
        #1000ms;
        $display("[tb_page_splitter] WATCHDOG: mo phong treo");
        $finish;
    end
endmodule
