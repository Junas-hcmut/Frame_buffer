`timescale 1ns/1ps
// =====================================================================
// u01_model: mô hình hành vi của U01 (NPU) phía giao diện với U02
//   Dùng thay U01 thật khi U01 chưa xong. Cùng cổng với U01 thật nên
//   lúc tích hợp chỉ cần đổi tên module ở top / testbench.
//
//   Hành vi theo đặc tả giao diện U02 → U01 (báo cáo, Mục 11.3):
//     - IBUF_N bộ đệm đầu vào, mỗi bộ chứa trọn 1 ROI (≤ 49.152 B)
//     - Nhận 1 gói AXIS / ROI: beat 0 là header (TUSER = 1), sau đó payload HWC,
//       beat cuối TLAST = 1, TKEEP = (1 << n) − 1
//     - Tính xong ROI → nhả bộ đệm → xung u01_ibuf_release (trả 1 credit)
//     - Thời gian tính ≈ CYC_FIXED + W·H·CYC_PER_PIXEL chu kỳ (mặc định ≈ 0,76 ms
//       cho 128 × 128 ở 150 MHz)
//
//   Kiểm tra (đếm lỗi, in tối đa 20 lỗi):
//     - Giao thức AXIS: dữ liệu giữ nguyên khi TVALID = 1 & TREADY = 0
//     - Header: magic 0xA5, 16 ≤ W, H ≤ 128, dec_log2 ≤ 2
//     - TUSER chỉ ở beat đầu; TKEEP = FF ở beat giữa; số byte = W·H·3
//     - Credit: U02 không được bắt đầu gói khi không còn bộ đệm trống
//
//   TREADY_MODE: 0 = luôn 1 (đúng cam kết của U01)
//                1 = ngẫu nhiên theo TREADY_PCT %
//                2 = theo cụm: 1 trong BURST_ON chu kỳ, 0 trong BURST_OFF chu kỳ
//   DUMP = 1: ghi mỗi ROI ra tệp roi_f<fid>_id<rid>_<W>x<H>.hex (1 byte / dòng, HWC)
//             để mô hình phần mềm (Python/C) của U01 đọc lại
// =====================================================================
module u01_model #(
    parameter int IBUF_N        = 1,
    parameter int CYC_PER_PIXEL = 7,
    parameter int CYC_FIXED     = 200,
    parameter int TREADY_MODE   = 0,
    parameter int TREADY_PCT    = 70,
    parameter int BURST_ON      = 40,
    parameter int BURST_OFF     = 10,
    parameter bit DUMP          = 0,
    parameter bit VERBOSE       = 1
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic [63:0] s_axis_tdata,
    input  logic [7:0]  s_axis_tkeep,
    input  logic        s_axis_tlast,
    input  logic [0:0]  s_axis_tuser,
    output logic        u01_ibuf_release,
    // thống kê cho testbench
    output int          roi_recv,
    output int          roi_done,
    output int          errors
);
    // ---------------- Cấu hình lúc chạy (plusargs ghi đè parameter) ----------------
    //   +U01_IBUF_N=n +U01_CPP=n +U01_FIXED=n +U01_TREADY=0|1|2 +U01_PCT=n +U01_DUMP +U01_QUIET
    int  ibuf_n = IBUF_N, cpp = CYC_PER_PIXEL, cfix = CYC_FIXED, tmode = TREADY_MODE, tpct = TREADY_PCT;
    bit  dump = DUMP, verbose = VERBOSE;
    initial begin
        if ($value$plusargs("U01_IBUF_N=%d", ibuf_n)) ;
        if ($value$plusargs("U01_CPP=%d",    cpp))    ;
        if ($value$plusargs("U01_FIXED=%d",  cfix))   ;
        if ($value$plusargs("U01_TREADY=%d", tmode))  ;
        if ($value$plusargs("U01_PCT=%d",    tpct))   ;
        if ($test$plusargs("U01_DUMP"))  dump = 1;
        if ($test$plusargs("U01_QUIET")) verbose = 0;
        $display("U01_MODEL: IBUF_N=%0d, tính = %0d + W*H*%0d chu kỳ, TREADY_MODE=%0d", ibuf_n, cfix, cpp, tmode);
    end

    // ---------------- Trạng thái ----------------
    typedef struct { int w, h, dl, fid, rid; bit has_pad; int nbytes; int t_hdr, t_last; } roi_t;
    roi_t   cur;
    bit     in_pkt;
    int     free_bufs;
    roi_t   compq[$];               // ROI đã nhận đủ, chờ / đang tính
    int     comp_left;              // chu kỳ còn lại của ROI đang tính (−1 = rảnh)
    int     cyc;
    logic [7:0] bytes[$];
    int     fd;
    int     nerr = 0;
    assign  errors = nerr;

    function automatic void err(input string s);
        nerr++;
        if (nerr <= 20) $display("[%0t] U01_MODEL ERR: %s", $time, s);
    endfunction

    // ---------------- TREADY ----------------
    logic rdy_pol;
    int   bcnt;
    always @(posedge clk) begin
        if (!rst_n) begin rdy_pol <= 1; bcnt <= 0; end
        else case (tmode)
            1: rdy_pol <= ($urandom_range(0, 99) < tpct);
            2: begin bcnt <= (bcnt + 1) % (BURST_ON + BURST_OFF); rdy_pol <= (bcnt < BURST_ON); end
            default: rdy_pol <= 1;
        endcase
    end
    // Không chặn header khi hết bộ đệm: U02 phải tự giữ credit, nếu gửi sai thì báo lỗi (bên dưới)
    assign s_axis_tready = rst_n && rdy_pol;

    // ---------------- Kiểm tra ổn định khi bị chặn ----------------
    logic        st_q; logic [63:0] d_q; logic [7:0] k_q; logic l_q, u_q;
    always @(posedge clk) if (rst_n) begin
        if (st_q && (!s_axis_tvalid || s_axis_tdata !== d_q || s_axis_tkeep !== k_q ||
                     s_axis_tlast !== l_q || s_axis_tuser[0] !== u_q))
            err("AXIS thay đổi khi đang bị chặn (TVALID=1, TREADY=0)");
        st_q <= s_axis_tvalid && !s_axis_tready;
        d_q <= s_axis_tdata; k_q <= s_axis_tkeep; l_q <= s_axis_tlast; u_q <= s_axis_tuser[0];
    end else st_q <= 0;

    // ---------------- Nhận gói ----------------
    always @(posedge clk) begin
        if (!rst_n) begin
            in_pkt <= 0; free_bufs <= ibuf_n; comp_left <= -1; cyc <= 0;
            u01_ibuf_release <= 0; roi_recv <= 0; roi_done <= 0;
            compq.delete(); bytes.delete();
        end else begin
            cyc <= cyc + 1;
            u01_ibuf_release <= 0;

            if (s_axis_tvalid && s_axis_tready) begin
                if (!in_pkt) begin
                    // ---- header ----
                    if (!s_axis_tuser[0]) err("beat đầu gói không có TUSER");
                    if (s_axis_tkeep != 8'hFF || s_axis_tlast) err("header sai TKEEP/TLAST");
                    if (s_axis_tdata[63:56] != 8'hA5) err($sformatf("magic = %h", s_axis_tdata[63:56]));
                    cur.w   = int'(s_axis_tdata[15:0]);  cur.h   = int'(s_axis_tdata[31:16]);
                    cur.rid = int'(s_axis_tdata[39:32]); cur.fid = int'(s_axis_tdata[47:40]);
                    cur.has_pad = s_axis_tdata[48];      cur.dl  = int'(s_axis_tdata[50:49]);
                    cur.nbytes = 0; cur.t_hdr = cyc;
                    if (cur.w < 16 || cur.w > 128 || cur.h < 16 || cur.h > 128 || cur.dl > 2)
                        err($sformatf("header ngoài khoảng: W=%0d H=%0d dl=%0d", cur.w, cur.h, cur.dl));
                    if (free_bufs == 0) err("U02 gửi gói khi U01 không còn bộ đệm (sai credit)");
                    free_bufs <= (free_bufs > 0) ? free_bufs - 1 : 0;
                    bytes.delete();
                    in_pkt <= 1;
                end else begin
                    // ---- payload ----
                    if (s_axis_tuser[0]) err("TUSER ở beat payload");
                    if (!s_axis_tlast && s_axis_tkeep != 8'hFF) err("TKEEP != FF ở beat giữa");
                    for (int k = 0; k < 8; k++) if (s_axis_tkeep[k]) bytes.push_back(s_axis_tdata[k*8 +: 8]);
                    if (s_axis_tlast) begin
                        cur.nbytes = bytes.size(); cur.t_last = cyc;
                        if (cur.nbytes != cur.w * cur.h * 3)
                            err($sformatf("ROI %0d: nhận %0d B, header nói %0d B", cur.rid, cur.nbytes, cur.w * cur.h * 3));
                        if (dump) begin
                            fd = $fopen($sformatf("roi_f%0d_id%0d_%0dx%0d.hex", cur.fid, cur.rid, cur.w, cur.h), "w");
                            foreach (bytes[i]) $fdisplay(fd, "%02h", bytes[i]);
                            $fclose(fd);
                        end
                        compq.push_back(cur);
                        roi_recv <= roi_recv + 1;
                        in_pkt <= 0;
                        if (verbose) $display("[%0t] U01_MODEL nhận ROI id=%0d fid=%0d %0dx%0d dec=%0d pad=%0d (%0d beat-chu kỳ)",
                                              $time, cur.rid, cur.fid, cur.w, cur.h, 1 << cur.dl, cur.has_pad, cyc - cur.t_hdr + 1);
                    end
                end
            end

            // ---------------- "Tính" ----------------
            if (comp_left < 0 && compq.size() > 0)
                comp_left <= cfix + compq[0].w * compq[0].h * cpp;
            else if (comp_left > 0)
                comp_left <= comp_left - 1;
            else if (comp_left == 0) begin
                roi_t r; r = compq.pop_front();
                u01_ibuf_release <= 1;
                free_bufs <= free_bufs + 1 - ((s_axis_tvalid && s_axis_tready && !in_pkt) ? 1 : 0);
                roi_done <= roi_done + 1;
                comp_left <= -1;
                if (verbose) $display("[%0t] U01_MODEL xong ROI id=%0d → ibuf_release", $time, r.rid);
            end
        end
    end
endmodule
