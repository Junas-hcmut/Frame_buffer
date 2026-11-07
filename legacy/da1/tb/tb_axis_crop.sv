`timescale 1ns / 1ps

import u02_pkg::*;

module tb_axis_crop;

    localparam DATA_WIDTH = 64;
    localparam KEEP_WIDTH = 8;
    localparam USER_WIDTH = 1;
    localparam CLK_PERIOD = 10;

    logic                  clk;
    logic                  rst_n;

    // Giao tiếp với addr_gen_2d
    logic                  gen_valid;
    logic                  gen_ready;
    logic                  gen_pad;
    logic                  gen_eof;
    logic                  gen_has_pad;
    logic [11:0]           gen_w;
    logic [11:0]           gen_h;
    logic [1:0]            gen_dec_log2;
    logic [7:0]            gen_roi_id;
    logic [7:0]            gen_frame_id;

    // Kênh nhận pixel
    logic                  pixel_rvalid;
    logic                  pixel_rready;
    logic [23:0]           pixel_rdata;

    // Giao tiếp AXI4-Stream Master
    logic                  m_axis_tvalid;
    logic                  m_axis_tready;
    logic [DATA_WIDTH-1:0] m_axis_tdata;
    logic [KEEP_WIDTH-1:0] m_axis_tkeep;
    logic                  m_axis_tlast;
    logic [USER_WIDTH-1:0] m_axis_tuser;

    // Biến giám sát Scoreboard
    integer err_cnt   = 0;
    integer match_cnt = 0;

    // Device Under Test (DUT)
    axis_crop #(
        .DATA_WIDTH(DATA_WIDTH),
        .KEEP_WIDTH(KEEP_WIDTH),
        .USER_WIDTH(USER_WIDTH)
    ) dut (.*);

    // Clock Generator (100 MHz)
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Hàng đợi tham chiếu phần mềm (Golden Model Queue)
    typedef struct {
        logic [63:0] data;
        logic [7:0]  keep;
        logic        last;
        logic        user;
    } axis_beat_t;

    axis_beat_t exp_queue[$];
    axis_beat_t exp_beat; // Khai báo tại phạm vi module, tránh lỗi vlog-2244

    // Mô hình tham chiếu: Xây dựng trước toàn bộ gói tin kỳ vọng (Header + Payloads)
    task automatic build_expected_roi(
        input int w, input int h,
        input logic [7:0] rid, fid,
        input logic [1:0] dec,
        input logic has_pad_val,
        input logic is_padded_test
    );
        axis_beat_t beat;
        u02_header_t hdr;
        byte payload_bytes[$];
        int total_bytes;
        logic [23:0] cur_px;

        // 1. Tạo Header Beat
        hdr.magic    = HEADER_MAGIC; // 8'hA5
        hdr.reserved = 5'b0;
        hdr.dec_log2 = dec;
        hdr.has_pad  = has_pad_val;
        hdr.frame_id = fid;
        hdr.roi_id   = rid;
        hdr.h        = 16'(h);
        hdr.w        = 16'(w);

        beat.data = 64'(hdr);
        beat.keep = 8'hFF;
        beat.last = 1'b0;
        beat.user = 1'b1; // Duy nhất ở Header
        exp_queue.push_back(beat);

        // 2. Tạo chuỗi byte RGB Payload
        total_bytes = w * h * 3;
        for (int p = 0; p < w * h; p++) begin
            if (is_padded_test && (p % 2 == 1)) begin
                cur_px = 24'h000000; // Byte pad đen
            end else begin
                cur_px = {8'(p + 2), 8'(p + 1), 8'(p)}; // R, G, B
            end
            payload_bytes.push_back(cur_px[7:0]);   // B
            payload_bytes.push_back(cur_px[15:8]);  // G
            payload_bytes.push_back(cur_px[23:16]); // R
        end

        // 3. Gom Payload thành từng beat 64-bit
        while (payload_bytes.size() > 0) begin
            beat.data = '0;
            beat.keep = 8'h00;
            beat.user = 1'b0;
            for (int b = 0; b < 8; b++) begin
                if (payload_bytes.size() > 0) begin
                    beat.data[b*8 +: 8] = payload_bytes.pop_front();
                    beat.keep[b]        = 1'b1;
                end
            end
            beat.last = (payload_bytes.size() == 0);
            exp_queue.push_back(beat);
        end
    endtask

    // Luồng gửi dữ liệu (Driver)
    task automatic send_roi(
        input int w, input int h,
        input logic [7:0] rid, fid,
        input logic [1:0] dec,
        input logic has_pad_val,
        input logic is_padded_test
    );
        int total_px = w * h;
        build_expected_roi(w, h, rid, fid, dec, has_pad_val, is_padded_test);

        @(posedge clk);
        gen_w        <= 12'(w);
        gen_h        <= 12'(h);
        gen_roi_id   <= rid;
        gen_frame_id <= fid;
        gen_dec_log2 <= dec;
        gen_has_pad  <= has_pad_val;

        for (int i = 0; i < total_px; i++) begin
            gen_valid <= 1'b1;
            gen_eof   <= (i == total_px - 1);

            if (is_padded_test && (i % 2 == 1)) begin
                gen_pad      <= 1'b1;
                pixel_rvalid <= 1'b0;
                pixel_rdata  <= '0;
            end else begin
                gen_pad      <= 1'b0;
                pixel_rvalid <= 1'b1;
                pixel_rdata  <= {8'(i + 2), 8'(i + 1), 8'(i)};
            end

            @(posedge clk);
            while (!gen_ready) @(posedge clk);
        end

        gen_valid    <= 1'b0;
        gen_pad      <= 1'b0;
        gen_eof      <= 1'b0;
        pixel_rvalid <= 1'b0;
    endtask

    // Scoreboard tự động đối chiếu ngõ ra AXI4-Stream
    always @(posedge clk) begin
        if (rst_n && m_axis_tvalid && m_axis_tready) begin
            if (exp_queue.size() == 0) begin
                $display("[SCB ERROR] Nhan duoc beat ngoai mong doi tai %0t ps! Data=0x%16h", $time, m_axis_tdata);
                err_cnt++;
            end else begin
                exp_beat = exp_queue.pop_front();
                if (m_axis_tdata !== exp_beat.data || m_axis_tkeep !== exp_beat.keep || 
                    m_axis_tlast !== exp_beat.last || m_axis_tuser !== exp_beat.user) begin
                    $display("[SCB ERROR] Sai beat tai %0t ps!", $time);
                    $display("  -> KY VONG: Data=0x%16h, Keep=0x%02h, Last=%b, User=%b", exp_beat.data, exp_beat.keep, exp_beat.last, exp_beat.user);
                    $display("  -> THUC TE : Data=0x%16h, Keep=0x%02h, Last=%b, User=%b", m_axis_tdata, m_axis_tkeep, m_axis_tlast, m_axis_tuser);
                    err_cnt++;
                end else begin
                    match_cnt++;
                end
            end
        end
    end

    // Kịch bản kiểm thử chính
    initial begin
        rst_n         = 0;
        gen_valid     = 0;
        gen_pad       = 0;
        gen_eof       = 0;
        gen_has_pad   = 0;
        pixel_rvalid  = 0;
        pixel_rdata   = '0;
        m_axis_tready = 1;

        #(CLK_PERIOD * 3);
        rst_n = 1;
        #(CLK_PERIOD * 2);

        $display("\n========================================================");
        $display("     BAT DAU KIEM THU TOAN DIEN AXIS_CROP (64-BIT)      ");
        $display("========================================================");

        // TEST 1: Kích thước chẵn 64-bit (16x16: 768 Bytes -> 96 beats, tkeep = 8'hFF)
        $display("[RUNNING] TEST 1: Kich thuoc chia het cho 8 (16x16: 768 Bytes)...");
        send_roi(16, 16, 8'd1, 8'd10, 2'd0, 1'b0, 1'b0);
        wait (exp_queue.size() == 0);
        $display("   -> [PASS] TEST 1 hoan tat.");

        // TEST 2: Kích thước lẻ byte (17x17: 867 Bytes -> tkeep beat cuoi phai bang 8'h07)
        $display("[RUNNING] TEST 2: Kich thuoc le byte (17x17: 867 Bytes, du 3 bytes)...");
        send_roi(17, 17, 8'd2, 8'd10, 2'd0, 1'b0, 1'b0);
        wait (exp_queue.size() == 0);
        $display("   -> [PASS] TEST 2 hoan tat: tkeep = 8'h07 va tlast duoc chot chinh xac.");

        // TEST 3: Kích thước tối thiểu (16x1: 48 Bytes -> 1 Header + dung 6 beat payload)
        $display("[RUNNING] TEST 3: Kich thuoc toi thieu (16x1: 48 Bytes)...");
        send_roi(16, 1, 8'd3, 8'd10, 2'd0, 1'b0, 1'b0);
        wait (exp_queue.size() == 0);
        $display("   -> [PASS] TEST 3 hoan tat.");

        // TEST 4: Zero-Padding xen ke (Kiem tra chen gia tri den 24'h000000)
        $display("[RUNNING] TEST 4: Kiem tra chen le Zero-Padding (16x2 xen ke pad)...");
        send_roi(16, 2, 8'd4, 8'd10, 2'd1, 1'b1, 1'b1);
        wait (exp_queue.size() == 0);
        $display("   -> [PASS] TEST 4 hoan tat: Du lieu Pad duoc dong goi bit-exact.");

        // TEST 5: Stress Backpressure ngau nhien tu phia U01 NPU
        $display("[RUNNING] TEST 5: Stress test Backpressure Stall (19x19, Ready 60%%)...");
        fork
            begin
                send_roi(19, 19, 8'd5, 8'd10, 2'd2, 1'b1, 1'b0);
            end
            begin
                while (exp_queue.size() > 0 || gen_valid) begin
                    @(posedge clk);
                    m_axis_tready <= (($urandom % 100) >= 40); // 60% ready, 40% stall
                end
                m_axis_tready <= 1'b1;
            end
        join
        wait (exp_queue.size() == 0);
        $display("   -> [PASS] TEST 5 phuc hoi an toan sau cac chu ky stall.");

        #(CLK_PERIOD * 10);
        $display("\n--------------------------------------------------------");
        $display("  Tong so beat khop (Passed Beats): %0d", match_cnt);
        $display("  Tong so beat loi  (Error Beats) : %0d", err_cnt);
        $display("--------------------------------------------------------");
        if (err_cnt == 0 && match_cnt > 0)
            $display(">>> [AXIS_CROP: ALL 5 CORNER TESTS PASSED (BIT-EXACT)] <<<");
        else
            $display(">>> [AXIS_CROP: THAT BAI CO %0d LOI] <<<", err_cnt);
        $display("========================================================\n");
        $stop;
    end

endmodule