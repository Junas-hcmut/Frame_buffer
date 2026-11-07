`timescale 1ns / 1ps
// Kiểm tra pingpong: ghi liên tiếp nhiều ROI qua AXIS (có ngắt tvalid ngẫu nhiên),
// phía NPU đọc lại từng ROI rồi báo npu_rd_done. ROI 5000 B vượt bank 4096 B
// để kiểm tra trường hợp tràn bank.
module tb_pingpong;
    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    logic        s_tvalid = 0, s_tready, s_tlast = 0;
    logic [7:0]  s_tdata  = 0;
    logic        npu_rd_en = 0, npu_rd_done = 0, buffer_ready_npu;
    logic [11:0] npu_rd_addr = 0;
    logic [7:0]  npu_rd_data;

    pingpong #(.DATA_WIDTH(8), .ADDR_WIDTH(12), .USER_WIDTH(4)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready), .s_axis_tdata(s_tdata),
        .s_axis_tlast(s_tlast), .s_axis_tuser(4'd0),
        .npu_rd_en(npu_rd_en), .npu_rd_addr(npu_rd_addr), .npu_rd_data(npu_rd_data),
        .npu_rd_done(npu_rd_done), .buffer_ready_npu(buffer_ready_npu)
    );

    localparam int NR = 6;
    int lens[NR] = '{100, 4096, 64, 5000, 200, 1};

    function automatic logic [7:0] pat(input int r, input int i);
        return 8'((i * 7) + (r * 13) + (i >> 8));
    endfunction

    // ---- Phía ghi (AXIS master) ----
    initial begin : writer
        int i;
        repeat (3) @(posedge clk);
        rst_n = 1;
        for (int r = 0; r < NR; r++) begin
            i = 0;
            while (i < lens[r]) begin
                @(negedge clk);
                s_tvalid = ($urandom % 100) < 80;
                s_tdata  = pat(r, i);
                s_tlast  = (i == lens[r] - 1);
                @(posedge clk);
                if (s_tvalid && s_tready) i++;
            end
            @(negedge clk); s_tvalid = 0; s_tlast = 0;
        end
    end

    // ---- Phía đọc (NPU) ----
    int total_err = 0;
    initial begin : reader
        int e;
        @(posedge rst_n);
        for (int r = 0; r < NR; r++) begin
            @(negedge clk);
            while (!buffer_ready_npu) @(negedge clk);
            e = 0;
            for (int i = 0; i < lens[r]; i++) begin
                @(negedge clk); npu_rd_en = 1; npu_rd_addr = i[11:0];
                @(posedge clk); #1;
                if (npu_rd_data !== pat(r, i)) e++;
            end
            @(negedge clk); npu_rd_en = 0; npu_rd_done = 1;
            @(negedge clk); npu_rd_done = 0;
            $display("[tb_pingpong] ROI %0d: %0d byte, sai %0d byte%s", r, lens[r], e,
                     (lens[r] > 4096) ? "  <-- vuot dung luong bank 4096 B" : "");
            total_err += e;
        end
        $display("[tb_pingpong] tong byte sai: %0d", total_err);
        $finish;
    end

    initial begin
        #10ms;
        $display("[tb_pingpong] TIMEOUT (treo)");
        $finish;
    end
endmodule
