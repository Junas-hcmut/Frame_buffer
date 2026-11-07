`timescale 1ns / 1ps

module pingpong #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 12,  // Dung lượng tối đa: 4096 bytes / Bank
    parameter USER_WIDTH = 4
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // Giao tiếp AXI4-Stream Slave (Nhận từ crop_system)
    input  logic                   s_axis_tvalid,
    output logic                   s_axis_tready,
    input  logic [DATA_WIDTH-1:0]  s_axis_tdata,
    input  logic                   s_axis_tlast,
    input  logic [USER_WIDTH-1:0]  s_axis_tuser,

    // Giao tiếp Đọc của NPU (Read Port)
    input  logic                   npu_rd_en,
    input  logic [ADDR_WIDTH-1:0]  npu_rd_addr,
    output logic [DATA_WIDTH-1:0]  npu_rd_data,

    // Điều khiển chuyển đổi Bank
    input  logic                   npu_rd_done,     // NPU báo đã xử lý xong ROI hiện tại
    output logic                   buffer_ready_npu // Báo cho NPU có dữ liệu mới để tính toán
);

    localparam RAM_DEPTH = 1 << ADDR_WIDTH;

    // 2 Bank BRAM vật lý
    logic [DATA_WIDTH-1:0] ram_bank0 [0:RAM_DEPTH-1];
    logic [DATA_WIDTH-1:0] ram_bank1 [0:RAM_DEPTH-1];

    // Cờ quản lý hoán đổi Bank: 0 -> Ghi Bank 0, Đọc Bank 1 | 1 -> Ghi Bank 1, Đọc Bank 0
    logic wr_bank;
    logic rd_bank;
    logic [ADDR_WIDTH-1:0] wr_addr_cnt;
    logic bank0_has_data, bank1_has_data;

    // NPU luôn có thể đọc từ Bank rảnh rỗi chứa dữ liệu
    assign buffer_ready_npu = (rd_bank == 1'b0) ? bank0_has_data : bank1_has_data;

    // Ghi luồng AXI-Stream: Luôn nhận nếu Bank đang ghi chưa bị đầy
    wire cur_wr_bank_ready = (wr_bank == 1'b0) ? !bank0_has_data : !bank1_has_data;
    assign s_axis_tready   = cur_wr_bank_ready;

    wire wr_en = s_axis_tvalid && s_axis_tready;

    // Ghi dữ liệu vào Bank tương ứng
    always_ff @(posedge clk) begin
        if (wr_en) begin
            if (wr_bank == 1'b0)
                ram_bank0[wr_addr_cnt] <= s_axis_tdata;
            else
                ram_bank1[wr_addr_cnt] <= s_axis_tdata;
        end
    end

    // Đọc dữ liệu ra NPU từ Bank tương ứng
    always_ff @(posedge clk) begin
        if (npu_rd_en) begin
            if (rd_bank == 1'b0)
                npu_rd_data <= ram_bank0[npu_rd_addr];
            else
                npu_rd_data <= ram_bank1[npu_rd_addr];
        end
    end

    // Điều khiển hoán đổi Ping-Pong Bank
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_bank         <= 1'b0;
            rd_bank         <= 1'b0;
            wr_addr_cnt     <= '0;
            bank0_has_data  <= 1'b0;
            bank1_has_data  <= 1'b0;
        end else begin
            // Tự động tăng con trỏ ghi địa chỉ
            if (wr_en) begin
                wr_addr_cnt <= wr_addr_cnt + 1'b1;

                // Tín hiệu tlast kết thúc một vùng ROI được gửi tới
                if (s_axis_tlast) begin
                    wr_addr_cnt <= '0;
                    if (wr_bank == 1'b0) begin
                        bank0_has_data <= 1'b1;
                        wr_bank        <= 1'b1; // Đổi sang ghi Bank 1
                    end else begin
                        bank1_has_data <= 1'b1;
                        wr_bank        <= 1'b0; // Đổi sang ghi Bank 0
                    end
                end
            end

            // NPU hoàn thành đọc một ROI
            if (npu_rd_done && buffer_ready_npu) begin
                if (rd_bank == 1'b0) begin
                    bank0_has_data <= 1'b0; // Giải phóng Bank 0
                    rd_bank        <= 1'b1; // Chuyển sang đọc Bank 1
                end else begin
                    bank1_has_data <= 1'b0; // Giải phóng Bank 1
                    rd_bank        <= 1'b0; // Chuyển sang đọc Bank 0
                end
            end
        end
    end

endmodule