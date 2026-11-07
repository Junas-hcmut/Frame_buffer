`timescale 1ns / 1ps

module skid_buffer #(
    parameter DATA_WIDTH = 64,
    parameter USER_WIDTH = 1
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Giao tiếp AXI4-Stream Slave (Upstream)
    input  logic                          s_axis_tvalid,
    output logic                          s_axis_tready,
    input  logic [DATA_WIDTH-1:0]         s_axis_tdata,
    input  logic [(DATA_WIDTH/8)-1:0]     s_axis_tkeep,  // Cổng tkeep đầu vào
    input  logic                          s_axis_tlast,
    input  logic [USER_WIDTH-1:0]         s_axis_tuser,

    // Giao tiếp AXI4-Stream Master (Downstream)
    output logic                          m_axis_tvalid,
    input  logic                          m_axis_tready,
    output logic [DATA_WIDTH-1:0]         m_axis_tdata,
    output logic [(DATA_WIDTH/8)-1:0]     m_axis_tkeep,  // Cổng tkeep đầu ra
    output logic                          m_axis_tlast,
    output logic [USER_WIDTH-1:0]         m_axis_tuser
);

    localparam KEEP_WIDTH    = DATA_WIDTH / 8;
    localparam PAYLOAD_WIDTH = DATA_WIDTH + KEEP_WIDTH + 1 + USER_WIDTH;
    wire [PAYLOAD_WIDTH-1:0] s_payload = {s_axis_tuser, s_axis_tlast, s_axis_tkeep, s_axis_tdata};

    // 2 cấp thanh ghi dữ liệu (KHÔNG cần Reset - tối ưu diện tích ASIC)
    logic [PAYLOAD_WIDTH-1:0] reg_data;
    logic [PAYLOAD_WIDTH-1:0] skid_data;

    // Cờ trạng thái điều khiển (Có Reset)
    logic                     reg_valid;
    logic                     skid_valid;

    // Upstream ready hoàn toàn độc lập với m_axis_tready
    assign s_axis_tready = !skid_valid;

    // 1. Khối Điều khiển Valid (Có Reset)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_valid  <= 1'b0;
            skid_valid <= 1'b0;
        end else begin
            if (m_axis_tready) begin
                reg_valid <= 1'b0;
            end

            if (skid_valid) begin
                if (m_axis_tready || !reg_valid) begin
                    reg_valid  <= 1'b1;
                    skid_valid <= 1'b0;
                end
            end else if (s_axis_tvalid) begin
                if (m_axis_tready || !reg_valid) begin
                    reg_valid <= 1'b1;
                end else begin
                    skid_valid <= 1'b1;
                end
            end
        end
    end

    // 2. Khối Datapath Payload (KHÔNG Reset - Tiết kiệm diện tích)
    always_ff @(posedge clk) begin
        if (skid_valid) begin
            if (m_axis_tready || !reg_valid) begin
                reg_data <= skid_data;
            end
        end else if (s_axis_tvalid) begin
            if (m_axis_tready || !reg_valid) begin
                reg_data <= s_payload;
            end else begin
                skid_data <= s_payload;
            end
        end
    end

    // Ngõ ra
    assign m_axis_tvalid = reg_valid;
    assign {m_axis_tuser, m_axis_tlast, m_axis_tkeep, m_axis_tdata} = reg_data;

endmodule