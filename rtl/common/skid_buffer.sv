`timescale 1ns / 1ps
module skid_buffer #(
    parameter DATA_WIDTH = 64,
    parameter KEEP_WIDTH = (DATA_WIDTH / 8),
    parameter USER_WIDTH = 1
)(
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          s_axis_tvalid,
    output logic                          s_axis_tready,
    input  logic [DATA_WIDTH-1:0]         s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0]         s_axis_tkeep,
    input  logic                          s_axis_tlast,
    input  logic [USER_WIDTH-1:0]         s_axis_tuser,
    output logic                          m_axis_tvalid,
    input  logic                          m_axis_tready,
    output logic [DATA_WIDTH-1:0]         m_axis_tdata,
    output logic [KEEP_WIDTH-1:0]         m_axis_tkeep,
    output logic                          m_axis_tlast,
    output logic [USER_WIDTH-1:0]         m_axis_tuser
);
    localparam PAYLOAD_WIDTH = DATA_WIDTH + KEEP_WIDTH + 1 + USER_WIDTH;
    wire [PAYLOAD_WIDTH-1:0] s_payload = {s_axis_tuser, s_axis_tlast, s_axis_tkeep, s_axis_tdata};
    logic [PAYLOAD_WIDTH-1:0] reg_data;
    logic [PAYLOAD_WIDTH-1:0] skid_data;
    logic                     reg_valid;
    logic                     skid_valid;
    assign s_axis_tready = !skid_valid;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_valid  <= 1'b0;
            skid_valid <= 1'b0;
            reg_data   <= '0;
            skid_data  <= '0;
        end else begin
            if (m_axis_tready) begin
                reg_valid <= 1'b0;
            end
            if (skid_valid) begin
                if (m_axis_tready || !reg_valid) begin
                    reg_data   <= skid_data;
                    reg_valid  <= 1'b1;
                    skid_valid <= 1'b0;
                end
            end else if (s_axis_tvalid) begin
                if (m_axis_tready || !reg_valid) begin
                    reg_data  <= s_payload;
                    reg_valid <= 1'b1;
                end else begin
                    skid_data  <= s_payload;
                    skid_valid <= 1'b1;
                end
            end
        end
    end
    assign m_axis_tvalid = reg_valid;
    assign {m_axis_tuser, m_axis_tlast, m_axis_tkeep, m_axis_tdata} = reg_data;
endmodule
