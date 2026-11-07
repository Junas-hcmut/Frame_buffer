`timescale 1ns / 1ps

// Đồng bộ reset: khẳng định bất đồng bộ, nhả đồng bộ theo clk (2 tầng).
module rst_sync (
    input  logic clk,
    input  logic arst_n,
    output logic rst_n
);
    logic q1;
    always_ff @(posedge clk or negedge arst_n) begin
        if (!arst_n) begin
            q1    <= 1'b0;
            rst_n <= 1'b0;
        end else begin
            q1    <= 1'b1;
            rst_n <= q1;
        end
    end
endmodule
