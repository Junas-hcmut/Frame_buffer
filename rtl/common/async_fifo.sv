`timescale 1ns / 1ps

// FIFO bất đồng bộ 2 miền xung (Cummings, SNUG 2002):
// con trỏ mã Gray, đồng bộ 2 tầng flip-flop, cờ full/empty tính trước.
// Đầu ra FWFT: rdata hợp lệ khi !rempty.
module async_fifo #(
    parameter int DW = 27,
    parameter int AW = 4            // độ sâu = 2^AW, AW >= 2
)(
    // Miền ghi
    input  logic          wclk,
    input  logic          wrst_n,
    input  logic          winc,
    input  logic [DW-1:0] wdata,
    output logic          wfull,
    // Miền đọc
    input  logic          rclk,
    input  logic          rrst_n,
    input  logic          rinc,
    output logic [DW-1:0] rdata,
    output logic          rempty
);
    logic [DW-1:0] mem [2**AW];

    logic [AW:0] wbin, wgray, rbin, rgray;
    logic [AW:0] wq1_rgray, wq2_rgray;   // rgray đồng bộ sang miền ghi
    logic [AW:0] rq1_wgray, rq2_wgray;   // wgray đồng bộ sang miền đọc

    // ---------------- Miền ghi ----------------
    wire [AW:0] wbin_next  = wbin + {{AW{1'b0}}, (winc & ~wfull)};
    wire [AW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;
    wire        wfull_next = (wgray_next == {~wq2_rgray[AW:AW-1], wq2_rgray[AW-2:0]});

    always_ff @(posedge wclk) begin
        if (winc && !wfull) mem[wbin[AW-1:0]] <= wdata;
    end

    always_ff @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin      <= '0;
            wgray     <= '0;
            wfull     <= 1'b0;
            wq1_rgray <= '0;
            wq2_rgray <= '0;
        end else begin
            wbin      <= wbin_next;
            wgray     <= wgray_next;
            wfull     <= wfull_next;
            wq1_rgray <= rgray;
            wq2_rgray <= wq1_rgray;
        end
    end

    // ---------------- Miền đọc ----------------
    wire [AW:0] rbin_next   = rbin + {{AW{1'b0}}, (rinc & ~rempty)};
    wire [AW:0] rgray_next  = (rbin_next >> 1) ^ rbin_next;
    wire        rempty_next = (rgray_next == rq2_wgray);

    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin      <= '0;
            rgray     <= '0;
            rempty    <= 1'b1;
            rq1_wgray <= '0;
            rq2_wgray <= '0;
        end else begin
            rbin      <= rbin_next;
            rgray     <= rgray_next;
            rempty    <= rempty_next;
            rq1_wgray <= wgray;
            rq2_wgray <= rq1_wgray;
        end
    end

    assign rdata = mem[rbin[AW-1:0]];
endmodule
