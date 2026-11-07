`timescale 1ns / 1ps

// FIFO đồng bộ, đầu ra FWFT (rd_data hợp lệ khi !empty).
// DEPTH phải là lũy thừa của 2. Bộ nhớ không reset.
module sync_fifo #(
    parameter int DW    = 8,
    parameter int DEPTH = 16
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    wr_en,
    input  logic [DW-1:0]           wr_data,
    output logic                    full,
    input  logic                    rd_en,
    output logic [DW-1:0]           rd_data,
    output logic                    empty,
    output logic [$clog2(DEPTH):0]  count
);
    localparam int PW = $clog2(DEPTH);

    // synopsys translate_off
    initial if ((1 << PW) != DEPTH) $fatal(1, "sync_fifo: DEPTH=%0d khong phai luy thua cua 2", DEPTH);
    // synopsys translate_on

    logic [DW-1:0] mem [DEPTH];
    logic [PW:0]   wptr, rptr;

    wire do_wr = wr_en && !full;
    wire do_rd = rd_en && !empty;

    assign count   = wptr - rptr;
    assign empty   = (wptr == rptr);
    assign full    = (count == (PW+1)'(DEPTH));
    assign rd_data = mem[rptr[PW-1:0]];

    always_ff @(posedge clk) begin
        if (do_wr) mem[wptr[PW-1:0]] <= wr_data;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr <= '0;
            rptr <= '0;
        end else begin
            if (do_wr) wptr <= wptr + 1'b1;
            if (do_rd) rptr <= rptr + 1'b1;
        end
    end
endmodule
