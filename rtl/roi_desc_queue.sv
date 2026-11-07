`timescale 1ns / 1ps

// Hàng đợi descriptor ROI (FIFO, đầu ra FWFT).
// - Nhận push cả khi đầy nếu cùng chu kỳ có pop (ô vừa được giải phóng).
// - Push bị từ chối -> xung overflow để regfile chốt cờ lỗi (không mất im lặng).
module roi_desc_queue
    import u02_pkg::*;
#(
    parameter int DEPTH = ROI_QUEUE_DEPTH           // bắt buộc là lũy thừa của 2
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Ghi (từ regfile)
    input  logic                    push,
    input  u02_roi_desc_t           in_desc,
    output logic                    push_ok,        // xung: push được nhận
    output logic                    overflow,       // xung: push bị từ chối vì đầy

    // Đọc (sang scheduler / addr_gen_2d)
    input  logic                    pop,
    output u02_roi_desc_t           out_desc,       // FWFT: hợp lệ khi valid = 1
    output logic                    valid,

    // Trạng thái
    output logic                    empty,
    output logic                    full,
    output logic                    almost_full,
    output logic [$clog2(DEPTH):0]  count
);

    localparam int PW = $clog2(DEPTH);

    // synopsys translate_off
    initial begin
        if ((1 << PW) != DEPTH)
            $fatal(1, "roi_desc_queue: DEPTH=%0d khong phai luy thua cua 2", DEPTH);
    end
    // synopsys translate_on

    u02_roi_desc_t mem [DEPTH];
    logic [PW:0]   wr_ptr, rd_ptr;

    assign count       = wr_ptr - rd_ptr;
    assign empty       = (wr_ptr == rd_ptr);
    assign full        = (count == (PW+1)'(DEPTH));
    assign almost_full = (count >= (PW+1)'(DEPTH - 1));
    assign valid       = !empty;

    wire do_pop  = pop && !empty;
    wire do_push = push && (!full || do_pop);

    assign push_ok  = do_push;
    assign overflow = push && !do_push;

    // Bộ nhớ: không reset (ánh xạ được sang SRAM/register file)
    always_ff @(posedge clk) begin
        if (do_push)
            mem[wr_ptr[PW-1:0]] <= in_desc;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (do_push) wr_ptr <= wr_ptr + 1'b1;
            if (do_pop)  rd_ptr <= rd_ptr + 1'b1;
        end
    end

    assign out_desc = mem[rd_ptr[PW-1:0]];

endmodule
