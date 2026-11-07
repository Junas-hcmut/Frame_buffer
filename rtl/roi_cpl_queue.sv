`timescale 1ns / 1ps

// Hàng đợi completion descriptor, phần mềm đọc qua regfile.
// Hai nguồn đẩy: A = scheduler (ưu tiên), B = descriptor sai từ regfile.
// Nếu A và B cùng chu kỳ, B được giữ 1 chu kỳ rồi đẩy sau (B cách nhau
// tối thiểu 2 chu kỳ vì mỗi lần push qua APB mất >= 2 chu kỳ).
// Đầy -> bỏ entry và phát xung ovf_evt.
module roi_cpl_queue
    import u02_pkg::*;
#(
    parameter int DEPTH = ROI_QUEUE_DEPTH
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    push_a,
    input  u02_roi_cpl_t            din_a,
    input  logic                    push_b,
    input  u02_roi_cpl_t            din_b,
    input  logic                    pop,
    output u02_roi_cpl_t            dout,
    output logic                    valid,
    output logic                    full,
    output logic [$clog2(DEPTH):0]  count,
    output logic                    ovf_evt
);

    logic         b_pend;
    u02_roi_cpl_t b_hold;

    // Chọn nguồn ghi trong chu kỳ này
    logic         wr_en;
    u02_roi_cpl_t wr_data;
    always_comb begin
        wr_en   = 1'b0;
        wr_data = din_a;
        if (push_a) begin
            wr_en   = 1'b1;
            wr_data = din_a;
        end else if (b_pend) begin
            wr_en   = 1'b1;
            wr_data = b_hold;
        end else if (push_b) begin
            wr_en   = 1'b1;
            wr_data = din_b;
        end
    end

    logic empty;
    sync_fifo #(.DW($bits(u02_roi_cpl_t)), .DEPTH(DEPTH)) u_fifo (
        .clk(clk), .rst_n(rst_n), .wr_en(wr_en), .wr_data(wr_data), .full(full),
        .rd_en(pop), .rd_data(dout), .empty(empty), .count(count)
    );
    assign valid = !empty;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_pend  <= 1'b0;
            ovf_evt <= 1'b0;
        end else begin
            ovf_evt <= wr_en && full && !pop;
            if (push_a && (push_b || b_pend)) begin
                b_pend <= 1'b1;
                if (push_b) b_hold <= din_b;
            end else if (b_pend) begin
                b_pend <= push_b;          // B mới đến đúng lúc đang xả B cũ
                if (push_b) b_hold <= din_b;
            end
        end
    end
endmodule
