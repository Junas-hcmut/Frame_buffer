`timescale 1ns / 1ps

// =====================================================================
// roi_cpl_queue: hàng đợi completion descriptor, phần mềm đọc qua regfile.
//
// Hai nguồn ghi, mỗi chu kỳ ghi tối đa 1 entry:
//   A = scheduler (DROPPED_LATE / STALE / OK / BUS_ERR) - ưu tiên.
//       A KHÔNG BAO GIỜ bị bỏ: scheduler chỉ drop/dispatch khi a_room = 1,
//       và hàng luôn giữ 1 chỗ cho ROI đang chạy (rsv_a).
//   B = descriptor sai từ regfile (CPL_INVALID), không có backpressure
//       (giao dịch APB đã xong). Nếu A và B cùng chu kỳ, B được giữ 1 chu
//       kỳ (b_pend) rồi ghi sau. B bị bỏ (kèm xung ovf_evt) khi ghi nó sẽ
//       chiếm chỗ đã giữ cho A.
//
// Giả thiết (có kiểm tra bằng assertion mô phỏng):
//   - push_b cách nhau >= 2 chu kỳ (mỗi giao dịch APB >= 2 chu kỳ).
//   - Scheduler tuân thủ a_room / rsv_a (xem u02_scheduler).
//   Với 2 giả thiết này b_pend tồn tại tối đa 1 chu kỳ (chứng minh ở
//   ghi chú review), nên b_hold không bao giờ bị ghi đè.
//
// Sửa lỗi so với bản cũ (review 06/11/2026):
//   [Lỗi 1] Bản cũ: sync_fifo chặn ghi khi full kể cả có pop cùng chu kỳ,
//           còn ovf_evt = wr_en && full && !pop -> entry mất mà không báo.
//           Bản mới: sync_fifo WR_THRU = 1, và ovf_evt tính đúng từ quyết
//           định chấp nhận/bỏ của chính module này.
//   [Lỗi 2] Bản cũ không giữ chỗ cho completion của ROI đang chạy.
//           Bản mới: B chỉ được ghi nếu count + 1 - pop + rsv_a <= DEPTH.
// =====================================================================
module roi_cpl_queue
    import u02_pkg::*;
#(
    parameter int DEPTH = ROI_QUEUE_DEPTH
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Nguồn A: scheduler
    input  logic                    push_a,
    input  u02_roi_cpl_t            din_a,
    input  logic                    rsv_a,    // scheduler đang nợ 1 completion (busy || dispatch)
    output logic                    a_room,   // scheduler được phép drop / dispatch chu kỳ này

    // Nguồn B: regfile (CPL_INVALID)
    input  logic                    push_b,
    input  u02_roi_cpl_t            din_b,

    // Phía đọc (regfile)
    input  logic                    pop,
    output u02_roi_cpl_t            dout,
    output logic                    valid,
    output logic                    full,
    output logic [$clog2(DEPTH):0]  count,
    output logic                    ovf_evt   // xung: 1 entry B bị bỏ (hoặc A, nếu vi phạm giao thức)
);

    localparam int CW = $clog2(DEPTH) + 1;     // độ rộng count

    logic         b_pend;
    u02_roi_cpl_t b_hold;
    logic         empty;

    wire pop_eff = pop && !empty;

    // ---------------- Kiểm tra chỗ trống ----------------
    // Sau chu kỳ này: count' = count + ghi - pop_eff. Ràng buộc bất biến
    //   count' + (ROI còn nợ completion) <= DEPTH.
    // So sánh viết dạng cộng để không cần số có dấu:
    //   count + 1 + rsv_a <= DEPTH + pop_eff
    wire [CW:0] b_need  = {1'b0, count} + (CW+1)'(1) + (CW+1)'(rsv_a);
    wire [CW:0] b_limit = (CW+1)'(DEPTH) + (CW+1)'(pop_eff);
    wire        b_fit   = (b_need <= b_limit);

    // A ghi được nếu còn ô trống (tính cả pop cùng chu kỳ nhờ WR_THRU)
    wire        a_fit   = !full || pop_eff;

    // Scheduler (ở IDLE, chưa nợ completion) được drop/dispatch khi:
    //  - còn ít nhất 1 ô (không tính pop: tránh đường tổ hợp APB -> scheduler)
    //  - không có B đang chờ (b_pend ghi ở chu kỳ này, A phải nhường)
    assign a_room = !b_pend && !full;

    // ---------------- Chọn nguồn ghi ----------------
    logic         wr_en, b_try, b_drop, a_drop;
    u02_roi_cpl_t wr_data, b_data;

    always_comb begin
        b_try   = 1'b0;
        b_data  = b_pend ? b_hold : din_b;
        if (!push_a) b_try = b_pend || push_b;

        wr_en   = 1'b0;
        wr_data = din_a;
        a_drop  = 1'b0;
        b_drop  = 1'b0;
        if (push_a) begin
            wr_en   = a_fit;
            wr_data = din_a;
            a_drop  = !a_fit;                  // không xảy ra nếu scheduler đúng giao thức
        end else if (b_try) begin
            wr_en   = b_fit;
            wr_data = b_data;
            b_drop  = !b_fit;
        end
    end

    sync_fifo #(.DW($bits(u02_roi_cpl_t)), .DEPTH(DEPTH), .WR_THRU(1'b1)) u_fifo (
        .clk(clk), .rst_n(rst_n), .wr_en(wr_en), .wr_data(wr_data), .full(full),
        .rd_en(pop), .rd_data(dout), .empty(empty), .count(count)
    );
    assign valid = !empty;

    // ---------------- B chờ ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_pend  <= 1'b0;
            ovf_evt <= 1'b0;
        end else begin
            ovf_evt <= a_drop || b_drop;
            if (push_a) begin
                // A chiếm cổng ghi: B mới (nếu có) phải chờ; B cũ (nếu có) tiếp tục chờ
                if (push_b) begin
                    b_pend <= 1'b1;
                end
            end else begin
                // b_pend (nếu có) vừa được xử lý (ghi hoặc bỏ) ở chu kỳ này.
                // B đến đúng lúc đang xả B cũ -> chờ tới chu kỳ sau.
                b_pend <= b_pend && push_b;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (push_b && (push_a || b_pend)) b_hold <= din_b;
    end

    // ---------------- Kiểm tra giả thiết (chỉ mô phỏng) ----------------
    // synopsys translate_off
    always @(posedge clk) if (rst_n) begin
        if (push_b && b_pend && push_a)
            $error("roi_cpl_queue: B moi ghi de B dang cho (push_b khi b_pend & push_a) -> mat CPL_INVALID");
        if (a_drop)
            $error("roi_cpl_queue: completion cua scheduler bi bo (vi pham a_room/rsv_a)");
    end
    // synopsys translate_on

endmodule
