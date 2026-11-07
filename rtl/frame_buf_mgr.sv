`timescale 1ns / 1ps

// =====================================================================
// frame_buf_mgr: quản lý NUM_FB (=3) frame buffer trong DDR.
//
// Trạng thái mỗi buffer: FREE -> WRITING -> READY <-> READING -> ...
//  - Writer xin buffer (alloc): ưu tiên FREE; nếu không có, thu hồi
//    buffer READY cũ nhất không bị đọc ("khung mới nhất thắng").
//  - Khung chỉ vào READY khi BRESP của burst cuối đã về (commit).
//  - Reader (scheduler) tra theo frame_id: chỉ READY/READING mới hợp lệ;
//    không tìm thấy -> ROI bị STALE.
//  - Ưu tiên reader: buffer đang được acquire trong chu kỳ này không bị
//    alloc thu hồi.
// =====================================================================
module frame_buf_mgr
    import u02_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // Writer (isp_ingress)
    input  logic        alloc_req,
    output logic        alloc_ok,
    output logic [1:0]  alloc_idx,
    output logic [7:0]  alloc_fid,

    // Kết thúc ghi (axi_master_wr)
    input  logic        commit_evt,
    input  logic [1:0]  commit_idx,
    input  logic        abort_evt,
    input  logic [1:0]  abort_idx,

    // Reader (scheduler)
    input  logic [7:0]  lookup_fid,     // tra liên tục (tổ hợp)
    output logic        lookup_hit,
    output logic [1:0]  lookup_idx,
    input  logic        acq_req,        // chiếm buffer vừa tra được
    input  logic        rel_evt,
    input  logic [1:0]  rel_idx,

    // Trạng thái cho regfile
    output logic [7:0]  latest_fid,
    output logic        latest_valid,
    output logic [15:0] cnt_committed,
    output logic [15:0] cnt_aborted,
    output logic [15:0] cnt_recycled,
    output logic [15:0] cnt_denied
);

    fb_state_t   st  [NUM_FB];
    logic [7:0]  fid [NUM_FB];
    logic [15:0] age [NUM_FB];     // thứ tự commit, dùng chọn READY cũ nhất
    logic [1:0]  rc  [NUM_FB];     // số reader đang giữ

    logic [7:0]  fid_cnt;
    logic [15:0] age_cnt;

    // ---------------- Tra cứu cho reader ----------------
    always_comb begin
        lookup_hit = 1'b0;
        lookup_idx = '0;
        for (int i = NUM_FB - 1; i >= 0; i--) begin
            if ((st[i] == FB_READY || st[i] == FB_READING) && fid[i] == lookup_fid) begin
                lookup_hit = 1'b1;
                lookup_idx = 2'(i);
            end
        end
    end

    wire acq_fire = acq_req && lookup_hit;

    // ---------------- Chọn buffer cho writer ----------------
    logic        found_free, found_ready;
    logic [1:0]  free_idx, ready_idx;
    logic [15:0] best_age;

    always_comb begin
        found_free  = 1'b0;  free_idx  = '0;
        found_ready = 1'b0;  ready_idx = '0;
        best_age    = '1;
        for (int i = NUM_FB - 1; i >= 0; i--) begin
            if (st[i] == FB_FREE) begin
                found_free = 1'b1;
                free_idx   = 2'(i);
            end
        end
        for (int i = 0; i < NUM_FB; i++) begin
            if (st[i] == FB_READY && rc[i] == 2'd0 && !(acq_fire && lookup_idx == 2'(i)) &&
                (!found_ready || age[i] < best_age)) begin
                found_ready = 1'b1;
                ready_idx   = 2'(i);
                best_age    = age[i];
            end
        end
    end

    assign alloc_ok  = alloc_req && (found_free || found_ready);
    assign alloc_idx = found_free ? free_idx : ready_idx;
    assign alloc_fid = fid_cnt;

    // ---------------- Cập nhật trạng thái ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_FB; i++) begin
                st[i]  <= FB_FREE;
                fid[i] <= '0;
                age[i] <= '0;
                rc[i]  <= '0;
            end
            fid_cnt       <= '0;
            age_cnt       <= '0;
            latest_fid    <= '0;
            latest_valid  <= 1'b0;
            cnt_committed <= '0;
            cnt_aborted   <= '0;
            cnt_recycled  <= '0;
            cnt_denied    <= '0;
        end else begin
            // Writer: cấp buffer
            if (alloc_req) begin
                if (alloc_ok) begin
                    st[alloc_idx]  <= FB_WRITING;
                    fid[alloc_idx] <= fid_cnt;
                    fid_cnt        <= fid_cnt + 1'b1;
                    if (!found_free) begin
                        cnt_recycled <= cnt_recycled + 1'b1;
                        if (latest_valid && fid[alloc_idx] == latest_fid)
                            latest_valid <= 1'b0;
                    end
                end else begin
                    cnt_denied <= cnt_denied + 1'b1;
                end
            end

            // Khung ghi xong
            if (commit_evt) begin
                st[commit_idx]  <= FB_READY;
                age[commit_idx] <= age_cnt;
                age_cnt         <= age_cnt + 1'b1;
                latest_fid      <= fid[commit_idx];
                latest_valid    <= 1'b1;
                cnt_committed   <= cnt_committed + 1'b1;
            end
            if (abort_evt) begin
                st[abort_idx] <= FB_FREE;
                cnt_aborted   <= cnt_aborted + 1'b1;
            end

            // Reader
            if (acq_fire) begin
                st[lookup_idx] <= FB_READING;
                rc[lookup_idx] <= rc[lookup_idx] + 1'b1;
            end
            if (rel_evt && rc[rel_idx] != 2'd0) begin
                // acquire và release cùng buffer trong 1 chu kỳ: giữ READING
                if (!(acq_fire && lookup_idx == rel_idx)) begin
                    rc[rel_idx] <= rc[rel_idx] - 1'b1;
                    if (rc[rel_idx] == 2'd1) st[rel_idx] <= FB_READY;
                end else begin
                    rc[rel_idx] <= rc[rel_idx];
                end
            end
        end
    end

endmodule
