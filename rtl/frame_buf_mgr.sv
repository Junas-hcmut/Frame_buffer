`timescale 1ns / 1ps

// =====================================================================
// frame_buf_mgr: quản lý 2 frame buffer (ping-pong) trong DDR.
//
// Trạng thái mỗi buffer: FREE -> WRITING -> READY <-> READING
//  - Writer xin buffer (alloc) ở đầu khung: ưu tiên FREE; nếu không có,
//    thu hồi buffer READY không bị đọc; nếu cả hai READY thì thu hồi
//    buffer cũ hơn ("khung mới nhất thắng"); nếu buffer còn lại đang
//    READING (hoặc đang WRITING) thì từ chối, isp_ingress bỏ khung đó.
//  - Khung chỉ vào READY khi BRESP của burst cuối đã về (commit).
//  - Reader (scheduler) tra theo frame_id: chỉ READY/READING mới hợp lệ;
//    không tìm thấy -> ROI bị STALE.
//  - Ưu tiên reader: buffer đang được acquire trong chu kỳ này không bị
//    alloc thu hồi.
//
// Thứ tự commit: với 2 buffer chỉ cần 1 bit 'newest' = chỉ số buffer
// commit gần nhất. Hai buffer cùng READY thì buffer còn lại (!newest)
// là buffer cũ hơn. Không có bộ đếm tuổi nên không có lỗi tràn vòng.
//
// Chỉ có một reader (scheduler phục vụ 1 ROI mỗi lúc và nhả buffer ở chu
// kỳ kết thúc, không dispatch cùng chu kỳ đó), nên không cần đếm số reader:
// trạng thái READING đủ để biết buffer đang bị giữ.
// =====================================================================
module frame_buf_mgr
    import u02_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // Writer (isp_ingress)
    input  logic        alloc_req,
    output logic        alloc_ok,
    output fb_idx_t     alloc_idx,
    output logic [7:0]  alloc_fid,

    // Kết thúc ghi (axi_master_wr)
    input  logic        commit_evt,
    input  fb_idx_t     commit_idx,
    input  logic        abort_evt,
    input  fb_idx_t     abort_idx,

    // Reader (scheduler)
    input  logic [7:0]  lookup_fid,     // tra liên tục (tổ hợp)
    output logic        lookup_hit,
    output fb_idx_t     lookup_idx,
    input  logic        acq_req,        // chiếm buffer vừa tra được
    input  logic        rel_evt,
    input  fb_idx_t     rel_idx,

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
    fb_idx_t     newest;               // buffer commit gần nhất
    logic [7:0]  fid_cnt;

    // ---------------- Tra cứu cho reader ----------------
    logic [NUM_FB-1:0] hit;
    always_comb begin
        for (int i = 0; i < NUM_FB; i++)
            hit[i] = (st[i] == FB_READY || st[i] == FB_READING) && fid[i] == lookup_fid;
        lookup_hit = |hit;
        lookup_idx = hit[0] ? 1'b0 : 1'b1;
    end

    wire acq_fire = acq_req && lookup_hit;

    // ---------------- Chọn buffer cho writer ----------------
    logic [NUM_FB-1:0] free, elig;     // elig: READY và không bị acquire chu kỳ này
    always_comb begin
        for (int i = 0; i < NUM_FB; i++) begin
            free[i] = (st[i] == FB_FREE);
            elig[i] = (st[i] == FB_READY) && !(acq_fire && lookup_idx == fb_idx_t'(i));
        end
    end

    wire     found_free  = |free;
    wire     found_ready = |elig;
    fb_idx_t free_idx, ready_idx;
    assign free_idx  = free[0] ? 1'b0 : 1'b1;
    assign ready_idx = (&elig) ? ~newest :         // cả hai READY: lấy buffer cũ hơn
                       (elig[0] ? 1'b0 : 1'b1);

    assign alloc_ok  = alloc_req && (found_free || found_ready);
    assign alloc_idx = found_free ? free_idx : ready_idx;
    assign alloc_fid = fid_cnt;

    // ---------------- Cập nhật trạng thái ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_FB; i++) begin
                st[i]  <= FB_FREE;
                fid[i] <= '0;
            end
            newest        <= '0;
            fid_cnt       <= '0;
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
                st[commit_idx] <= FB_READY;
                newest         <= commit_idx;
                latest_fid     <= fid[commit_idx];
                latest_valid   <= 1'b1;
                cnt_committed  <= cnt_committed + 1'b1;
            end
            if (abort_evt) begin
                st[abort_idx] <= FB_FREE;
                cnt_aborted   <= cnt_aborted + 1'b1;
            end

            // Reader
            if (acq_fire)
                st[lookup_idx] <= FB_READING;
            if (rel_evt && st[rel_idx] == FB_READING)
                st[rel_idx] <= FB_READY;
        end
    end

    // synopsys translate_off
    always @(posedge clk) if (rst_n) begin
        if (acq_fire && rel_evt)
            $error("frame_buf_mgr: acquire va release cung chu ky (scheduler chi co 1 ROI)");
        if (acq_fire && st[lookup_idx] == FB_READING)
            $error("frame_buf_mgr: acquire buffer dang READING");
    end
    // synopsys translate_on

endmodule
