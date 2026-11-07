`timescale 1ns / 1ps

// =====================================================================
// u02_scheduler (ĐA2): phục vụ ROI theo thứ tự FIFO, mỗi lúc 1 ROI.
//
// Với ROI ở đầu hàng đợi (khi cfg_enable = 1 và hàng completion còn chỗ):
//  1. deadline != 0 và đã quá hạn           -> CPL_DROPPED_LATE
//  2. frame_id không còn READY/READING       -> CPL_STALE
//  3. U01 chưa có credit bộ đệm đầu vào      -> chờ
//  4. còn lại: chiếm frame, khởi động row_cmd_gen, trừ 1 credit
// Kết thúc khi axis_crop báo beat tlast đã được U01 nhận -> ghi completion
// (OK hoặc BUS_ERR), nhả frame.
// Credit: dùng = số header đã phát - số xung ibuf_release; có thể phát khi
// dùng < cfg_ibuf_n.
// Điểm mở rộng cho DATN: thay bước chọn ROI đầu hàng bằng EDF.
// =====================================================================
module u02_scheduler
    import u02_pkg::*;
(
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 cfg_enable,
    input  logic [3:0]           cfg_ibuf_n,
    input  logic [TS_WIDTH-1:0]  ts_now,

    // roi_desc_queue (FWFT)
    input  logic                 q_valid,
    input  u02_roi_desc_t        q_desc,
    output logic                 q_pop,

    // frame_buf_mgr
    output logic [7:0]           lookup_fid,
    input  logic                 lookup_hit,
    input  logic [1:0]           lookup_idx,
    output logic                 acq_req,
    output logic                 rel_evt,
    output logic [1:0]           rel_idx,

    // row_cmd_gen
    output logic                 rg_start,
    output u02_roi_desc_t        rg_desc,
    output logic [31:0]          rg_base,

    // axis_crop
    input  logic                 roi_done,
    input  logic                 roi_err,

    // axi_master_rd
    output logic                 rd_cnt_clear,
    input  logic [19:0]          rd_beat_cnt,

    // U01
    input  logic                 ibuf_release,

    // Completion
    output logic                 cpl_push,
    output u02_roi_cpl_t         cpl,
    input  logic                 cpl_full,

    output logic                 busy,
    output logic                 done_evt
);

    typedef enum logic { S_IDLE, S_RUN } st_t;
    st_t st;

    u02_roi_desc_t       cur;
    logic [1:0]          cur_idx;
    logic [TS_WIDTH-1:0] t_start;
    logic [4:0]          used;           // credit đang dùng

    wire late      = (q_desc.deadline != '0) && ($signed(ts_now - q_desc.deadline) > 0);
    wire credit_ok = (used < {1'b0, cfg_ibuf_n});
    wire can_eval  = (st == S_IDLE) && cfg_enable && q_valid && !cpl_full;

    wire drop_late = can_eval && late;
    wire drop_stale= can_eval && !late && !lookup_hit;
    wire dispatch  = can_eval && !late && lookup_hit && credit_ok;

    assign lookup_fid   = q_desc.frame_id;
    assign acq_req      = dispatch;
    assign q_pop        = drop_late || drop_stale || dispatch;
    assign rg_start     = dispatch;
    assign rg_desc      = q_desc;
    assign rg_base      = fb_base(lookup_idx);
    assign rd_cnt_clear = dispatch;
    assign busy         = (st == S_RUN);

    // Completion (tổ hợp, đẩy trong chu kỳ quyết định)
    wire finish = (st == S_RUN) && roi_done;

    always_comb begin
        cpl      = '0;
        cpl_push = 1'b0;
        if (drop_late || drop_stale) begin
            cpl_push       = 1'b1;
            cpl.roi_id     = q_desc.roi_id;
            cpl.frame_id   = q_desc.frame_id;
            cpl.status     = drop_late ? CPL_DROPPED_LATE : CPL_STALE;
            cpl.t_submit   = q_desc.t_submit;
            cpl.t_start    = ts_now;
            cpl.t_end      = ts_now;
        end else if (finish) begin
            cpl_push          = 1'b1;
            cpl.roi_id        = cur.roi_id;
            cpl.frame_id      = cur.frame_id;
            cpl.status        = roi_err ? CPL_BUS_ERR : CPL_OK;
            cpl.t_submit      = cur.t_submit;
            cpl.t_start       = t_start;
            cpl.t_end         = ts_now;
            cpl.bytes_ddr     = {rd_beat_cnt[16:0], 3'b000};       // beat * 8
            cpl.bytes_stream  = 16'(cur.roi_w * cur.roi_h * PIXEL_BYTES);
        end
    end

    assign rel_evt  = finish;
    assign rel_idx  = cur_idx;
    assign done_evt = cpl_push;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            cur     <= '0;
            cur_idx <= '0;
            t_start <= '0;
            used    <= '0;
        end else begin
            case (st)
                S_IDLE: if (dispatch) begin
                    st      <= S_RUN;
                    cur     <= q_desc;
                    cur_idx <= lookup_idx;
                    t_start <= ts_now;
                end
                S_RUN:  if (roi_done) st <= S_IDLE;
                default: st <= S_IDLE;
            endcase

            // Credit U01
            case ({dispatch, ibuf_release && used != '0})
                2'b10:   used <= used + 1'b1;
                2'b01:   used <= used - 1'b1;
                default: ;
            endcase
        end
    end

endmodule
