`timescale 1ns / 1ps

// =====================================================================
// u02_scheduler (ĐA2, bản thu gọn): phục vụ ROI theo thứ tự FIFO, mỗi lúc 1 ROI.
//
// Với ROI ở đầu hàng đợi (khi cfg_enable = 1):
//  1. frame_id không còn READY/READING        -> bỏ ROI (STALE): xung stale_evt
//  2. hàng completion đầy hoặc U01 hết credit -> chờ
//  3. còn lại: chiếm frame, khởi động row_cmd_gen, trừ 1 credit
// Kết thúc khi axis_crop báo beat tlast đã được U01 nhận -> ghi completion
// (OK hoặc BUS_ERR), nhả frame.
//
// Không mất completion: chỉ dispatch khi hàng completion chưa đầy. Từ lúc
// dispatch tới lúc kết thúc, scheduler là nguồn ghi duy nhất của hàng
// completion (ghi đúng 1 entry lúc kết thúc), phần mềm chỉ có thể lấy bớt,
// nên lúc kết thúc hàng luôn còn chỗ.
//
// Credit: dùng = số header đã phát - số xung ibuf_release; có thể phát khi
// dùng < cfg_ibuf_n.
// Điểm mở rộng cho DATN: deadline, timestamp và chọn ROI theo EDF.
// =====================================================================
module u02_scheduler
    import u02_pkg::*;
(
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 cfg_enable,
    input  logic [3:0]           cfg_ibuf_n,

    // roi_desc_queue (FWFT)
    input  logic                 q_valid,
    input  u02_roi_desc_t        q_desc,
    output logic                 q_pop,

    // frame_buf_mgr
    output logic [7:0]           lookup_fid,
    input  logic                 lookup_hit,
    input  fb_idx_t              lookup_idx,
    output logic                 acq_req,
    output logic                 rel_evt,
    output fb_idx_t              rel_idx,

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

    // Hàng completion
    output logic                 cpl_push,
    output u02_roi_cpl_t         cpl,
    input  logic                 cpl_full,

    // ROI bị bỏ vì frame đã bị thu hồi
    output logic                 stale_evt,
    output logic [7:0]           stale_roi_id,
    output logic [7:0]           stale_frame_id,

    output logic                 busy,
    output logic                 done_evt
);

    typedef enum logic { S_IDLE, S_RUN } st_t;
    st_t st;

    u02_roi_desc_t       cur;
    fb_idx_t             cur_idx;
    logic [4:0]          used;           // credit đang dùng

    wire credit_ok  = (used < {1'b0, cfg_ibuf_n});
    wire head_ok    = (st == S_IDLE) && cfg_enable && q_valid;

    wire drop_stale = head_ok && !lookup_hit;
    wire dispatch   = head_ok && lookup_hit && credit_ok && !cpl_full;

    assign lookup_fid     = q_desc.frame_id;
    assign acq_req        = dispatch;
    assign q_pop          = drop_stale || dispatch;
    assign rg_start       = dispatch;
    assign rg_desc        = q_desc;
    assign rg_base        = fb_base(lookup_idx);
    assign rd_cnt_clear   = dispatch;
    assign busy           = (st == S_RUN);

    assign stale_evt      = drop_stale;
    assign stale_roi_id   = q_desc.roi_id;
    assign stale_frame_id = q_desc.frame_id;

    // Completion (tổ hợp, đẩy trong chu kỳ kết thúc)
    wire finish = (st == S_RUN) && roi_done;

    always_comb begin
        cpl              = '0;
        cpl.roi_id       = cur.roi_id;
        cpl.frame_id     = cur.frame_id;
        cpl.status       = roi_err ? CPL_BUS_ERR : CPL_OK;
        cpl.bytes_ddr    = {rd_beat_cnt[16:0], 3'b000};       // beat * 8
        cpl.bytes_stream = 16'(cur.roi_w * cur.roi_h * PIXEL_BYTES);
    end
    assign cpl_push = finish;

    assign rel_evt  = finish;
    assign rel_idx  = cur_idx;
    assign done_evt = finish;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            cur     <= '0;
            cur_idx <= '0;
            used    <= '0;
        end else begin
            case (st)
                S_IDLE: if (dispatch) begin
                    st      <= S_RUN;
                    cur     <= q_desc;
                    cur_idx <= lookup_idx;
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

    // synopsys translate_off
    always @(posedge clk) if (rst_n && cpl_push && cpl_full)
        $error("u02_scheduler: hang completion day luc ket thuc ROI -> mat completion");
    // synopsys translate_on

endmodule
