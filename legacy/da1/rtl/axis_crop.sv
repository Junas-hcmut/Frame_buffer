`timescale 1ns / 1ps

import u02_pkg::*;

module axis_crop #(
    parameter DATA_WIDTH = 64,
    parameter KEEP_WIDTH = (DATA_WIDTH / 8),
    parameter USER_WIDTH = 1
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Giao tiếp với addr_gen_2d
    input  logic                          gen_valid,
    output logic                          gen_ready,
    input  logic                          gen_pad,
    input  logic                          gen_eof,
    input  logic                          gen_has_pad,
    input  logic [11:0]                   gen_w,
    input  logic [11:0]                   gen_h,
    input  logic [1:0]                    gen_dec_log2,
    input  logic [7:0]                    gen_roi_id,
    input  logic [7:0]                    gen_frame_id,

    // Kênh nhận pixel (từ DDR Master Reader / Buffer)
    input  logic                          pixel_rvalid,
    output logic                          pixel_rready,
    input  logic [23:0]                   pixel_rdata, // RGB888: 3 Bytes

    // Giao tiếp AXI4-Stream Master xuất sang U01 NPU
    output logic                          m_axis_tvalid,
    input  logic                          m_axis_tready,
    output logic [DATA_WIDTH-1:0]         m_axis_tdata,
    output logic [KEEP_WIDTH-1:0]         m_axis_tkeep,
    output logic                          m_axis_tlast,
    output logic [USER_WIDTH-1:0]         m_axis_tuser
);

    // -------------------------------------------------------------
    // 1. Định nghĩa FSM trạng thái
    // -------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE   = 2'b00,
        ST_HEADER = 2'b01, // Phát 1 beat Header 64-bit (Metadata)
        ST_PIXEL  = 2'b10  // Gom byte và phát luồng payload RGB888
    } state_t;

    state_t state_reg, state_next;

    // Thanh ghi chốt cấu hình ROI cho Header Beat
    logic [11:0] roi_w_r, roi_h_r;
    logic [7:0]  roi_id_r, frame_id_r;
    logic [1:0]  dec_log2_r;
    logic        has_pad_r;

    // Bộ đếm byte còn lại của ROI (W * H * 3, tối đa 128*128*3 = 49152 B)
    logic [16:0] rem_roi_bytes;

    // Bộ đệm dịch byte (Chứa tối đa 16 bytes = 128 bits)
    logic [127:0] shift_reg_q, shift_reg_d;
    logic [4:0]   byte_cnt_q, byte_cnt_d;

    // Tín hiệu kết nối nội bộ sang Skid Buffer
    logic                  pre_tvalid;
    logic                  pre_tready;
    logic [DATA_WIDTH-1:0] pre_tdata;
    logic [KEEP_WIDTH-1:0] pre_tkeep;
    logic                  pre_tlast;
    logic [USER_WIDTH-1:0] pre_tuser;

    // -------------------------------------------------------------
    // 2. Hàm tiện ích tính toán tkeep cho beat cuối cùng
    // -------------------------------------------------------------
    function automatic logic [7:0] calc_tkeep(input logic [3:0] rem_b);
        case (rem_b[2:0])
            3'd1:    return 8'b0000_0001; // 8'h01
            3'd2:    return 8'b0000_0011; // 8'h03
            3'd3:    return 8'b0000_0111; // 8'h07
            3'd4:    return 8'b0000_1111; // 8'h0F
            3'd5:    return 8'b0001_1111; // 8'h1F
            3'd6:    return 8'b0011_1111; // 8'h3F
            3'd7:    return 8'b0111_1111; // 8'h7F
            default: return 8'b1111_1111; // 8'hFF (rem_b == 8 hoặc chia hết)
        endcase
    endfunction

    // -------------------------------------------------------------
    // 3. Logic đóng gói Header Beat 64-bit
    // -------------------------------------------------------------
    u02_header_t hdr;
    always_comb begin
        hdr.magic    = HEADER_MAGIC; // 8'hA5
        hdr.reserved = 5'b0;
        hdr.dec_log2 = dec_log2_r;
        hdr.has_pad  = has_pad_r;
        hdr.frame_id = frame_id_r;
        hdr.roi_id   = roi_id_r;
        hdr.h        = {4'b0, roi_h_r};
        hdr.w        = {4'b0, roi_w_r};
    end

    // -------------------------------------------------------------
    // 4. Điều kiện xác định Beat Payload hợp lệ (AMBA Handshake Rule)
    // -------------------------------------------------------------
    // Beat cuối: Khi số byte còn lại <= 8 và bộ đệm đã nạp đủ số byte đó
    wire is_final_beat  = (rem_roi_bytes > 0) && (rem_roi_bytes <= 17'd8) && 
                          (byte_cnt_q >= rem_roi_bytes[4:0]);
    // Beat thường: Khi gom đủ 8 bytes
    wire is_normal_beat = (rem_roi_bytes > 17'd8) && (byte_cnt_q >= 5'd8);

    // Xác định số byte xả ra downstream ở chu kỳ này
    logic [3:0] pop_bytes;
    assign pop_bytes = is_final_beat ? rem_roi_bytes[3:0] : 4'd8;

    // Các sự kiện bắt tay (Fire)
    wire pop_fire       = pre_tvalid && pre_tready;
    wire pop_pixel_beat = pop_fire && (state_reg == ST_PIXEL);

    // Chuẩn bị pixel nạp vào (Chèn pad đen 0x00 nếu ngoài biên)
    wire [23:0] in_pixel_rgb = gen_pad ? {3{PAD_VAL_DEFAULT}} : pixel_rdata;
    
    // Upstream sẵn sàng nạp: Chỉ khi ở ST_PIXEL và bộ đệm còn chỗ chứa (<= 13 bytes)
    wire in_ready   = (state_reg == ST_PIXEL) && (byte_cnt_q <= 5'd13);
    assign gen_ready    = in_ready && (gen_pad || pixel_rvalid);
    assign pixel_rready = in_ready && gen_valid && !gen_pad;

    wire push_fire  = gen_valid && gen_ready;

    // -------------------------------------------------------------
    // 5. Bộ gom byte hai bước (Pop -> Push) sạch sẽ, không xung đột
    // -------------------------------------------------------------
    always_comb begin
        shift_reg_d = shift_reg_q;
        byte_cnt_d  = byte_cnt_q;

        // Bước 1: Xả 8 bytes (hoặc số byte cuối) ra downstream nếu có handshake
        if (pop_pixel_beat) begin
            shift_reg_d = shift_reg_d >> (pop_bytes * 8);
            byte_cnt_d  = byte_cnt_d - pop_bytes;
        end

        // Bước 2: Nạp 3 bytes pixel mới vào vị trí trống tiếp theo
        if (push_fire) begin
            shift_reg_d[byte_cnt_d * 8 +: 24] = in_pixel_rgb;
            byte_cnt_d = byte_cnt_d + 5'd3;
        end
    end

    // Cập nhật thanh ghi bộ đệm
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg_q <= '0;
            byte_cnt_q  <= '0;
        end else begin
            shift_reg_q <= shift_reg_d;
            byte_cnt_q  <= byte_cnt_d;
        end
    end

    // -------------------------------------------------------------
    // 6. FSM Điều phối luồng và quản lý chu kỳ sống của ROI
    // -------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg     <= ST_IDLE;
            roi_w_r       <= '0;
            roi_h_r       <= '0;
            roi_id_r      <= '0;
            frame_id_r    <= '0;
            dec_log2_r    <= '0;
            has_pad_r     <= 1'b0;
            rem_roi_bytes <= '0;
        end else begin
            state_reg <= state_next;

            case (state_reg)
                ST_IDLE: begin
                    if (gen_valid) begin
                        roi_w_r       <= gen_w;
                        roi_h_r       <= gen_h;
                        roi_id_r      <= gen_roi_id;
                        frame_id_r    <= gen_frame_id;
                        dec_log2_r    <= gen_dec_log2;
                        has_pad_r     <= gen_has_pad;
                        // rem_roi_bytes = W * H * 3
                        rem_roi_bytes <= (gen_w * gen_h) + ((gen_w * gen_h) << 1);
                    end
                end

                ST_PIXEL: begin
                    if (pop_pixel_beat) begin
                        rem_roi_bytes <= rem_roi_bytes - pop_bytes;
                    end
                end

                default: ;
            endcase
        end
    end

    // Chuyển trạng thái FSM
    always_comb begin
        state_next = state_reg;
        case (state_reg)
            ST_IDLE: begin
                if (gen_valid) state_next = ST_HEADER;
            end

            ST_HEADER: begin
                // Chờ skid buffer nhận xong Header Beat
                if (pop_fire) state_next = ST_PIXEL;
            end

            ST_PIXEL: begin
                // Hoàn tất khi beat cuối cùng được skid buffer tiếp nhận
                if (pop_pixel_beat && is_final_beat) state_next = ST_IDLE;
            end

            default: state_next = ST_IDLE;
        endcase
    end

    // -------------------------------------------------------------
    // 7. Sinh tín hiệu AXI4-Stream ngõ ra (TVALID KHÔNG PHỤ THUỘC TREADY)
    // -------------------------------------------------------------
    always_comb begin
        pre_tvalid = 1'b0;
        pre_tdata  = '0;
        pre_tkeep  = 8'h00;
        pre_tlast  = 1'b0;
        pre_tuser  = 1'b0;

        case (state_reg)
            ST_HEADER: begin
                // Phát Header: Valid = 1 vô điều kiện, user[0] = 1 duy nhất ở beat này
                pre_tvalid = 1'b1;
                pre_tdata  = hdr;
                pre_tkeep  = 8'hFF;
                pre_tlast  = 1'b0;
                pre_tuser  = 1'b1;
            end

            ST_PIXEL: begin
                // Phát Payload: Valid = 1 ngay khi có đủ 8 bytes hoặc đủ số byte cuối
                if (is_normal_beat || is_final_beat) begin
                    pre_tvalid = 1'b1;
                    pre_tdata  = shift_reg_q[63:0];
                    pre_tuser  = 1'b0;
                    pre_tlast  = is_final_beat;
                    pre_tkeep  = is_final_beat ? calc_tkeep(rem_roi_bytes[3:0]) : 8'hFF;
                end
            end

            default: ;
        endcase
    end

    // -------------------------------------------------------------
    // 8. Đấu nối Skid Buffer 64-bit (Đầy đủ cổng tkeep, ngắt trễ tổ hợp)
    // -------------------------------------------------------------
    // -------------------------------------------------------------
    // 8. Đấu nối Skid Buffer 64-bit
    // -------------------------------------------------------------
    skid_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .USER_WIDTH (USER_WIDTH)
    ) u_skid (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axis_tvalid (pre_tvalid),
        .s_axis_tready (pre_tready),
        .s_axis_tdata  (pre_tdata),
        .s_axis_tkeep  (pre_tkeep),
        .s_axis_tlast  (pre_tlast),
        .s_axis_tuser  (pre_tuser),
        .m_axis_tvalid (m_axis_tvalid),
        .m_axis_tready (m_axis_tready),
        .m_axis_tdata  (m_axis_tdata),
        .m_axis_tkeep  (m_axis_tkeep),
        .m_axis_tlast  (m_axis_tlast),
        .m_axis_tuser  (m_axis_tuser)
    );
endmodule