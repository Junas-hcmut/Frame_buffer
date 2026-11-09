# U02 – Bản đồ thanh ghi APB và luồng phần mềm

APB4, dữ liệu 32 bit, cửa sổ 4 KB. `PREADY` luôn = 1 (không có chu kỳ chờ).
Địa chỉ ngoài bản đồ: `PSLVERR = 1`, `PRDATA = 0`. Nguồn: [`rtl/regfile.sv`](../rtl/regfile.sv).

## Điều khiển & nạp ROI

| Địa chỉ | Tên | Kiểu | Bit |
|---|---|---|---|
| 0x000 | CTRL | RW | [0] enable (xử lý ROI) · [1] cap_enable (ghi khung từ ISP) |
| 0x004 | STATUS | RO / W1C | [0] busy · [1] done (W1C) · [2] q_full · [3] q_empty · [4] err_overflow (W1C, push khi hàng đầy) · [5] err_invalid (W1C) · [6] err_cpl_ovf (W1C, hàng completion bỏ entry) · [7] cpl_valid · [12:8] q_count · [20:16] cpl_count |
| 0x008 | IRQ_EN | RW | [1] done · [4] err_overflow · [5] err_invalid · [6] err_cpl_ovf · [7] cpl_valid (mức) |
| 0x00C | PAD_VAL | RW | [7:0] byte chèn lề (mặc định 0x00) |
| 0x010 | CAP | RO | [7:0] ROI_MIN = 16 · [15:8] ROI_MAX = 128 · [23:16] QUEUE_DEPTH = 16 · [31:24] VERSION = 0x01 |
| 0x014 | IBUF_N | RW | [3:0] số bộ đệm đầu vào của U01 (credit ban đầu) |
| 0x020 | ROI_POS | RW | [11:0] x (có dấu) · [27:16] y (có dấu) |
| 0x024 | ROI_DIM | RW | [7:0] W · [23:16] H (16..128) |
| 0x028 | ROI_ID | RW | [7:0] roi_id · [15:8] frame_id · [17:16] dec_log2 (0/1/2) |
| 0x02C | ROI_DL_LO | RW | deadline[31:0] (deadline = 0: không có hạn) |
| 0x030 | ROI_DL_HI | RW | [15:0] deadline[47:32] |
| 0x034 | ROI_PUSH | WO | ghi bit0 = 1: kiểm tra và đẩy descriptor (sai → completion CPL_INVALID) |
| 0x038 | TS_LO | RO | timestamp[31:0]; đọc thanh ghi này chốt TS_HI |
| 0x03C | TS_HI | RO | [15:0] timestamp[47:32] đã chốt |

> **Lưu ý deadline (việc A1):** deadline là thời điểm tuyệt đối 48 bit. Driver phải
> ghi cả ROI_DL_HI; nếu để ROI_DL_HI = 0 thì sau 2³² chu kỳ (~28,6 s @150 MHz)
> mọi ROI có deadline đều bị DROPPED_LATE. Cách đúng: đọc TS_LO rồi TS_HI, cộng
> khoảng hạn, ghi DL_LO và DL_HI.

## Completion (đầu hàng `roi_cpl_queue`)

Các trường ổn định tới khi ghi CPL_POP.

| Địa chỉ | Tên | Bit |
|---|---|---|
| 0x040 | CPL_ID | [7:0] roi_id · [15:8] frame_id · [18:16] status · [31] valid |
| 0x044 / 0x048 | CPL_TSUB_LO / HI | t_submit (lúc ghi ROI_PUSH) |
| 0x04C / 0x050 | CPL_TSTA_LO / HI | t_start (lúc scheduler dispatch) |
| 0x054 / 0x058 | CPL_TEND_LO / HI | t_end (lúc beat tlast được U01 nhận) |
| 0x05C | CPL_BYTES_DDR | [19:0] số byte đọc trên AXI |
| 0x060 | CPL_BYTES_STREAM | [15:0] W × H × 3 |
| 0x064 | CPL_POP | WO: ghi bit0 = 1 bỏ entry đầu hàng (bỏ qua nếu rỗng) |

`status`: 0 OK · 1 DROPPED_LATE · 2 STALE · 3 INVALID · 4 BUS_ERR.

Hàng completion luôn giữ 1 chỗ cho ROI đang chạy (sửa lỗi 2): completion của
scheduler không bao giờ bị bỏ; khi hàng hết chỗ, chỉ CPL_INVALID mới bị bỏ
(STATUS.err_cpl_ovf = 1).

## Thống kê khung & lỗi

| Địa chỉ | Tên | Bit |
|---|---|---|
| 0x070 | FRAME_STAT | [7:0] latest_fid · [8] latest_valid |
| 0x074 | FRAME_CNT0 | [15:0] committed · [31:16] aborted |
| 0x078 | FRAME_CNT1 | [15:0] recycled · [31:16] denied |
| 0x07C | ERR_CNT | [15:0] isp_ovf · [31:16] frame_err |
| 0x080 | BUS_ERR_CNT | [15:0] số BRESP/RRESP lỗi |

Các bộ đếm 16 bit, bão hòa (ERR_CNT, BUS_ERR_CNT) hoặc quay vòng (FRAME_CNT).

## Luồng phần mềm

```
khởi tạo:  PAD_VAL, IBUF_N, IRQ_EN;  CTRL = 0x3
vòng lặp:
  FRAME_STAT -> latest_valid & latest_fid mới?
      ROI_POS, ROI_DIM, ROI_ID(frame_id = latest_fid), ROI_DL_LO/HI, ROI_PUSH
  while CPL_ID.valid:
      đọc CPL_* ; latency = t_end − t_submit ; ghi CPL_POP = 1
```

`tb/tb_u02_top.sv` và `tb/tb_u02_u01m.sv` làm đúng luồng này qua APB.
