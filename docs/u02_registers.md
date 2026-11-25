# U02 – Bản đồ thanh ghi APB và luồng phần mềm

APB4, dữ liệu 32 bit, cửa sổ 4 KB. `PREADY` luôn = 1 (không có chu kỳ chờ).
Địa chỉ ngoài bản đồ: `PSLVERR = 1`, `PRDATA = 0`. Nguồn: [`rtl/regfile.sv`](../rtl/regfile.sv).

## Điều khiển & nạp ROI

| Địa chỉ | Tên | Kiểu | Bit |
|---|---|---|---|
| 0x000 | CTRL | RW | [0] enable (xử lý ROI) · [1] cap_enable (ghi khung từ ISP) |
| 0x004 | STATUS | RO / W1C | [0] busy · [1] done (W1C) · [2] q_full · [3] q_empty · [4] err_overflow (W1C, push khi hàng đầy) · [5] err_invalid (W1C) · [6] err_stale (W1C, ROI bị bỏ vì khung đã bị ghi đè) · [7] cpl_valid · [12:8] q_count · [20:16] cpl_count |
| 0x008 | IRQ_EN | RW | [1] done · [4] err_overflow · [5] err_invalid · [6] err_stale · [7] cpl_valid (mức) |
| 0x00C | PAD_VAL | RW | [7:0] byte chèn lề (mặc định 0x00) |
| 0x010 | CAP | RO | [7:0] ROI_MIN = 16 · [15:8] ROI_MAX = 128 · [23:16] QUEUE_DEPTH = 16 · [31:24] VERSION = 0x02 |
| 0x014 | IBUF_N | RW | [3:0] số bộ đệm đầu vào của U01 (credit ban đầu) |
| 0x020 | ROI_POS | RW | [11:0] x (có dấu) · [27:16] y (có dấu) |
| 0x024 | ROI_DIM | RW | [7:0] W · [23:16] H (16..128) |
| 0x028 | ROI_ID | RW | [7:0] roi_id · [15:8] frame_id · [17:16] dec_log2 (0/1/2) |
| 0x034 | ROI_PUSH | WO | ghi bit0 = 1: kiểm tra và đẩy descriptor (sai → không vào hàng, err_invalid, DROP_CNT/DROP_LAST) |

Bản thu gọn ĐA2 (nhánh `lite`) không còn deadline và timestamp: các địa chỉ
0x02C–0x03C và 0x044–0x058 trả PSLVERR.

## Completion (đầu hàng completion, sync_fifo 16 × 54 bit)

Chỉ ROI đã dispatch mới có completion. Các trường ổn định tới khi ghi CPL_POP.

| Địa chỉ | Tên | Bit |
|---|---|---|
| 0x040 | CPL_ID | [7:0] roi_id · [15:8] frame_id · [17:16] status · [31] valid |
| 0x05C | CPL_BYTES_DDR | [19:0] số byte đọc trên AXI |
| 0x060 | CPL_BYTES_STREAM | [15:0] W × H × 3 |
| 0x064 | CPL_POP | WO: ghi bit0 = 1 bỏ entry đầu hàng (bỏ qua nếu rỗng) |

`status`: 0 OK · 1 BUS_ERR.

Scheduler chỉ dispatch khi hàng completion chưa đầy và là nguồn ghi duy nhất,
nên completion của ROI đã chạy không bao giờ bị mất.

## Thống kê khung & lỗi

| Địa chỉ | Tên | Bit |
|---|---|---|
| 0x070 | FRAME_STAT | [7:0] latest_fid · [8] latest_valid |
| 0x074 | FRAME_CNT0 | [15:0] committed · [31:16] aborted |
| 0x078 | FRAME_CNT1 | [15:0] recycled · [31:16] denied |
| 0x07C | ERR_CNT | [15:0] isp_ovf · [31:16] frame_err |
| 0x080 | BUS_ERR_CNT | [15:0] số BRESP/RRESP lỗi |
| 0x084 | DROP_CNT | [15:0] số ROI STALE · [31:16] số ROI INVALID |
| 0x088 | DROP_LAST | ROI bị bỏ gần nhất: [7:0] roi_id · [15:8] frame_id · [16] lý do (0 STALE, 1 INVALID) · [31] valid |

Các bộ đếm 16 bit, bão hòa (ERR_CNT, BUS_ERR_CNT, DROP_CNT) hoặc quay vòng (FRAME_CNT).

## Luồng phần mềm

```
khởi tạo:  PAD_VAL, IBUF_N, IRQ_EN;  CTRL = 0x3
vòng lặp:
  FRAME_STAT -> latest_valid & latest_fid mới?
      ROI_POS, ROI_DIM, ROI_ID(frame_id = latest_fid), ROI_PUSH
  while CPL_ID.valid:
      đọc CPL_* ; ghi CPL_POP = 1
  thỉnh thoảng: DROP_CNT / DROP_LAST để biết ROI bị bỏ (STALE / INVALID)
```

`tb/tb_u02_top.sv` và `tb/tb_u02_u01m.sv` làm đúng luồng này qua APB.
