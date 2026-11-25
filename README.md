# U02 – Frame buffer + 2D DMA (ISP → DDR → U01)

Khối U02 trong pipeline camera → NPU: nhận khung VGA 640×480 RGB888 từ ISP,
ghi vào 2 frame buffer (ping-pong) trong DDR qua AXI4, rồi cắt các ROI (16..128 px,
decimation 1/2/4, cho phép tràn biên với padding) và phát sang U01 qua AXI4-Stream.
Phần mềm điều khiển qua APB (hàng đợi ROI descriptor và hàng completion).

## Cấu trúc thư mục

```
rtl/                  16 file tổng hợp được, top = u02_top (thứ tự biên dịch: sim/rtl.f)
  u02_pkg.sv          tham số, struct (descriptor, completion, header, row_info, wr_cmd)
  u02_top.sv          top
  regfile.sv          thanh ghi APB – docs/u02_registers.md
  u02_scheduler.sv    phục vụ ROI theo FIFO, bỏ ROI STALE, credit bộ đệm U01
  roi_desc_queue.sv   hàng đợi ROI descriptor
  frame_buf_mgr.sv    2 frame buffer FREE/WRITING/READY/READING, bit newest
  isp_ingress.sv      clk_isp → clk_sys, đóng burst ghi 128 B
  axi_master_wr.sv    AXI4 write master, commit/abort khung theo BRESP
  row_cmd_gen.sv      tách ROI thành lệnh đọc theo hàng (lead pad / valid / trail pad)
  page_splitter.sv    chia lệnh đọc thành burst ≤ 16 beat, không vượt 4 KB
  axi_master_rd.sv    AXI4 read master
  axis_crop.sv        ghép header + pad + pixel (decimation) thành AXI4-Stream, ≤ 8 B/clk
  common/             sync_fifo, async_fifo, skid_buffer, rst_sync
tb/
  tb_u02_top.sv       toàn hệ thống: ISP VGA, DDR AXI4 ngẫu nhiên, U01 nội tuyến, CPU APB, kiểm từng byte
  tb_u02_u01m.sv      toàn hệ thống với mô hình U01 tách rời (u01_model.sv)
  tb_lat.sv           đo latency với DDR lý tưởng (+RLAT, +BLAT, +FASTISP)
  read_roi.py         đọc file ROI .hex do u01_model xuất (+U01_DUMP)
  tb_read_path.sv     nhánh đọc row_cmd_gen → axis_crop so với mô hình vàng độc lập
  tb_axi_master_wr.sv AXI write master với slave ngẫu nhiên, tiêm BRESP lỗi, slave chờ WVALID
  tb_sched_cpl.sv     scheduler + hàng completion: 20 000 ROI, phần mềm lấy completion chậm
  tb_fbm_pingpong.sv  frame_buf_mgr 2 buffer: thu hồi khung cũ hơn, không đọc khung ghi dở
  tb_page_splitter.sv, tb_roi_desc_queue.sv
sim/                  rtl.f, run_all.do (ModelSim/Questa), run_all_verilator.sh, run_legacy.do
docs/                 bản đồ thanh ghi, ghi chú review
legacy/da1/           khối và TB thời ĐA1 – KHÔNG khớp u02_top hiện tại
```

## Giao diện chính

| Cổng | Mô tả |
|---|---|
| APB4 slave | cấu hình, nạp ROI descriptor, đọc completion – [docs/u02_registers.md](docs/u02_registers.md) |
| ISP (clk_isp) | `isp_valid`, `isp_data[23:0]` = {B, G, R}, `isp_sof`, `isp_eol`; không backpressure |
| AXI4 master | 64 bit, 1 ID, burst ≤ 16 beat, ≤ 4 burst chờ mỗi chiều; FB0/1 tại 0x8000_0000 / 0x8010_0000 |
| AXI4-Stream | mỗi ROI: 1 beat header (`tuser = 1`), rồi W×H×3 byte RGB (HWC) liền nhau, `tlast` ở beat cuối |
| `u01_ibuf_release` | U01 trả 1 credit bộ đệm đầu vào |

## Chạy mô phỏng

```tcl
cd D:/test/sim        ;# trong Transcript ModelSim dùng dấu /
do run_all.do
```
Từ cmd: `cd /d D:\test\sim` rồi `vsim -c -do run_all.do`.
Verilator 5.x: `cd sim && ./run_all_verilator.sh`.

## Lịch sử

- **Bản thu gọn ĐA2 (nhánh `lite`)** – để vừa khối lượng nhóm 2 người: bỏ timestamp
  48 bit và deadline; bỏ `roi_cpl_queue` (hàng completion là `sync_fifo`, chỉ
  scheduler ghi; ROI STALE/INVALID đếm ở DROP_CNT/DROP_LAST); về 2 buffer ping-pong
  (`newest` thay age matrix); bỏ `sync_fifo.WR_THRU`. Ba tính năng này để cho DATN.
- **Review 06/11/2026** – [docs/u02_review_2026-11-06.md](docs/u02_review_2026-11-06.md).
  Commit *"Add regression TBs for bugs 1-3"* chứa TB chạy trên RTL gốc (FAIL),
  commit kế tiếp sửa RTL (PASS):
  1. `roi_cpl_queue` mất completion im lặng khi ghi lúc đầy + pop (`sync_fifo.WR_THRU`).
  2. Completion của ROI đang chạy bị mất khi CPL_INVALID làm đầy hàng
     (giao thức `cpl_rsv` / `a_room`: hàng luôn giữ 1 chỗ cho ROI đang chạy).
  3. `frame_buf_mgr` thu hồi nhầm khung mới nhất sau 65.536 khung (age matrix 3×3).
- **Bộ v2 (phiên làm việc 09/11/2026)** – `regfile`, `axis_crop`, `axi_master_wr`,
  `row_cmd_gen` (`has_pad` chính xác), `skid_buffer` và các TB toàn hệ thống.
  `axi_master_wr` phát W không chờ AWREADY (AXI4 A3.3.1). Các sửa lỗi 1–3 ở trên
  được giữ (thay cho cách vá `b_pend` / so sánh age có dấu của v2).

## Latency nhánh đọc (`tb_lat +RLAT=30 +FASTISP`, chu kỳ clk_sys từ dispatch tới tlast)

| ROI | axis_crop v2 (1 pixel/clk) | axis_crop hiện tại (≤ 8 B/clk) | Beat R đọc DDR |
|---|---:|---:|---:|
| 128×128 dec1 | 16 424 | 6 692 | 6 272 |
| 128×128 dec2 | 16 425 | 12 836 | 12 416 |
| 128×112 dec4 | 28 601 | 21 876 | 21 504 |
| 64×64 dec1 | 4 136 | 1 828 | 1 600 |
| 16×16 dec1 | 296 | 196 | 112 |
| 16×16 toàn pad | 262 | 148 | 0 |

Với axis_crop hiện tại, latency xấp xỉ số beat R: nhánh đọc bị giới hạn bởi băng
thông AXI chứ không còn bởi axis_crop. Đổi lại axis_crop dùng bộ dịch 128 bit và
mạch nén byte – cần đối chiếu diện tích khi tổng hợp (mục E).

## Việc còn mở

Xem danh sách trong ghi chú dự án. Chính: frame_id 8 bit (A2), push khi hàng đầy
chưa ghi vào DROP_LAST (A3), khung mới bị bỏ khi abort/SOF sớm trong lúc buffer kia
đang được đọc (A4). A5 (throughput axis_crop) đã xử lý – bảng trên.
