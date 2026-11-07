# U02 – ROI Crop Engine (RTL)

Khối U02 trong pipeline camera → U01: nhận khung VGA 640×480 RGB888 từ ISP,
ghi vào 3 frame buffer trong DDR qua AXI4, rồi cắt các ROI (16..128 px,
decimation 1/2/4, cho phép tràn biên với padding) và phát sang U01 qua AXI4-Stream.
Phần mềm điều khiển qua APB (hàng đợi ROI descriptor và hàng completion).

## Cấu trúc thư mục

```
rtl/                  RTL dùng trong u02_top
  u02_pkg.sv          tham số, struct (descriptor, completion, header, row_info, wr_cmd)
  u02_top.sv          top
  u02_scheduler.sv    chọn ROI, deadline, credit buffer U01
  roi_desc_queue.sv   hàng đợi ROI descriptor
  roi_cpl_queue.sv    hàng đợi completion
  frame_buf_mgr.sv    quản lý 3 frame buffer (FREE/WRITING/READY/READING)
  isp_ingress.sv      clk_isp → clk_sys, đóng burst ghi 128 B
  row_cmd_gen.sv      tách ROI thành lệnh đọc theo hàng (lead pad / valid / trail pad)
  page_splitter.sv    chia lệnh đọc thành burst AXI ≤16 beat, không vượt 4 KB
  axi_master_rd.sv    AXI4 read master
  common/             sync_fifo, async_fifo, skid_buffer, rst_sync
tb/                   testbench tự kiểm (in PASS/FAIL)
sim/                  script ModelSim (run_all.do, run_legacy.do)
legacy/da1/           khối và TB thời ĐA1 – KHÔNG khớp u02_top hiện tại
```

## Chạy mô phỏng (ModelSim / Questa)

```tcl
cd D:/test/sim        ;# trong Transcript ModelSim dùng dấu /
do run_all.do
```
hoặc từ cmd: `cd /d D:\test\sim` rồi `vsim -c -do run_all.do`.

## Kết quả mô phỏng (`sim/run_all.do`)

| Testbench | Kiểm tra | Kết quả |
|---|---|---|
| `tb_roi_desc_queue` | push/pop, hàng đầy → overflow | PASS |
| `tb_page_splitter` | burst 1–16 beat, không vượt 4 KB, có stall | PASS |
| `tb_bug1_cpl_queue` | hàng completion không mất entry khi đầy + pop | PASS |
| `tb_bug2_sched_cpl` | completion của ROI đang chạy luôn có chỗ | PASS |
| `tb_bug3_fbm_age` | chọn đúng buffer cũ nhất sau > 65.536 khung | PASS |

Kiểm bằng Verilator 5.020; ba TB lỗi cũng đã chạy trên ModelSim và Icarus 12.

## Sửa lỗi sau review RTL (06/11/2026)

Chi tiết: [`docs/u02_review_2026-11-06.md`](docs/u02_review_2026-11-06.md).
Commit *"Add regression TBs for bugs 1-3"* chứa TB chạy trên RTL gốc (FAIL);
commit kế tiếp sửa RTL (PASS) – xem diff giữa hai commit.

1. **roi_cpl_queue mất completion im lặng** khi ghi đúng chu kỳ hàng đầy và có pop.
   `sync_fifo` thêm tham số `WR_THRU` (mặc định 0, chỉ hàng completion bật).
2. **Completion của ROI đang chạy có thể bị mất** nếu CPL_INVALID làm đầy hàng
   trong lúc ROI chạy. Giao thức mới: scheduler báo `cpl_rsv` (đang nợ 1 completion),
   hàng trả `a_room`; CPL_INVALID không được lấy chỗ đã giữ (bị bỏ kèm `ovf_evt`).
3. **frame_buf_mgr thu hồi nhầm khung mới nhất** sau 65.536 khung (~36 phút @30 fps)
   do bộ đếm age 16 bit tràn vòng. Thay bằng age matrix 3×3 (6 FF), không tràn.

## Trạng thái

- Chưa elaborate được `u02_top`: còn thiếu `axi_master_wr.sv`, `regfile.sv` và
  `axis_crop.sv` bản mới (bản trong `legacy/da1` là giao diện cũ).
- `legacy/da1` (`sim/run_legacy.do`): `tb_addr_gen_2d`, `tb_regfile` PASS;
  `tb_pingpong` báo 904 byte sai trên Verilator – chưa kiểm lại trên ModelSim.
