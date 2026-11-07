# ------------------------------------------------------------------
# sim/run_all.do - biên dịch và chạy testbench U02
#
# Chạy từ thư mục sim/:
#   ModelSim GUI : cd <repo>/sim   rồi   do run_all.do
#   cmd Windows  : cd /d <repo>\sim  rồi  vsim -c -do run_all.do
# (Trong Transcript của ModelSim dùng dấu / cho đường dẫn: cd D:/test/sim)
# ------------------------------------------------------------------
# Toàn bộ rtl/ được biên dịch để kiểm cú pháp. u02_top chưa elaborate
# được vì còn thiếu regfile / axis_crop bản mới và axi_master_wr.
set RTL ../rtl
set TB  ../tb

if {[file exists work]} { vdel -lib work -all }
vlib work

vlog -sv -work work \
    $RTL/u02_pkg.sv \
    $RTL/common/sync_fifo.sv \
    $RTL/common/async_fifo.sv \
    $RTL/common/skid_buffer.sv \
    $RTL/common/rst_sync.sv \
    $RTL/roi_desc_queue.sv \
    $RTL/roi_cpl_queue.sv \
    $RTL/frame_buf_mgr.sv \
    $RTL/u02_scheduler.sv \
    $RTL/row_cmd_gen.sv \
    $RTL/page_splitter.sv \
    $RTL/axi_master_rd.sv \
    $RTL/isp_ingress.sv \
    $RTL/u02_top.sv \
    $TB/tb_roi_desc_queue.sv \
    $TB/tb_page_splitter.sv

foreach tb {tb_roi_desc_queue tb_page_splitter} {
    echo "================ $tb ================"
    vsim -onfinish stop work.$tb
    run -all
    quit -sim
}

echo "================ XONG ================"
if {[batch_mode]} { quit -f }
