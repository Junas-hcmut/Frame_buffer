# ------------------------------------------------------------------
# sim/run_all.do - biên dịch và chạy testbench U02
#
# Chạy từ thư mục sim/:
#   ModelSim GUI : cd <repo>/sim   rồi   do run_all.do
#   cmd Windows  : cd /d <repo>\sim  rồi  vsim -c -do run_all.do
# (Trong Transcript của ModelSim dùng dấu / cho đường dẫn: cd D:/test/sim)
# ------------------------------------------------------------------
set RTL ../rtl
set TB  ../tb

if {[file exists work]} { vdel -lib work -all }
vlib work

vlog -sv -work work \
    $RTL/u02_pkg.sv \
    $RTL/common/sync_fifo.sv \
    $RTL/roi_desc_queue.sv \
    $RTL/page_splitter.sv \
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
