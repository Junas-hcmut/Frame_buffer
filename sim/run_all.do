# ------------------------------------------------------------------
# sim/run_all.do - biên dịch và chạy toàn bộ testbench U02
#
# Chạy từ thư mục sim/:
#   ModelSim GUI : cd <repo>/sim   rồi   do run_all.do
#   cmd Windows  : cd /d <repo>\sim  rồi  vsim -c -do run_all.do
# (Trong Transcript của ModelSim dùng dấu / cho đường dẫn: cd D:/test/sim)
# Danh sách RTL theo thứ tự biên dịch: rtl.f (dùng chung với Verilator)
# ------------------------------------------------------------------
set TB ../tb

if {[file exists work]} { vdel -lib work -all }
vlib work

vlog -sv -work work -f rtl.f \
    $TB/tb_roi_desc_queue.sv \
    $TB/tb_page_splitter.sv \
    $TB/tb_bug1_cpl_queue.sv \
    $TB/tb_bug2_sched_cpl.sv \
    $TB/tb_bug3_fbm_age.sv \
    $TB/u01_model.sv \
    $TB/tb_u02_top.sv \
    $TB/tb_u02_u01m.sv \
    $TB/tb_lat.sv

# TB cấp khối
foreach tb {tb_roi_desc_queue tb_page_splitter tb_bug1_cpl_queue tb_bug2_sched_cpl tb_bug3_fbm_age} {
    echo "================ $tb ================"
    vsim -onfinish stop work.$tb
    run -all
    quit -sim
}

# TB toàn hệ thống (DDR ngẫu nhiên, U01 nội tuyến, kiểm từng byte)
echo "================ tb_u02_top ================"
vsim -onfinish stop work.tb_u02_top +FRAMES=3
run -all
quit -sim

# TB dùng mô hình U01 tách rời
echo "================ tb_u02_u01m ================"
vsim -onfinish stop work.tb_u02_u01m +FRAMES=2 +U01_QUIET
run -all
quit -sim

# Đo latency (DDR lý tưởng, RLAT = 30)
echo "================ tb_lat ================"
vsim -onfinish stop work.tb_lat +RLAT=30 +FASTISP
run -all
quit -sim

echo "================ XONG ================"
if {[batch_mode]} { quit -f }
