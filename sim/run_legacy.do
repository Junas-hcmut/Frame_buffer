# ------------------------------------------------------------------
# sim/run_legacy.do - testbench các khối thời ĐA1 (legacy/da1)
# Các khối này KHÔNG khớp u02_top hiện tại; giữ lại để tham khảo.
# Chạy từ thư mục sim/:  do run_legacy.do
# ------------------------------------------------------------------
set RTL ../rtl
set LEG ../legacy/da1

if {[file exists work_legacy]} { vdel -lib work_legacy -all }
vlib work_legacy

vlog -sv -work work_legacy \
    $RTL/u02_pkg.sv \
    $RTL/common/sync_fifo.sv \
    $RTL/roi_desc_queue.sv \
    $LEG/rtl/addr_gen_2d.sv \
    $LEG/rtl/pingpong.sv \
    $LEG/rtl/regfile.sv \
    $LEG/tb/tb_addr_gen_2d.sv \
    $LEG/tb/tb_pingpong.sv \
    $LEG/tb/tb_regfile.sv

foreach tb {tb_addr_gen_2d tb_pingpong tb_regfile} {
    echo "================ $tb ================"
    vsim -onfinish stop work_legacy.$tb
    run -all
    quit -sim
}

echo "================ XONG ================"
if {[batch_mode]} { quit -f }
