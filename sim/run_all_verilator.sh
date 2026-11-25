#!/bin/bash
# Verilator >= 5.0 – chạy trong thư mục sim/
set -e
OPT="--binary --timing -j 8 -O2 -Wno-fatal -Wno-lint -Wno-style"
TB=../tb
build() { top=$1; shift; verilator $OPT --top-module $top -Mdir obj_$top -f rtl.f "$@" >/dev/null; }
for t in tb_roi_desc_queue tb_page_splitter tb_sched_cpl tb_fbm_pingpong tb_axi_master_wr tb_read_path; do
    build $t $TB/$t.sv; ./obj_$t/V$t
done
build tb_u02_top  $TB/tb_u02_top.sv;                      ./obj_tb_u02_top/Vtb_u02_top   +FRAMES=3
build tb_u02_u01m $TB/u01_model.sv $TB/tb_u02_u01m.sv;    ./obj_tb_u02_u01m/Vtb_u02_u01m +FRAMES=2 +U01_QUIET
build tb_lat      $TB/tb_lat.sv;                          ./obj_tb_lat/Vtb_lat           +RLAT=30 +FASTISP
