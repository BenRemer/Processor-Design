transcript on
if ![file isdirectory project3_frame_iputf_libs] {
	file mkdir project3_frame_iputf_libs
}

if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

###### Libraries for IPUTF cores 
###### End libraries for IPUTF cores 
###### MIF file copy and HDL compilation commands for IPUTF cores 


vlog "C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3/Pll_sim/Pll.vo"

vlog -vlog01compat -work work +incdir+C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3 {C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3/SevenSeg.v}
vlog -vlog01compat -work work +incdir+C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3 {C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3/project3_frame.v}

vlog -vlog01compat -work work +incdir+C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3 {C:/Users/benar/OneDrive/Documents/School/ProcessorDesign/project3/tb_project3.v}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cyclonev_ver -L cyclonev_hssi_ver -L cyclonev_pcie_hip_ver -L rtl_work -L work -voptargs="+acc"  tb_project3

add wave *
view structure
view signals
run -all
