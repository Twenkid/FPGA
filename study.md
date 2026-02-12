# Study, explore, ... 
* Power A20 ...
"compiles with verilator, iverilog, yosys
runs simple version of kernel/bios/random test with cocotb (A2L2 interface partially implemented in Python) and Verilog core wrappers (A2L2<->mem/wb interfaces)
wrapper converts A2L2 interface to mem and Wishbone interfaces
verilator now runs with a2o_litex and litex SOC
verilator & litex software build working for 32BE but looks like problem with printf %d
verilator & litex software build working for 64LE with same printf errors"

* https://github.com/OpenPOWERFoundation/a2o/tree/master
* https://github.com/OpenPOWERFoundation/a2o/blob/master/dev/verilog/work/fu_gst.v
* https://www.cocotb.org/  - Testbench - cocotb is an open source coroutine-based cosimulation testbench environment for verifying VHDL and SystemVerilog RTL using Python.
