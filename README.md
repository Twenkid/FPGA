# FPGA
CPU, FPGA, HDL, Verilog, VHDL, Icarus, GTKWave, Digital; Design &amp; Verification, System Verilog, testbench, dynamic reconfiguration, Xilinx, Virtex, Spartan, ...

* See also the designs in: https://github.com/Twenkid/ASIC-FPGA-Verilog
* A DSP block and LRU Cache (2008), pipelines with registers for delaying and synchronizing the signals
https://github.com/Twenkid/ASIC-FPGA-Verilog/tree/main/DataCacheTag


https://github.com/Twenkid/ASIC-FPGA-Verilog/blob/main/DataCacheTag/cache0.txt   
https://github.com/Twenkid/ASIC-FPGA-Verilog/blob/main/DataCacheTag/drawtiming-commands.txt   

Generating time diagrams from a specification of buses and events with drawtiming.

<img src="https://github.com/Twenkid/ASIC-FPGA-Verilog/blob/main/DataCacheTag/cache0.gif">  

```
           CLOCK=0, TICK="X", RESET=0, ADDR=X, RD_WR_B=0, VALID=0, RA=X, RE=0, Q=X, TAG_0_T4=X, TAG_0_T5=X, EQ_0_T4=X, WRITE_HIT_T5 =0, WRITE_MISS_T5=0, READ_HIT_T5=0, READ_MISS_T5=0, REPLACE = 0, WE=X, WA=X, DATA_IN=X, ADDR_IN_T5=X.
	   CLOCK=1.
	   CLOCK=0.
	   CLOCK=1, TICK="0", RESET=1, RD_WR_B=1, VALID=1, ADDR = A0.
           CLOCK=0.
           CLOCK=1, TICK="1", ADDR = A1.
           CLOCK=0.
           CLOCK=1, TICK="2", RA="A0", RE=1, ADDR = A2.
           CLOCK=0.
           CLOCK=1, TICK="3", RE=1, RA="A1", Q="Q0", ADDR = A3.
           CLOCK=0.
           CLOCK=1, TICK="4", RA="A2", Q="Q1", ADDR = A4, TAG_0_T4 = X, EQ_0_T4=0.
           CLOCK=0.
           CLOCK=1, TICK="5", RA="A4", Q="Q3", ADDR = A6, WA="Tg_0", WE=1, DATA_IN="D0", ADDR_IN_T5=A0, TAG_0_T4 = X, TAG_0_T5=X, WRITE_HIT_T5=0, WRITE_MISS_T5=0, READ_HIT_T5=0, READ_MISS_T5=1, REPLACE = 1.
           CLOCK=0.
           CLOCK=1, TICK="6", RA="A5", Q="Q4", ADDR = A7, WA="Tg_1", WE=1, DATA_IN="D1", ADDR_IN_T5=A1, TAG_0_T4 = TG_T2.
           CLOCK=0.
           CLOCK=1, TICK="7", RA="A6", Q="Q5", ADDR = A8, WA="0", WE=0, DATA_IN="X", ADDR_IN_T5=A2, TAG_0_T4 = TG_T3.
           CLOCK=0.
           CLOCK=1, TICK="8", RA="A7", Q="Q6", ADDR = A9, WA="0", WE=0, DATA_IN="X", ADDR_IN_T5=A3, TAG_0_T4 = TG_T4.
           CLOCK=0.



 drawtiming --output out2.gif newc
 drawtiming -w40 -c30 -f12 --output out3.gif newc
 drawtiming -w40 -c30 -f12 --output cache0.gif cache0.txt
```

...
* See Dictionary below

* https://github.com/alfikpl/ao486  486SX full implementation  ... with SoC .. 
* https://github.com/jaywonchung/Verilog-Harvard-CPU/
* https://github.com/jaywonchung/Verilog-Harvard-CPU/blob/master/02.%20Multi-cycle%20CPU/ROM.v
* https://github.com/jaywonchung/Verilog-Harvard-CPU/tree/master/06.%20Tournament%20Prediction%20CPU
* https://github.com/jaywonchung/Verilog-Harvard-CPU/blob/master/02.%20Multi-cycle%20CPU/Design.pdf
* https://github.com/alfikpl/aoOCS
* https://github.com/alfikpl/aoR3000
* https://github.com/alfikpl/aoR3000/tree/master
* https://github.com/alfikpl/aoR3000/blob/master/rtl/aoR3000.v
* https://github.com/alfikpl/ao68000/blob/master/rtl/ao68000.v
* https://github.com/ultraembedded/cores/blob/master/sdram/rtl/sdram.v
* https://github.com/ultraembedded/cores/tree/master/usb_device/src_v
* https://github.com/ultraembedded/cores
* https://github.com/ultraembedded/riscv?tab=readme-ov-file
* https://github.com/ultraembedded/riscv/blob/master/doc/riscv_isa_spec.pdf
* https://github.com/ultraembedded/riscv/blob/master/doc/riscv_privileged_spec.pdf
* https://github.com/ultraembedded/riscv/tree/master/core/riscv
* https://github.com/ultraembedded/riscv/blob/master/core/riscv/riscv_core.v
* https://github.com/ultraembedded/riscv/blob/master/core/riscv/riscv_alu.v
* https://github.com/ultraembedded/riscv/blob/master/core/riscv/riscv_decoder.v
* https://github.com/ultraembedded/riscv/blob/master/core/riscv/riscv_regfile.v
* https://github.com/ultraembedded/riscv/blob/master/core/riscv/riscv_multiplier.v
* https://github.com/ultraembedded
* https://github.com/ultraembedded?tab=repositories
* https://github.com/ultraembedded/core_jpeg
* https://github.com/ultraembedded/core_jpeg/tree/main/src_v
* https://github.com/ultraembedded/core_jpeg/blob/main/src_v/jpeg_input.v
* https://github.com/ultraembedded/biriscv
* https://github.com/ultraembedded/biriscv/blob/master/src/icache/icache.v
* https://github.com/ultraembedded/biriscv/blob/master/src/top/riscv_top.v


...
  
### Software, examples

* https://steveicarus.github.io/iverilog/     https://steveicarus.github.io/iverilog/usage/simulation.html
* https://github.com/hneemann/Digital - Design & simulate logic
* biRISC-V - 32-bit dual issue RISC-V CPU: https://github.com/ultraembedded/biriscv
* https://github.com/ultraembedded/biriscv/tree/master/src/core
* https://github.com/ultraembedded/biriscv/blob/master/src/tcm/dport_axi.v
* https://github.com/ultraembedded/cores/blob/master/usb_device/src_v/usbf_fifo.v

### Learn Verilog
* https://www.chipverify.com/verilog/verilog-interview-questions-set-2 (etc. 3,4,5...) 
https://www.chipverify.com/verilog/verilog-define-macros  
https://www.chipverify.com/verilog/verilog-vcd  
https://www.chipverify.com/verilog/verilog-dump-vcd  
https://www.chipverify.com/verilog/verilog-interview-questions-set-14  
https://www.chipverify.com/verilog/verilog-interview-questions-set-1  
https://www.chipverify.com/  
https://www.chipverify.com/verilog/verilog-examples  
https://www.chipverify.com/verilog/verilog-single-port-ram

https://github.com/1sand0s-git/8bit_CPU
https://github.com/1sand0s-git/8bit_CPU/blob/main/QA_Main.v
https://github.com/1sand0s-git/FPGABasics/blob/main/Episode%20008%20-%20ROMs%20and%207%20Segment%20Displays/QA_7Seg.mem
https://austinmorlan.com/posts/8bit_breadboard_fpga/
https://code.austinmorlan.com/austin/2021-8bit-cpu-fpga
https://code.austinmorlan.com/austin/2021-8bit-cpu-fpga/src/branch/master/code
https://code.austinmorlan.com/austin/2021-8bit-cpu-fpga/src/branch/master/code/cpu.v

https://steveicarus.github.io/iverilog/usage/simulation.html

http://eprints.utar.edu.my/5966/1/David_Ngu_Teck_Joung_21AGM06719.pdf   DESIGN AND SIMULATE RISC-V PROCESOR USING VERILOG, DAVID NGU TECK JOUNG


### Quick Verilog Samples:


```Verilog
`define ALU_NONE                                4'b0000
always @*

//FSM, decimal
localparam STATE_W                       = 3;
localparam STATE_RX_IDLE                 = 3'd0;
localparam STATE_RX_DATA                 = 3'd1;
localparam STATE_RX_DATA_READY           = 3'd2;

reg [STATE_W-1:0] state_q;
reg [79:0] cpu_state;
reg intr_o, intr_q;

assign intr_o = intr_q;

//wire data_in=
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    intr_q <= 1'b0;
// SOF
else if (frame_valid_w && reg_int_en_sof_i)
    intr_q <= 1'b1;
// Reset event
else if (!rst_event_q && usb_rst_w)
    intr_q <= 1'b1;


...

always @ (posedge clk or posedge rst)
  if (rst)
    x <= 32'b0;
  else if (valid)
    x <= rx_data;

always @ *
begin
    tx_d_valid_r = 1'b0;
    tx_data_strb_r  = 1'b0;
    tx_data_r       = 8'b0;
    tx_data_last_r  = 1'b0;

    4'd2:
    begin
        tx_data_valid_r = ep2_tx_data_valid_i;
        tx_data_strb_r  = ep2_tx_data_strb_i;
        tx_data_r       = ep2_tx_data_i;
        tx_data_last_r  = ep2_tx_data_last_i;
    end
...
    default: tx_data = epx_1;
    endcase    
end


//////////////
//usbf_fifo.v
///////////////

module usbf_fifo
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [  7:0]  data_i
    ,input           push_i
    ,input           pop_i
    ,input           flush_i

    // Outputs
    ,output          full_o
    ,output          empty_o
    ,output [  7:0]  data_o
);

parameter WIDTH   = 8;
parameter DEPTH   = 4;
parameter ADDR_W  = 2;
//-----------------------------------------------------------------
// Local Params
//-----------------------------------------------------------------
localparam COUNT_W = ADDR_W + 1;

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [WIDTH-1:0]         ram [DEPTH-1:0];
reg [ADDR_W-1:0]        rd_ptr;
reg [ADDR_W-1:0]        wr_ptr;
reg [COUNT_W-1:0]       count;

///////////////

usbf_fifo #(.WIDTH(16), .DEPTH(4), .ADDR_W(2)) usb_fifo_instance (clk, ...);

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    count   <= {(COUNT_W) {1'b0}};  //repeat 0
    rd_ptr  <= {(ADDR_W) {1'b0}};
    wr_ptr  <= {(ADDR_W) {1'b0}};
end
else
begin
  ...

if (push_i & ~full_o)
    begin
        ram[wr_ptr] <= data_i;      //non-blocking assignment <=, parallel
        wr_ptr      <= wr_ptr + 1;
    end

//Blocking: sequential, where the order is important, Combinatorial logic
always @(*) begin
    a = b + c; // a gets computed immediately
    d = a - e; // d uses the updated value of a
end

/* verilator lint_off WIDTH */
assign full_o    = (count == DEPTH);
assign empty_o   = (count == 0);

// continuous assignment, meaning the right-hand side is always evaluated, and the left-hand side net // is continuously updated with that value. Righthand --> Combinatorial logic e.g. state 
...
end_module


```

https://www.chipverify.com/verilog/verilog-synthesis
Synthesis:  Vivado, ...
Synopsys Design Constraints (SDC) 
```
# Set the version of the SDC file
set_version 2.1

# Define the clock
create_clock -period 10 [get_ports clk]  ; # 100 MHz clock
https://www.chipverify.com/verilog/verilog-synthesis
```

### Ensure that the Verilog code adheres to synthesizable constructs; not all Verilog features are suitable for synthesis.

* Initial Blocks	- only testbenches, not synthesizable;ignored during synthesis.
* Delay Constructs	- #10 ... only for simulation.
* Real Data Types	- The real and time data types are not synthesizable.
* Fork/Join Constructs
* Random Functions	- $random, not synthesizable.
* X and Z States	- unknown (x) and high impedance (z) - not allowed in synthesizable designs.
* Primitives - Only gate level primitives are supported.
* Force and Release	- Force and release of data types not supported.
...

Incremental compilation https://adaptivesupport.amd.com/s/article/696400?language=en_US

Out-Of-Context Synthesis  https://adaptivesupport.amd.com/s/article/694864?language=en_US

Global – Performs a traditional top-down synthesis of the entire design. ... 


### Interview questions ...
https://www.chipverify.com/verilog/verilog-interview-questions-set-15#how-is-the-connectivity-established-in-verilog-when-connecting-wires-of-different-widths
```verilog
// Causes all unconnected input ports following this to be pulled down to logic 0
`unconnected_drive pull0

module mod_2341( ... );
 ...
endmodule

// Do not apply for rest of the code
`nounconnected_drive
```

### Designing 8-bit CPU etc. Youtube ... 

Designing an 8 bit CPU - Episode 1, 1s and 0s, 2,03 хил. абонати, 1490 показвания  29.04.2022 г.


### Dictionary
* LUT - Look-up table, BRAM, DSP, LUTRAM ...
* FF, FFs - Filp-flops; LU, LE - logic units, logic elements ... e.g. Virtex ... 114000 ... (can fit 486SX with Sound card and periphery; the CPU - about 37K, ... Sound about so or more, ... in total about 90 ... see ao486 )
* Clock dividers with FFs and PLLs: PLL stands for Phase-Locked Loop. It is a closed-loop feedback control system used to generate an output signal whose phase is related to the phase of an input signal .. Phase Detector (or Phase Comparator): ... Loop Filter: This is typically a low-pass filter ... Voltage-Controlled Oscillator (VCO):
* The AXI (Advanced eXtensible Interface); Tightly Coupled Memory (TCM) module.

