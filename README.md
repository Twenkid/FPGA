# FPGA
CPU, FPGA, HDL, Verilog, VHDL, Icarus, GTKWave, Digital; Design &amp; Verification, System Verilog, testbench, dynamic reconfiguration, Xilinx, Virtex, Spartan, ...



https://steveicarus.github.io/iverilog/

https://github.com/hneemann/Digital

biRISC-V - 32-bit dual issue RISC-V CPU: https://github.com/ultraembedded/biriscv
https://github.com/ultraembedded/biriscv/tree/master/src/core
https://github.com/ultraembedded/biriscv/blob/master/src/tcm/dport_axi.v
https://github.com/ultraembedded/cores/blob/master/usb_device/src_v/usbf_fifo.v

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
