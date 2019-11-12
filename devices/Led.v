module LED_DEVICE(LEDR, ABUS, DBUS, WE, CLK, RESET);
  parameter BITS;
  parameter BASE;

  output wire [9:0] LEDR;
  input wire [BITS-1:0] ABUS;
  inout wire [BITS-1:0] DBUS;
  input wire WE,CLK,RESET;

  reg [9:0] LED_register;

  wire sel_data = ABUS == BASE;             //address of LEDR
  wire wr_data = WE && sel_data;
  wire rd_data = !WE && sel_data;

  always @(posedge CLK or posedge RESET) begin
  	if (RESET)
  		LED_register <= {10{1'b0}};
  	else if (wr_data)
  		LED_register <= DBUS[9:0];
  end

  assign LEDR = LED_register;

  assign DBUS = rd_data ? {{22{1'b0}}, LED_register} : {BITS{1'bz}};
endmodule