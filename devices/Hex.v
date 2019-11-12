module HEX_DEVICE(HEX, ABUS, DBUS, WE, CLK, RESET);
  parameter BITS;
  parameter BASE;

  output wire [41:0] HEX;
  input wire [BITS-1:0] ABUS;
  inout wire [BITS-1:0] DBUS;
  input wire WE,CLK,RESET;

  reg [23:0] HEX_out;

  wire sel_data = ABUS == BASE;             //address of HEX
  wire wr_data = WE && sel_data;
  wire rd_data = !WE && sel_data;

  always @(posedge CLK or posedge RESET) begin
  	if (RESET)
  		HEX_out <= 24'hFEDEAD;
  	else if (wr_data)
  		HEX_out <= DBUS[23:0];
	end

  SevenSeg ss5(.OUT(HEX[41:35]), .IN(HEX_out[23:20]), .OFF(1'b0));
  SevenSeg ss4(.OUT(HEX[34:28]), .IN(HEX_out[19:16]), .OFF(1'b0));
  SevenSeg ss3(.OUT(HEX[27:21]), .IN(HEX_out[15:12]), .OFF(1'b0));
  SevenSeg ss2(.OUT(HEX[20:14]), .IN(HEX_out[11:8]), .OFF(1'b0));
  SevenSeg ss1(.OUT(HEX[13:7]), .IN(HEX_out[7:4]), .OFF(1'b0));
  SevenSeg ss0(.OUT(HEX[6:0]), .IN(HEX_out[3:0]), .OFF(1'b0));

  assign DBUS = rd_data ? {{BITS-24{1'b0}}, HEX_out} : {BITS{1'bz}};
endmodule