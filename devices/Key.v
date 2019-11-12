module KEY_DEVICE(KEY, ABUS, DBUS, WE, INTR, CLK, RESET);
  parameter WBITS;
  parameter DBITS;
  parameter CBITS;
  parameter BASE;

  input wire [DBITS-1:0] KEY;
  input wire [WBITS-1:0] ABUS;
  inout wire [WBITS-1:0] DBUS;
  input wire WE,CLK,RESET;
  output wire INTR;

  reg [DBITS-1:0] DATA;
  reg [CBITS-1:0] CTRL;

  wire sel_data = ABUS == BASE;             //address of DATA
  wire rd_data = !WE && sel_data;

  wire sel_ctrl = ABUS == (BASE + 32'd4);   //address of CTRL (control/status)
  wire wr_ctrl = WE && sel_ctrl;
  wire rd_ctrl = !WE && sel_ctrl;

  //Writes
  always @(posedge CLK or posedge RESET) begin
  	if (RESET) begin
  		CTRL <= {CBITS{1'b0}};
  		DATA <= {DBITS{1'b0}};
    end else begin
      if (rd_data)              //if reading DATA
        CTRL[0] <= 0;
      else if (wr_ctrl)
        CTRL <= {DBUS[4:2], DBUS[1] & CTRL[1], CTRL[0]};
      else if (DATA != KEY) begin   //if change in DATA detected
        if (CTRL[0])
          CTRL[1] <= 1;             //overrun bit set
        CTRL[0] <= 1;               //ready bit set
      end

      DATA <= KEY;                  //sets DATA.
    end
  end

	//Reads
  assign DBUS = rd_data ? {{WBITS-DBITS{1'b0}}, DATA} :
                rd_ctrl ? {{WBITS-CBITS{1'b0}}, CTRL} :
                {WBITS{1'bz}};

  assign INTR = CTRL[0] && CTRL[4];
endmodule