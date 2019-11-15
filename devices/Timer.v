module TIMER_DEVICE(ABUS, DBUS, WE, INTR, CLK, RESET);
  parameter WBITS; // passed in as 32
  parameter CBITS; // passed in as 5
  parameter BASE; // passed in address
  parameter ONE_MS = 15000; // 15,000,000 cycles per second ==> 15,000 cycles per millisecond

  input wire [WBITS-1:0] ABUS;
  inout wire [WBITS-1:0] DBUS;
  input wire WE,CLK,RESET;
  output wire INTR;

  // LIM is passed a value in milliseconds to count to
  // CNT represents the number of ms we've already counted to get to LIM
  // CLK_COUNT represents the number of clock ticks to get to 1 ms
  reg [CBITS-1:0] CTRL;
  reg [WBITS-1:0] CNT, LIM, CLK_COUNT;

  wire sel_cnt = ABUS == BASE;             //address of CNT
  wire wr_cnt = WE && sel_cnt;
  wire rd_cnt = !WE && sel_cnt;

  wire sel_lim = ABUS == (BASE + 32'd4);   //address of LIM
  wire wr_lim = WE && sel_lim;
  wire rd_lim = !WE && sel_lim;

  wire sel_ctrl = ABUS == (BASE + 32'd8);   //address of CTRL (control/status)
  wire wr_ctrl = WE && sel_ctrl;
  wire rd_ctrl = !WE && sel_ctrl;

  //Writes
  always @(posedge CLK or posedge RESET) begin
    if (RESET) begin
      CNT <= {WBITS{1'b0}};
      LIM <= {WBITS{1'b0}};
      CLK_COUNT <= {WBITS{1'b0}};
      CTRL <= {CBITS{1'b0}};
    end else begin
      if (wr_cnt)   //if writing to tcnt
        CNT <= DBUS;
      else if (wr_lim)       //if writing to tlim
        LIM <= DBUS;
      else if (wr_ctrl)       //if writing to tctl
        CTRL <= {DBUS[4:2], DBUS[1] && CTRL[1], CTRL[0]}; // 4 is IE, 1 is overrun, 0 is ready (data change has been detected)
      else if (CLK_COUNT >= ONE_MS) begin     //number of clocks in 1 ms
        CNT <= CNT + 1;
      end else if ((CNT >= LIM - 1) && (LIM != 0)) begin //if counter reached
        CNT <= 0;
        if (CTRL[0])    //if ready == 1 already then overflow
          CTRL[1] <= 1;
        CTRL[0] <= 1;
      end
      CLK_COUNT <= (wr_cnt || wr_lim) ? {WBITS{1'b0}} : (CLK_COUNT + 1) % (ONE_MS + 1);
    end
  end

  //Reads
  assign DBUS = rd_cnt ? CNT :
                rd_lim ? LIM :
                rd_ctrl ? CTRL :
                {WBITS{1'bz}}; 

  assign INTR = CTRL[0] && CTRL[4];
endmodule
