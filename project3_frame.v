//`include "devices/Timer.v"
//`include "devices/Hex.v"
//`include "devices/Key.v"
//`include "devices/Led.v"
//`include "devices/Switch.v"
//`include "devices/SignExtend.v"
//`include "devices/SevenSeg.v"

module project3_frame(
  input        CLOCK_50,
  input        RESET_N,
  input  [3:0] KEY,
  input  [9:0] SW,
  output [6:0] HEX0,
  output [6:0] HEX1,
  output [6:0] HEX2,
  output [6:0] HEX3,
  output [6:0] HEX4,
  output [6:0] HEX5,
  output [9:0] LEDR
);

  parameter DBITS    = 32;
  parameter INSTSIZE = 32'd4;
  parameter INSTBITS = 32; //instruction bits
  parameter REGNOBITS = 4;
  parameter REGWORDS = (1 << REGNOBITS);
  parameter IMMBITS  = 16;
  parameter STARTPC  = 32'h100;
  parameter ADDRHEX  = 32'hFFFFF000;
  parameter ADDRLEDR = 32'hFFFFF020;
  parameter ADDRKEY  = 32'hFFFFF080;
  parameter ADDRSW   = 32'hFFFFF090;
  parameter NOP		= 32'b0;
  parameter INTRPC	= 32'h20;
  parameter ADDRTIMER = 32'hFFFFF100;
  parameter ADDRKCTRL = 32'hFFFFF084;
  parameter ADDRSCTRL = 32'hFFFFF094;
  parameter TIMERDBITS = 32;
  parameter SWITCHDBITS = 10;
  parameter KEYDBITS = 4;
  parameter CTRLBITS = 5;

  // test file location
//  parameter IMEMINITFILE = "part2-tests/test.mif";
//   parameter IMEMINITFILE = "part2-tests/fmedian2.mif";
//   parameter IMEMINITFILE = "part2-tests/xmax.mif";
//   parameter IMEMINITFILE = "in_class_assignment_11_14/test_intr.mif";
  parameter IMEMINITFILE = "in_class_assignment_11_14/xmax.mif";

  parameter IMEMADDRBITS = 16;
  parameter IMEMWORDBITS = 2;
  parameter IMEMWORDS	 = (1 << (IMEMADDRBITS - IMEMWORDBITS));
  parameter DMEMADDRBITS = 16;
  parameter DMEMWORDBITS = 2;
  parameter DMEMWORDS	 = (1 << (DMEMADDRBITS - DMEMWORDBITS));

  /* OP1 */
  parameter OP1BITS  = 6;
  parameter OP1_ALUR = 6'b000000;
  parameter OP1_BEQ  = 6'b001000;
  parameter OP1_BLT  = 6'b001001;
  parameter OP1_BLE  = 6'b001010;
  parameter OP1_BNE  = 6'b001011;
  parameter OP1_JAL  = 6'b001100;
  parameter OP1_LW   = 6'b010010;
  parameter OP1_SW   = 6'b011010;
  parameter OP1_ADDI = 6'b100000;
  parameter OP1_ANDI = 6'b100100;
  parameter OP1_ORI  = 6'b100101;
  parameter OP1_XORI = 6'b100110;
  parameter OP1_SYS	= 6'b011111;

  /* OP2 */
  parameter OP2BITS  = 8;
  parameter OP2_EQ   = 8'b00001000;
  parameter OP2_LT   = 8'b00001001;
  parameter OP2_LE   = 8'b00001010;
  parameter OP2_NE   = 8'b00001011;
  parameter OP2_ADD  = 8'b00100000;
  parameter OP2_AND  = 8'b00100100;
  parameter OP2_OR   = 8'b00100101;
  parameter OP2_XOR  = 8'b00100110;
  parameter OP2_SUB  = 8'b00101000;
  parameter OP2_NAND = 8'b00101100;
  parameter OP2_NOR  = 8'b00101101;
  parameter OP2_NXOR = 8'b00101110;
  parameter OP2_RSHF = 8'b00110000;
  parameter OP2_LSHF = 8'b00110001;
  parameter OP2_NOP 	= 8'b00000000;
  
  parameter OP2_RETI	= 8'b00000001;
  parameter OP2_RSR 	= 8'b00000010;
  parameter OP2_WSR	= 8'b00000011;

  parameter HEXBITS  = 24;
  parameter LEDRBITS = 10;
  parameter KEYBITS = 4;
  

  //*** PLL ***//
  // The reset signal comes from the reset button on the DE0-CV board
  // RESET_N is active-low, so we flip its value ("reset" is active-high)
  // The PLL is wired to produce clk and locked signals for our logic
  wire clk;
  wire locked;
  wire reset;

  Pll myPll(
    .refclk	(CLOCK_50),
    .rst     	(!RESET_N),
    .outclk_0 	(clk),
    .locked   	(locked)
  );

  assign reset = !locked;
  
  
  wire IRQ;
  wire RETI;
  wire intr_key;
  wire intr_sws;
  wire intr_timer;
  
  reg [DBITS-1:0] PCS;
  reg [DBITS-1:0] IHA;
  reg [DBITS-1:0] IRA;
  reg [DBITS-1:0] IDN;

 //  wire [DBITS-1:0] aluout_EX_w;
	// // Forward EX
	// wire forward_from_exstage_rs;
	// wire forward_from_exstage_rt;
	// wire forward_from_exstage;
	
	// // Forward MEM
	// wire forward_from_memstage_rs;
	// wire forward_from_memstage_rt;
	// wire forward_from_memstage;
	
	// // Forward WB
	// wire forward_from_wbstage_rs;
	// wire forward_from_wbstage_rt;
	// wire forward_from_wbstage;

  //*** FETCH STAGE ***//
  // The PC register and update logic
  wire [DBITS-1:0] pcplus_FE;
  wire [DBITS-1:0] pcpred_FE;
  wire [DBITS-1:0] inst_FE_w;
  wire stall_pipe;
  wire mispred_EX_w;
  // wire send_nop;
  // wire send_nop_EX_w;
  // wire send_nop_MEM_w;
  // wire reti;
  wire [DBITS-1:0] pcgood_EX_w;

  // reg [DBITS-1:0] pcgood_EX;
  reg [DBITS-1:0] PC_FE;
  reg [INSTBITS-1:0] inst_FE;
  // I-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0] imem [IMEMWORDS-1:0];
  // reg mispred_EX;

  // This statement is used to initialize the I-MEM during simulation using Model-Sim
//  initial begin
//    $readmemh("part2-tests/fmedian2.hex", imem); //TODO: change sim/model/tests/*.hex
//	 $readmemh("part2-tests/fmedian2.hex", dmem);
//  end

  assign inst_FE_w = imem[PC_FE[IMEMADDRBITS-1:IMEMWORDBITS]];

  always @ (posedge clk or posedge reset) begin
    if(reset)
      PC_FE <= STARTPC;
	 else if(IRQ)
		PC_FE <= IHA;
    else if(RETI)
      PC_FE <= IRA;	// IHA, set to 0x20
    else if(mispred_EX_w) // This represents a branch taken or jmp in EX stage
      PC_FE <= pcgood_EX_w; // assign to what was caclulated in EX stage
    else if(!stall_pipe)   // maybe &&?
      PC_FE <= pcpred_FE; // if no stall, assign to the incremented pc (PC + 4)
    // Take from IRA sys_reg: holds the starting PC of all instructions that 
    // were still in the pipeline when the interrupt occured
	 else
      PC_FE <= PC_FE; // if stall, PC_FE stays the same
  end

  // This is the value of "incremented PC", computed in the FE stage
  assign pcplus_FE = PC_FE + INSTSIZE; // PC + 4
  // This is the predicted value of the PC that we use to fetch the next instruction
  assign pcpred_FE = pcplus_FE;

  // FE_latch
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
      inst_FE <= {INSTBITS{1'b0}};
    end else begin
	   // Specify inst_FE considering misprediction and stall
		if(mispred_EX_w || IRQ || RETI) begin
			inst_FE <= {INSTBITS{1'b0}};
		end else if (stall_pipe) begin
			inst_FE <= inst_FE; // set it to itself so it stays here{INSTBITS{1'b0}};
		end else begin
			inst_FE <= inst_FE_w;
		 end
	  end
  end


  //*** DECODE STAGE ***//
  wire [OP1BITS-1:0] op1_ID_w;
  wire [OP2BITS-1:0] op2_ID_w;
  wire [IMMBITS-1:0] imm_ID_w;
  wire [REGNOBITS-1:0] rd_ID_w;
  wire [REGNOBITS-1:0] rs_ID_w;
  wire [REGNOBITS-1:0] rt_ID_w;
  // Two read ports, always using rs and rt for register numbers
  wire [DBITS-1:0] regval1_ID_w;
  wire [DBITS-1:0] regval2_ID_w;
  wire [DBITS-1:0] sxt_imm_ID_w;
  wire is_br_ID_w;
  wire is_jmp_ID_w;
  wire rd_mem_ID_w;
  wire wr_mem_ID_w;
  wire wr_reg_ID_w;
  wire [4:0] ctrlsig_ID_w;
  wire [REGNOBITS-1:0] wregno_ID_w;
  wire wr_reg_EX_w;
  wire wr_reg_MEM_w;
  wire [DBITS-1:0] out_MEM_w;

  wire is_alui_operation;
  wire is_op2_ID;
  wire is_op1_ID;
  wire [REGNOBITS-1:0] ss_ID_w;
  wire [REGNOBITS-1:0] sd_ID_w;
  wire [DBITS-1:0] sys_regval_ID_w;
  wire is_sys_instr_ID_w;
  wire is_wr_sys_ID_w;
  wire is_rd_sys_ID_w;
  wire is_reti_ID_w;

  // Register file
  reg [DBITS-1:0] PC_ID;
  reg [DBITS-1:0] regs [REGWORDS-1:0];
  reg signed [DBITS-1:0] regval1_ID;
  reg signed [DBITS-1:0] regval2_ID;
  reg signed [DBITS-1:0] immval_ID;
  reg [OP1BITS-1:0] op1_ID;
  reg [OP2BITS-1:0] op2_ID;
  reg [4:0] ctrlsig_ID;
  reg [REGNOBITS-1:0] wregno_ID;
  // Declared here for stall check
  reg [REGNOBITS-1:0] wregno_EX;
  reg [REGNOBITS-1:0] wregno_MEM;
  reg [INSTBITS-1:0] inst_ID;

  reg [INSTBITS-1:0] inst_ID_temp;
  reg [DBITS-1:0]	PC_ID_temp;
  reg is_sys_inst_ID;
  reg is_reti_ID;
  reg signed [DBITS-1:0] sys_regval_ID;

  // Specify signals such as op*_ID_w, imm_ID_w, r*_ID_w
  assign op1_ID_w = inst_FE[31:26];
  assign op2_ID_w = inst_FE[25:18];
  assign imm_ID_w = inst_FE[23:8];
  assign rd_ID_w = inst_FE[11:8];
  assign rs_ID_w = inst_FE[7:4];
  assign rt_ID_w = inst_FE[3:0];
  assign is_alui_operation = inst_FE[31];
  assign is_op2_ID = ((op1_ID_w == OP1_ALUR) && (op2_ID_w != OP2_NOP))  ? 1 : 0; // if op1 is all zeros, we know it is op2
  assign is_op1_ID = (op1_ID_w != OP1_ALUR) ? 1 : 0;

  // assign ss_ID_w = inst_FE[13:10];
  // assign sd_ID_w = inst_FE[17:14];

  assign is_sys_instr_ID_w = (op1_ID_w == OP1_SYS);
  assign is_wr_sys_ID_w = (op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_WSR);
  assign is_rd_sys_ID_w = (op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RSR);
  assign is_reti_ID_w = (op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RETI);
  
  // Read register values
  assign regval1_ID_w = regs[rs_ID_w];
  assign regval2_ID_w = regs[rt_ID_w];
  // assign sys_regval_ID_w = is_rd_sys_ID_w ? sys_regs[ss_ID_w] : (is_wr_sys_ID_w ? regs[ss_ID_w] : 4'b0);

  // Sign extension
  SXT mysxt (.IN(imm_ID_w), .OUT(sxt_imm_ID_w));

  // Specify control signals such as is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, etc.
  // You may add or change control signals if needed
  assign is_br_ID_w = (op1_ID_w === OP1_BEQ
							|| op1_ID_w === OP1_BLT
							|| op1_ID_w === OP1_BLE
							|| op1_ID_w === OP1_BNE) ? 1 : 0;
  assign is_jmp_ID_w = (op1_ID_w === OP1_JAL) ? 1 : 0;
  assign rd_mem_ID_w = (op1_ID_w === OP1_LW) ? 1 : 0; // are we reading from memory?
  assign wr_mem_ID_w = (op1_ID_w === OP1_SW) ? 1 : 0; // are we writing to memory?
  assign wr_reg_ID_w = (is_op2_ID				 			// any OP2 writes to a register
							|| is_alui_operation 	 			// any alui operation writes to a register
							|| op1_ID_w == OP1_JAL	 			// JAL and LW also write to a register
							|| op1_ID_w == OP1_LW) ? 1 : 0;  // are we writing to a register
  

  //wregno is the register number that will be written to
  // TODO: we must sign extend the system regnos, but still, we have to make sure forwarding doesn't work
  // assign wregno_ID_w = is_sys_instr_ID_w ? (is_wr_sys_ID_w ? sd_ID_w : sd_ID_w) : 
	// 	(is_jmp_ID_w || op1_ID_w == OP1_LW || is_alui_operation) ? rt_ID_w : (is_op2_ID ? rd_ID_w : 0);
  assign wregno_ID_w = (is_op2_ID || is_sys_instr_ID_w) ? rd_ID_w : rt_ID_w;

  // concatenates everything together to be put in buffers/registers later {4:0}
  assign ctrlsig_ID_w = {is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};

  // Stalling
  wire read_rt;
  // Rt is not used only in JAL, LW, and immediate arithmetic instructions
  assign read_rt = !(is_jmp_ID_w || rd_mem_ID_w|| is_alui_operation || is_sys_instr_ID_w);
  // Check if one of the registers this instruction depends on is going to be written to by the instructions 
  // in EX or MEM. All instructions depend on Rs - only stall on write to Rt if this instruction actually
  // depends on it.
  wire rs_ex_hazard_w;
  wire rs_mem_hazard_w;
  wire rt_ex_hazard_w;
  wire rt_mem_hazard_w;
  
  assign rs_ex_hazard_w = (wregno_ID == rs_ID_w && ctrlsig_ID[0]); //===
  assign rs_mem_hazard_w = (wregno_EX == rs_ID_w && ctrlsig_EX[0]);
  assign rt_ex_hazard_w = (read_rt && wregno_ID == rt_ID_w  && ctrlsig_ID[0]);
  assign rt_mem_hazard_w = (read_rt && wregno_EX == rt_ID_w && ctrlsig_EX[0]);
  assign stall_pipe = (rs_ex_hazard_w || rt_ex_hazard_w) 
      && (ctrlsig_ID[2] || rd_sys_EX_w); // if rd_mem or is rsr in ex
  
  // ID_latch
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
      PC_ID	 		<= {DBITS{1'b0}};
		inst_ID	 	<= {INSTBITS{1'b0}};
      op1_ID	 	<= {OP1BITS{1'b0}};
      op2_ID	 	<= {OP2BITS{1'b0}};
      regval1_ID  <= {DBITS{1'b0}};
      regval2_ID  <= {DBITS{1'b0}};
      wregno_ID	<= {REGNOBITS{1'b0}};
      ctrlsig_ID 	<= 5'h0;
		  is_sys_inst_ID <= 1'b0;
      is_reti_ID  <= 1'b0;
	 end else if(stall_pipe || mispred_EX_w || IRQ || RETI) begin // for some reason reset goes first and alone by convention
	 // Send nops that are all 1s because all 0s evaluates to isOp2 == true and false positive for send_nop
      PC_ID	 		<= {DBITS{1'b0}};
		inst_ID	 	<= {INSTBITS{1'b0}};
      op1_ID	 	<= {OP1BITS{1'b0}};
      op2_ID	 	<= {OP2BITS{1'b0}};
      regval1_ID  <= {DBITS{1'b0}};
      regval2_ID  <= {DBITS{1'b0}};
      wregno_ID	<= {REGNOBITS{1'b0}};
      ctrlsig_ID 	<= 5'h0;
		is_sys_inst_ID <= 1'b0;
    is_reti_ID  <= 1'b0;
	 end else begin
		PC_ID	 		<= PC_FE;
		inst_ID	 	<= inst_FE;
      op1_ID	 	<= op1_ID_w;
      op2_ID	 	<= op2_ID_w;
      wregno_ID	<= wregno_ID_w;
		ctrlsig_ID 	<= ctrlsig_ID_w;
		immval_ID 	<= sxt_imm_ID_w;
		is_sys_inst_ID <= is_sys_instr_ID_w;
		is_reti_ID  <= is_reti_ID_w;

		if (rs_ex_hazard_w) begin
			regval1_ID  <= aluout_EX_r;
		end else if (rs_mem_hazard_w) begin
			regval1_ID  <= out_MEM_w;
		end else begin
			regval1_ID  <=  regval1_ID_w;  
		end
		
		if (rt_ex_hazard_w) begin
			regval2_ID  <= aluout_EX_r;
		end else if (rt_mem_hazard_w) begin
			regval2_ID  <= out_MEM_w;
		end else begin
			regval2_ID  <= regval2_ID_w;
		end
	 end
	end

  //*** AGEN/EXEC STAGE ***//
  wire is_br_EX_w;
  wire is_jmp_EX_w;
  // wire [DBITS-1:0] pcgood_EX_w;

  reg [INSTBITS-1:0] inst_EX; /* This is for debugging */
  reg [DBITS-1:0] PC_EX;
  reg br_cond_EX;
  reg [2:0] ctrlsig_EX;
  // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
  reg signed [DBITS-1:0] aluout_EX_r;
  reg [DBITS-1:0] aluout_EX;
  reg [DBITS-1:0] regval1_EX;
  reg [DBITS-1:0] regval2_EX;
  reg [OP1BITS-1:0] op1_EX;
  reg [OP2BITS-1:0] op2_EX;
  reg [DBITS-1:0] pc_next_EX;

  // wire [OP1BITS-1:0] op1_EX_w;
  // wire [OP2BITS-1:0] op2_EX_w;
  // wire [REGNOBITS-1:0] rd_EX_w;
  // wire [REGNOBITS-1:0] rs_EX_w;
  // wire [REGNOBITS-1:0] rt_EX_w;
  // wire is_op2_EX;
  // wire is_op1_EX;
  wire rd_sys_EX_w;
  wire wr_sys_EX_w;
  reg rd_sys_EX;
  reg wr_sys_EX;
  reg is_sys_inst_EX;
  reg is_reti_EX;

  // assign op1_EX_w = inst_ID[31:26];
  // assign op2_EX_w = inst_ID[25:18];
  // assign rd_EX_w = inst_ID[11:8];
  // assign rt_EX_w = inst_ID[3:0];
  // assign is_op2_EX = (op1_EX_w == OP1_ALUR) && (op2_EX_w != OP2_NOP);
  // assign is_op1_EX = op1_EX_w != OP1_ALUR;

  always @ (op1_ID or regval1_ID or regval2_ID) begin
    case (op1_ID)
      OP1_BEQ : br_cond_EX = (regval1_ID == regval2_ID);
      OP1_BLT : br_cond_EX = (regval1_ID < regval2_ID);
      OP1_BLE : br_cond_EX = (regval1_ID <= regval2_ID);
      OP1_BNE : br_cond_EX = (regval1_ID != regval2_ID);
      default : br_cond_EX = 1'b0;
    endcase
  end

  always @ (op1_ID or op2_ID or regval1_ID or regval2_ID or immval_ID) begin
    if(op1_ID == OP1_ALUR)
      case (op2_ID)
			OP2_EQ	 : aluout_EX_r = {31'b0, regval1_ID == regval2_ID}; //changed to triple equals
			OP2_LT	 : aluout_EX_r = {31'b0, regval1_ID < regval2_ID};
			OP2_LE    : aluout_EX_r = {31'b0, regval1_ID <= regval2_ID};
			OP2_NE    : aluout_EX_r = {31'b0, regval1_ID != regval2_ID};
 			OP2_ADD   : aluout_EX_r = {31'b0, regval1_ID + regval2_ID};
  			OP2_AND   : aluout_EX_r = {31'b0, regval1_ID & regval2_ID};
 			OP2_OR    : aluout_EX_r = {31'b0, regval1_ID | regval2_ID};
 			OP2_XOR   : aluout_EX_r = {31'b0, regval1_ID ^ regval2_ID};
 			OP2_SUB   : aluout_EX_r = {31'b0, regval1_ID - regval2_ID};
 			OP2_NAND  : aluout_EX_r = {31'b0, ~(regval1_ID & regval2_ID)};
 			OP2_NOR   : aluout_EX_r = {31'b0, ~(regval1_ID | regval2_ID)};
 			OP2_NXOR  : aluout_EX_r = {31'b0, ~(regval1_ID ^ regval2_ID)};
 			OP2_RSHF  : aluout_EX_r = {31'b0, regval1_ID >> regval2_ID}; 
 			OP2_LSHF  : aluout_EX_r = {31'b0, regval1_ID << regval2_ID}; 
		default	 : aluout_EX_r = {DBITS{1'b0}};
	  endcase
    else if(op1_ID == OP1_LW || op1_ID == OP1_SW || op1_ID == OP1_ADDI)
      aluout_EX_r = regval1_ID + immval_ID;
    else if(op1_ID == OP1_ANDI)
      aluout_EX_r = regval1_ID & immval_ID;
    else if(op1_ID == OP1_ORI)
      aluout_EX_r = regval1_ID | immval_ID;
    else if(op1_ID == OP1_XORI)
      aluout_EX_r = regval1_ID ^ immval_ID;
	 else if(op1_ID == OP1_JAL)
		aluout_EX_r = PC_ID;
    else
      aluout_EX_r = {DBITS{1'b0}};
  end
  
  // assign aluout_EX_w = aluout_EX_r;
  assign is_br_EX_w = ctrlsig_ID[4];
  assign is_jmp_EX_w = ctrlsig_ID[3];
  assign wr_reg_EX_w = ctrlsig_ID[0];
  
  assign rd_sys_EX_w = is_sys_inst_ID && (op2_ID == OP2_RSR);
  assign wr_sys_EX_w = is_sys_inst_ID && (op2_ID == OP2_WSR);

  // Specify signals such as mispred_EX_w, pcgood_EX_w
  // calculates the new pc value for BR or JAL:
  assign mispred_EX_w = is_jmp_EX_w || br_cond_EX;
  assign pcgood_EX_w = is_jmp_EX_w ? (regval1_ID + 4*immval_ID) : (PC_ID + 4*immval_ID);
  
  // if branch, check if branch taken
  // if jump, no need to check, we know it is taken
  

  // EX_latch
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
	   inst_EX	 	<= {INSTBITS{1'b0}};
		PC_EX			<= {DBITS{1'b0}};
      aluout_EX	<= {DBITS{1'b0}};
      wregno_EX	<= {REGNOBITS{1'b0}};
      ctrlsig_EX 	<= 3'h0;
    regval1_EX  <= {DBITS{1'b0}};
		regval2_EX	<= {DBITS{1'b0}};
		wr_sys_EX 	<= 1'b0;
      rd_sys_EX 	<= 1'b0;
		is_sys_inst_EX <= 1'b0;
      is_reti_EX 	<= 1'b0;
		pc_next_EX    <= {DBITS{1'b0}};
	 end else if (IRQ || RETI) begin // Flush out EX state after branch or Jump
		inst_EX   <= {INSTBITS{1'b0}};
    PC_EX     <= {DBITS{1'b0}};
      aluout_EX <= {DBITS{1'b0}};
      wregno_EX <= {REGNOBITS{1'b0}};
      ctrlsig_EX  <= 3'h0;
    regval1_EX  <= {DBITS{1'b0}};
    regval2_EX  <= {DBITS{1'b0}};
    wr_sys_EX   <= 1'b0;
      rd_sys_EX   <= 1'b0;
    is_sys_inst_EX <= 1'b0;
      is_reti_EX  <= 1'b0;
    pc_next_EX    <= {DBITS{1'b0}};
    end else begin
		inst_EX	 	<= inst_ID;
		PC_EX			<= PC_ID;
      aluout_EX	<= aluout_EX_r;
      wregno_EX	<= wregno_ID;
      ctrlsig_EX 	<= {ctrlsig_ID[2], ctrlsig_ID[1], ctrlsig_ID[0]}; // MEM stage needs: read mem, write mem, and write reg
		regval1_EX	<= regval1_ID;
		regval2_EX 	<= regval2_ID; // pass this along for SW
		wr_sys_EX 	<= wr_sys_EX_w;
      rd_sys_EX 	<= rd_sys_EX_w;
		is_sys_inst_EX <= is_sys_inst_ID;
      is_reti_EX 	<= is_reti_ID;
		pc_next_EX    <= mispred_EX_w ? pcgood_EX_w : PC_ID;
    end
  end


  //*** MEM STAGE ***//

  wire [DBITS-1:0] abus;
  wire [DBITS-1:0] dbus;

  wire rd_mem_MEM_w;
  wire wr_mem_MEM_w;

  wire wr_sys_MEM_w;
  reg wr_sys_MEM;

  wire [DBITS-1:0] memaddr_MEM_w;
  wire [DBITS-1:0] rd_val_MEM_w;

  reg [INSTBITS-1:0] inst_MEM; /* This is for debugging */
  reg [DBITS-1:0] regval_MEM;
  reg ctrlsig_MEM;
  reg [DBITS-1:0] PC_MEM;
  // D-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0] dmem[DMEMWORDS-1:0];
  
  assign memaddr_MEM_w = aluout_EX;
  assign rd_mem_MEM_w = ctrlsig_EX[2];
  assign wr_mem_MEM_w = ctrlsig_EX[1];
  assign wr_reg_MEM_w = ctrlsig_EX[0];
  
  // // Read from D-MEM
  assign rd_val_MEM_w = memaddr_MEM_w >= 32'hFFFFF000 ? dbus :
                  dmem[memaddr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]];

  // Write to D-MEM
  always @ (posedge clk) begin
    if(wr_mem_MEM_w)
      dmem[memaddr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]] <= regval2_EX;
  end

  assign abus = memaddr_MEM_w;
  assign dbus = wr_mem_MEM_w ? regval2_EX : {32{1'bz}};

  // Interupts
  assign IRQ = inst_EX != {DBITS{!'b0}} && !is_sys_inst_EX && PCS[0] && (intr_key || intr_sws || intr_timer);
  assign RETI = is_reti_EX;

  // System Latch
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      PCS <= {DBITS{1'b0}};
      IHA <= {{DBITS-12{1'b0}},{12'h200}};
      IRA <= {DBITS{1'b0}};
      IDN <= {DBITS{1'b0}};
   end else begin
    if (wr_sys_EX) begin
      case (wregno_EX)
        4'h1: IRA <= regval1_EX;
        4'h2: IHA <= regval1_EX;
        4'h3: IDN <= regval1_EX;
        4'h4: PCS <= regval1_EX;
      endcase
    end else if (IRQ) begin 
      IRA <= pc_next_EX;
      PCS[1] <= PCS[0];
      PCS[0] <= 0;
      IDN <= intr_timer ? 32'h1 :
           intr_key   ? 32'h2 :
           intr_sws   ? 32'h3 :
                        32'h0 ;
    end else if (RETI) begin
      PCS[0] <= PCS[1];
    end
   end
  end

  wire [REGNOBITS-1:0] sys_reg_MEM_w;
  assign sys_reg_MEM_w = inst_EX[7:4];
  wire [DBITS-1:0] sys_val_MEM_w;
  assign sys_val_MEM_w = sys_reg_MEM_w === 4'h1 ? IRA :
                 sys_reg_MEM_w === 4'h2 ? IHA :
                 sys_reg_MEM_w === 4'h3 ? IDN :
                 sys_reg_MEM_w === 4'h4 ? PCS :
                 {DBITS{1'b0}};
  
  assign out_MEM_w = rd_mem_MEM_w ? rd_val_MEM_w : 
              rd_sys_EX ? sys_val_MEM_w :
              aluout_EX;

  // MEM_latch
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
	   inst_MEM		<= {INSTBITS{1'b0}};
      regval_MEM  <= {DBITS{1'b0}};
      wregno_MEM  <= {REGNOBITS{1'b0}};
      ctrlsig_MEM <= 1'b0;
    end else begin
		inst_MEM		<= inst_EX;
      regval_MEM  <= out_MEM_w;
      wregno_MEM  <= wregno_EX;
      ctrlsig_MEM <= ctrlsig_EX[0];
    end
  end

  /*** WRITE BACK STAGE ***/

  wire wr_reg_WB_w;
  // regs is already declared in the ID stage

  assign wr_reg_WB_w = ctrlsig_MEM;

  always @ (negedge clk or posedge reset) begin
    if(reset) begin
		regs[0] <= {DBITS{1'b0}};
		regs[1] <= {DBITS{1'b0}};
		regs[2] <= {DBITS{1'b0}};
		regs[3] <= {DBITS{1'b0}};
		regs[4] <= {DBITS{1'b0}};
		regs[5] <= {DBITS{1'b0}};
		regs[6] <= {DBITS{1'b0}};
		regs[7] <= {DBITS{1'b0}};
		regs[8] <= {DBITS{1'b0}};
		regs[9] <= {DBITS{1'b0}};
		regs[10] <= {DBITS{1'b0}};
		regs[11] <= {DBITS{1'b0}};
		regs[12] <= {DBITS{1'b0}};
		regs[13] <= {DBITS{1'b0}};
		regs[14] <= {DBITS{1'b0}};
		regs[15] <= {DBITS{1'b0}};
	 end else if(wr_reg_WB_w) begin
      regs[wregno_MEM] <= regval_MEM;
	 end
  end


 //  /***** STALL **************************************/

 //  //We check wrregno_ID because whatever is in the ID latch is actually in the EX stage!
 //  wire read_rs = (rs_ID_w != 0) && ((wregno_MEM == rs_ID_w) // mem is wb, ex is mem, id is ex
	// 	|| (wregno_EX == rs_ID_w) 
	// 	|| (wregno_ID == rs_ID_w)); 
		
 //  wire read_rt = (rt_ID_w != 0) && ((wregno_MEM == rt_ID_w) 
	// 	|| (wregno_EX == rt_ID_w) 
	// 	|| (wregno_ID == rt_ID_w));

 //  wire read_ss = (ss_ID_w != 0) && ((wregno_MEM == ss_ID_w) 
	// 	|| (wregno_EX == ss_ID_w) 
	// 	|| (wregno_ID == ss_ID_w));
  
 //  assign stall_pipe = (
 //    ((is_rd_sys_ID_w || is_wr_sys_ID_w) && read_ss)||
 //    (is_br_ID_w && (read_rs || read_rt)) 
	// 	||(is_jmp_ID_w && (read_rs)) 
	// 	||(ctrlsig_ID[2] && ((wregno_ID == rs_ID_w) || (wregno_ID == rt_ID_w)))
 //    );//rd_mem_ID_w && (read_rs))));// && ctrlsig_ID[2] && (wregno_ID == rs_ID_w)));
	// 	//||(is_alui_operation && (read_rs))
	// 	//||(wr_mem_ID_w && (read_rs || read_rt)));
	// 	//(is_op2_ID && (read_rs || read_rt)) 
		
	// //******************************************************************

	/*********FORWARDING*****************************************/	
	//aluout_EX_r // output value ex
	//rd_mem_MEM_w ? rd_val_MEM_w : aluout_EX; // output value mem
	//regval_mem // output wb
	
	// Forward EX
  // Don't forward if EX instruction is a sys_write or ID/RR instruction is a sys_read
	// assign forward_from_exstage_rs = ((rs_ID_w != 0) && (wregno_ID == rs_ID_w)) ? 1 : 0;
	// assign forward_from_exstage_rt = ((rt_ID_w != 0) && (wregno_ID == rt_ID_w)) ? 1 : 0; 
	// assign forward_from_exstage = (forward_from_exstage_rs || forward_from_exstage_rt) && !is_sys_instr_ID_w ? 1 : 0;
 //                                  // && !is_rd_sys_ID_w && !wr_sys_EX_w ? 1 : 0;
	
	// // Forward MEM
 //  // Don't forward if MEM instruction is a sys_write or ID/RR instruction is a sys_read
	// assign forward_from_memstage_rs = ((rs_ID_w != 0) && (wregno_EX == rs_ID_w)) ? 1 : 0;
	// assign forward_from_memstage_rt = ((rt_ID_w != 0) && (wregno_EX == rt_ID_w)) ? 1 : 0;
	// assign forward_from_memstage = (forward_from_memstage_rs || forward_from_memstage_rs) && !is_sys_instr_ID_w ? 1 : 0;
 //                                  // && !is_rd_sys_ID_w && !wr_sys_EX ? 1 : 0;
	
	// // Forward WB
 //  // Don't forward if WB instruction is a sys_write or ID/RR instruction is a sys_read
	// assign forward_from_wbstage_rs = ((rs_ID_w != 0) && (wregno_MEM == rs_ID_w)) ? 1 : 0;
	// assign forward_from_wbstage_rt = ((rt_ID_w != 0) && (wregno_MEM == rt_ID_w)) ? 1 : 0;
	// assign forward_from_wbstage = (forward_from_wbstage_rs || forward_from_wbstage_rs) && !is_sys_instr_ID_w ? 1 : 0;
                                // && !is_rd_sys_ID_w && !wr_sys_MEM ? 1 : 0;
	

	//*******************************************************/

  /*** I/O ***/
  // Create and connect HEX register
//  reg [23:0] HEX_out;
//
//  SevenSeg ss5(.OUT(HEX5), .IN(HEX_out[23:20]), .OFF(1'b0));
//  SevenSeg ss4(.OUT(HEX4), .IN(HEX_out[19:16]), .OFF(1'b0));
//  SevenSeg ss3(.OUT(HEX3), .IN(HEX_out[15:12]), .OFF(1'b0));
//  SevenSeg ss2(.OUT(HEX2), .IN(HEX_out[11:8]), .OFF(1'b0));
//  SevenSeg ss1(.OUT(HEX1), .IN(HEX_out[7:4]), .OFF(1'b0));
//  SevenSeg ss0(.OUT(HEX0), .IN(HEX_out[3:0]), .OFF(1'b0));
//
//  always @ (posedge clk or posedge reset) begin
//    if(reset)
//	   HEX_out <= 24'hFEDEAD;
//	 else if(wr_mem_MEM_w && (memaddr_MEM_w == ADDRHEX))
//      HEX_out <= regval2_EX[HEXBITS-1:0];
//  end
//
//  // Write the code for LEDR here
//
//  reg [9:0] LEDR_out;
//
//  // If the code stores a word into the memory address of our LEDR I/O device, then output the value to LEDR
//  always @ (posedge clk or posedge reset) begin
//    if(reset)
//	   LEDR_out <= 10'b0;
//	 else if(wr_mem_MEM_w && (memaddr_MEM_w == ADDRLEDR))
//      LEDR_out <= regval2_EX[LEDRBITS-1:0];
//  end
//  
//  assign LEDR = LEDR_out;

  // wire [31:0] regval2_MEM_w;

  // assign regval2_MEM_w = (wr_mem_MEM_w) ? regval2_EX : {DBITS{1'bz}};

  // assign abus = memaddr_MEM_w;
  // assign dbus = regval2_MEM_w;
  wire we;
  assign we = wr_mem_MEM_w;
  
   Ledr #(.BITS(DBITS), .BASE(ADDRLEDR)) ledr(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .WE(we),
	 .OUT(LEDR),
	 .CLK(clk),
	 .RESET(reset)
  );
  
   Hex #(.BITS(DBITS), .BASE(ADDRHEX)) hex(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .WE(we),
	 .OUTHEX5(HEX5),
	 .OUTHEX4(HEX4),
	 .OUTHEX3(HEX3),
	 .OUTHEX2(HEX2),
	 .OUTHEX1(HEX1),
	 .OUTHEX0(HEX0),
	 .CLK(clk),
	 .RESET(reset)
  );
  
  Key #(.BITS(DBITS), .BASE(ADDRKEY)) key(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .KEY(KEY),
	 .WE(we),
	 .INTR(intr_key),
	 .CLK(clk),
	 .RESET(reset)
  );
  
  Switch #(.BITS(DBITS), .BASE(ADDRSW)) switch(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .SW(SW),
	 .WE(we),
	 .INTR(intr_sws),
	 .CLK(clk),
	 .RESET(reset)
  );
  
  Timer #(.BITS(DBITS), .BASE(ADDRTIMER)) timer(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .WE(we),
	 .INTR(intr_timer),
	 .CLK(clk),
	 .RESET(reset)
  );
  
endmodule


module SXT(IN, OUT);
  parameter IBITS = 16;
  parameter OBITS = 32;

  input  [IBITS-1:0] IN;
  output [OBITS-1:0] OUT;

  assign OUT = {{(OBITS-IBITS){IN[IBITS-1]}}, IN};
endmodule

module Ledr(ABUS, DBUS, WE, OUT, CLK, RESET);
	parameter BITS;
	parameter BASE;

	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire WE, CLK, RESET;
	output wire [9:0] OUT;

	reg [9:0] LEDRDATA;

	wire selData = (ABUS === BASE);
	wire rdData = (!WE) && selData; 

	always @ (posedge CLK or posedge RESET) begin
		if (RESET) begin
			LEDRDATA <= 10'd0;
		end else begin
			
			if (WE && selData) begin
				LEDRDATA <= DBUS[9:0];
			end
			
		end
		
	end

	assign OUT = LEDRDATA;
	assign DBUS = rdData ? {22'b0,LEDRDATA} :
					  {BITS{1'bz}};
	
endmodule

module Hex(ABUS, DBUS, WE, OUTHEX5, OUTHEX4, OUTHEX3, OUTHEX2, OUTHEX1, OUTHEX0, CLK, RESET);
	parameter BITS;
	parameter BASE;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire WE, CLK, RESET;
	output wire [7:0] OUTHEX5, OUTHEX4, OUTHEX3, OUTHEX2, OUTHEX1, OUTHEX0;
	
	reg [23:0] HEXDATA;
	
	wire selData = (ABUS === BASE);
	wire rdData = (!WE) && selData; 
	
	always @ (posedge CLK or posedge RESET) begin
		if (RESET) begin
			HEXDATA <= 24'hFEDEAD;
		end else begin
		
			if (WE && selData) begin
				HEXDATA <= DBUS[23:0];
			end 
			
		end
	end
	
	wire [3:0] HEX5 = HEXDATA[23:20];
	wire [3:0] HEX4 = HEXDATA[19:16];
	wire [3:0] HEX3 = HEXDATA[15:12];
	wire [3:0] HEX2 = HEXDATA[11:8];
	wire [3:0] HEX1 = HEXDATA[7:4];
	wire [3:0] HEX0 = HEXDATA[3:0];
	
	assign OUTHEX5 =
		(1'b0)         ? 7'b1111111 : /* IN == OFF */
		(HEX5 == 4'h0) ? 7'b1000000 :
		(HEX5 == 4'h1) ? 7'b1111001 :
		(HEX5 == 4'h2) ? 7'b0100100 :
		(HEX5 == 4'h3) ? 7'b0110000 :
		(HEX5 == 4'h4) ? 7'b0011001 :
		(HEX5 == 4'h5) ? 7'b0010010 :
		(HEX5 == 4'h6) ? 7'b0000010 :
		(HEX5 == 4'h7) ? 7'b1111000 :
		(HEX5 == 4'h8) ? 7'b0000000 :
		(HEX5 == 4'h9) ? 7'b0010000 :
		(HEX5 == 4'hA) ? 7'b0001000 :
		(HEX5 == 4'hb) ? 7'b0000011 :
		(HEX5 == 4'hc) ? 7'b1000110 :
		(HEX5 == 4'hd) ? 7'b0100001 :
		(HEX5 == 4'he) ? 7'b0000110 :
		/*HEX5 == 4'hf*/ 7'b0001110 ;
  
  assign OUTHEX4 =
		(1'b0)         ? 7'b1111111 : /* IN == OFF */
		(HEX4 == 4'h0) ? 7'b1000000 :
		(HEX4 == 4'h1) ? 7'b1111001 :
		(HEX4 == 4'h2) ? 7'b0100100 :
		(HEX4 == 4'h3) ? 7'b0110000 :
		(HEX4 == 4'h4) ? 7'b0011001 :
		(HEX4 == 4'h5) ? 7'b0010010 :
		(HEX4 == 4'h6) ? 7'b0000010 :
		(HEX4 == 4'h7) ? 7'b1111000 :
		(HEX4 == 4'h8) ? 7'b0000000 :
		(HEX4 == 4'h9) ? 7'b0010000 :
		(HEX4 == 4'hA) ? 7'b0001000 :
		(HEX4 == 4'hb) ? 7'b0000011 :
		(HEX4 == 4'hc) ? 7'b1000110 :
		(HEX4 == 4'hd) ? 7'b0100001 :
		(HEX4 == 4'he) ? 7'b0000110 :
		/*HEX4 == 4'hf*/ 7'b0001110 ;
		
	assign OUTHEX3 =
		(1'b0)         ? 7'b1111111 : /* IN == OFF */
		(HEX3 == 4'h0) ? 7'b1000000 :
		(HEX3 == 4'h1) ? 7'b1111001 :
		(HEX3 == 4'h2) ? 7'b0100100 :
		(HEX3 == 4'h3) ? 7'b0110000 :
		(HEX3 == 4'h4) ? 7'b0011001 :
		(HEX3 == 4'h5) ? 7'b0010010 :
		(HEX3 == 4'h6) ? 7'b0000010 :
		(HEX3 == 4'h7) ? 7'b1111000 :
		(HEX3 == 4'h8) ? 7'b0000000 :
		(HEX3 == 4'h9) ? 7'b0010000 :
		(HEX3 == 4'hA) ? 7'b0001000 :
		(HEX3 == 4'hb) ? 7'b0000011 :
		(HEX3 == 4'hc) ? 7'b1000110 :
		(HEX3 == 4'hd) ? 7'b0100001 :
		(HEX3 == 4'he) ? 7'b0000110 :
		/*HEX3 == 4'hf*/ 7'b0001110 ;
		
	assign OUTHEX2 =
		(1'b0)         ? 7'b1111111 : /* IN == OFF */
		(HEX2 == 4'h0) ? 7'b1000000 :
		(HEX2 == 4'h1) ? 7'b1111001 :
		(HEX2 == 4'h2) ? 7'b0100100 :
		(HEX2 == 4'h3) ? 7'b0110000 :
		(HEX2 == 4'h4) ? 7'b0011001 :
		(HEX2 == 4'h5) ? 7'b0010010 :
		(HEX2 == 4'h6) ? 7'b0000010 :
		(HEX2 == 4'h7) ? 7'b1111000 :
		(HEX2 == 4'h8) ? 7'b0000000 :
		(HEX2 == 4'h9) ? 7'b0010000 :
		(HEX2 == 4'hA) ? 7'b0001000 :
		(HEX2 == 4'hb) ? 7'b0000011 :
		(HEX2 == 4'hc) ? 7'b1000110 :
		(HEX2 == 4'hd) ? 7'b0100001 :
		(HEX2 == 4'he) ? 7'b0000110 :
		/*HEX2 == 4'hf*/ 7'b0001110 ;
		
	assign OUTHEX1 =
		(1'b0)         ? 7'b1111111 : /* IN == OFF */
		(HEX1 == 4'h0) ? 7'b1000000 :
		(HEX1 == 4'h1) ? 7'b1111001 :
		(HEX1 == 4'h2) ? 7'b0100100 :
		(HEX1 == 4'h3) ? 7'b0110000 :
		(HEX1 == 4'h4) ? 7'b0011001 :
		(HEX1 == 4'h5) ? 7'b0010010 :
		(HEX1 == 4'h6) ? 7'b0000010 :
		(HEX1 == 4'h7) ? 7'b1111000 :
		(HEX1 == 4'h8) ? 7'b0000000 :
		(HEX1 == 4'h9) ? 7'b0010000 :
		(HEX1 == 4'hA) ? 7'b0001000 :
		(HEX1 == 4'hb) ? 7'b0000011 :
		(HEX1 == 4'hc) ? 7'b1000110 :
		(HEX1 == 4'hd) ? 7'b0100001 :
		(HEX1 == 4'he) ? 7'b0000110 :
		/*HEX1 == 4'hf*/ 7'b0001110 ;
   
	assign OUTHEX0 =
		(1'b0)         ? 7'b1111111 : /* IN == OFF */
		(HEX0 == 4'h0) ? 7'b1000000 :
		(HEX0 == 4'h1) ? 7'b1111001 :
		(HEX0 == 4'h2) ? 7'b0100100 :
		(HEX0 == 4'h3) ? 7'b0110000 :
		(HEX0 == 4'h4) ? 7'b0011001 :
		(HEX0 == 4'h5) ? 7'b0010010 :
		(HEX0 == 4'h6) ? 7'b0000010 :
		(HEX0 == 4'h7) ? 7'b1111000 :
		(HEX0 == 4'h8) ? 7'b0000000 :
		(HEX0 == 4'h9) ? 7'b0010000 :
		(HEX0 == 4'hA) ? 7'b0001000 :
		(HEX0 == 4'hb) ? 7'b0000011 :
		(HEX0 == 4'hc) ? 7'b1000110 :
		(HEX0 == 4'hd) ? 7'b0100001 :
		(HEX0 == 4'he) ? 7'b0000110 :
		/*HEX0 == 4'hf*/ 7'b0001110 ;
  
	assign DBUS = rdData ? {8'b0,HEXDATA} :
					  {BITS{1'bz}};
	
endmodule

module Key(ABUS, DBUS, KEY, WE, INTR, CLK, RESET);
	parameter BITS;
	parameter BASE;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire [3:0] KEY;
	input wire CLK, WE, RESET;
	output wire INTR;
	
	wire selData = (ABUS === BASE);
	wire selCtl = (ABUS === BASE + 4);
	reg [3:0] sample;
	reg [3:0] last_sample;
	reg [3:0] KEYDATA;
	reg [(BITS-1):0] KEYCTRL;
	reg [3:0] clockCount;
	always @ (posedge CLK or posedge RESET) begin 
		if (RESET) begin
			KEYDATA <= 4'h0;
			KEYCTRL <= {(BITS-1){1'b0}};
			last_sample <= 4'h0;
			sample <= 4'h0;
			clockCount <= 4'h0;
		end else begin
			if (selData && !WE) begin
				KEYCTRL[0] <= 0;
			end
			if (clockCount === 4'hF) begin
				if (last_sample === sample && KEYDATA !== sample) begin
					KEYDATA <= sample;
					if (KEYCTRL[0]) begin
						KEYCTRL[1] <= 1;
					end else begin
						KEYCTRL[0] <= 1;
					end
				end
				sample <= ~KEY;
				last_sample <= sample;
				clockCount <= 4'h0;
			end
			clockCount <= clockCount + 4'h1;
			if (selCtl && WE) begin
				if (DBUS[1] === 0) begin
					KEYCTRL[1] <= 0;
				end
				KEYCTRL[4] <= DBUS[4];
			end
		end
	end
	
	assign INTR = KEYCTRL[0] && KEYCTRL[4];
	assign DBUS = (selData && !WE) ? {{(BITS-4){1'b0}},KEYDATA} : 
					  (selCtl && !WE) ? KEYCTRL : 
					  {BITS{1'bz}};
endmodule

module Switch(ABUS, DBUS, SW, WE, INTR, CLK, RESET);
	parameter BITS;
	parameter BASE;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire [9:0] SW;
	input wire CLK, WE, RESET;
	output wire INTR;
	
	wire selData = (ABUS === BASE);
	wire selCtl = (ABUS === BASE + 4);
	reg [9:0] sample;
	reg [9:0] last_sample;
	reg [9:0] SDATA;
	reg [(BITS-1):0] SCTRL;
	reg [11:0] clockCount;
	always @ (posedge CLK or posedge RESET) begin 
		if (RESET) begin
			SDATA <= 10'h0;
			SCTRL <= {(BITS-1){1'b0}};
			sample <= 10'h0;
			last_sample <= 10'h0;
			clockCount <= 12'h0;
		end else begin
			if (selData && !WE) begin
				SCTRL[0] <= 0;
			end
			if (clockCount === 12'hFFF) begin
				if (last_sample === sample && SDATA !== sample) begin
					SDATA <= sample;
					SCTRL[0] <= 1;
				end
				sample <= SW;
				last_sample <= sample;
				clockCount <= 12'h0;
			end
			clockCount <= clockCount + 12'h1;
			if (WE && selCtl) begin
				if (DBUS[1] === 0) begin
					SCTRL[1] <= 0;
				end
				SCTRL[4] <= DBUS[4];
			end
		end
	end
	
	assign INTR = SCTRL[0] && SCTRL[4];
	assign DBUS = (selData && !WE) ? {{(BITS-10){1'b0}},SDATA} : 
					  (selCtl && !WE) ? SCTRL : 
					  {BITS{1'bz}};
endmodule

module Timer(ABUS, DBUS, WE, INTR, CLK, RESET);
	parameter BITS;
	parameter BASE;
	parameter CLOCK_FREQ = 100000000;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire WE, CLK, RESET;
	output wire INTR;
	
	wire selCnt = (ABUS === BASE);
	wire selLim = (ABUS === BASE + 4);
	wire selCtl = (ABUS === BASE + 8); // select TCTL
	
	wire rdLim = (!WE) && selLim;
	wire rdCtl = (!WE) && selCtl;
	wire rdCnt = (!WE) && selCnt;
	
	reg [(BITS - 1):0] TCNT; // current value of counter
	reg [(BITS - 1):0] TLIM; // counter limit
	reg [(BITS - 1):0] TCTL; // control/status reg
	
	reg [(BITS - 1):0] ms_count;
	
	always @ (posedge CLK or posedge RESET) begin
		if (RESET) begin 
			TCNT <= {(BITS - 1){1'b0}};
			TLIM <= {(BITS - 1){1'b0}};
			TCTL <= {(BITS - 1){1'b0}};
			ms_count <= {(BITS-1){1'b0}};
		end else begin 
			if (WE) begin
				if (selCnt) begin
					TCNT <= DBUS;
				end else if (selLim) begin
					TCNT <= 0;
					TLIM <= DBUS;
				end
			
				if (selCtl) begin
					if (DBUS[0] == 0) 
						TCTL[0] <= DBUS[0];	// clear ready bit
					
					if (DBUS[1] == 0) 
						TCTL[1] <= DBUS[1];	// clear overflow bit
					
					TCTL[4] <= DBUS[4]; 	// fill interrupt bit
				end
			end
			
			if ((TLIM != 0) && (TCNT >= TLIM)) begin
				TCNT <= 0;
				if (TCTL[0]) begin
					TCTL[1] <= 1;
				end else begin
					TCTL[0] <= 1;
				end
			end
			ms_count <= ms_count + 1;
			if (ms_count >= CLOCK_FREQ / 1000) begin
				TCNT <= TCNT + 1;
				ms_count <= 0;
			end
		end
	end
	
	assign INTR = TCTL[4] && TCTL[0];
	assign DBUS = rdCtl ? {TCTL} :
					  rdCnt ? {TCNT} :
					  rdLim ? {TLIM} :
					  {BITS{1'bz}};
	
endmodule