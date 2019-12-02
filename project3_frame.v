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
//  parameter IMEMINITFILE = "in_class_assignment_11_14/test_intr.mif";
//  parameter IMEMINITFILE = "benschau/test_interrupt.mif";
//  parameter IMEMINITFILE = "benschau/xmax.mif";
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
  
  //Interrupt control signals:
  wire IRQ;
  wire RETI;
  wire intr_key;
  wire intr_sw;
  wire intr_timer;
  
  //System Registers:
  reg [DBITS-1:0] PCS;
  reg [DBITS-1:0] IHA;
  reg [DBITS-1:0] IRA;
  reg [DBITS-1:0] IDN;

  //*** FETCH STAGE ***//
  // The PC register and update logic
  wire [DBITS-1:0] pcplus_FE;
  wire [DBITS-1:0] pcpred_FE;
  wire [DBITS-1:0] inst_FE_w;
  wire stall_pipe;
  wire mispred_EX_w;
  wire [DBITS-1:0] pcgood_EX_w;

  reg [DBITS-1:0] PC_FE;
  reg [INSTBITS-1:0] inst_FE;
  // I-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0] imem [IMEMWORDS-1:0];

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
		if (mispred_EX_w || IRQ || RETI) begin
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
  wire [DBITS-1:0] output_value_MEM_w;

  wire is_alui_operation;
  wire is_op2_ID;
  wire is_op1_ID;
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

  reg is_sys_inst_ID;
  reg is_reti_ID;

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

  assign is_sys_instr_ID_w = (op1_ID_w == OP1_SYS);
  assign is_wr_sys_ID_w = is_sys_instr_ID_w && (op2_ID_w == OP2_WSR);
  assign is_rd_sys_ID_w = is_sys_instr_ID_w && (op2_ID_w == OP2_RSR);
  assign is_reti_ID_w = is_sys_instr_ID_w && (op2_ID_w == OP2_RETI);
  
  // Read register values
  assign regval1_ID_w = regs[rs_ID_w];
  assign regval2_ID_w = regs[rt_ID_w];

  // Sign extension
  SXT mysxt (.IN(imm_ID_w), .OUT(sxt_imm_ID_w));

  // Specify control signals such as is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, etc.
  // You may add or change control signals if needed
  assign is_br_ID_w = (op1_ID_w == OP1_BEQ
							|| op1_ID_w == OP1_BLT
							|| op1_ID_w == OP1_BLE
							|| op1_ID_w == OP1_BNE) ? 1 : 0;
  assign is_jmp_ID_w = (op1_ID_w == OP1_JAL) ? 1 : 0;
  assign rd_mem_ID_w = (op1_ID_w == OP1_LW) ? 1 : 0; // are we reading from memory?
  assign wr_mem_ID_w = (op1_ID_w == OP1_SW) ? 1 : 0; // are we writing to memory?
  assign wr_reg_ID_w = (is_op2_ID				 			// any OP2 writes to a register
							|| is_alui_operation 	 			// any alui operation writes to a register
							|| op1_ID_w == OP1_JAL	 			// We write to rt in JAL
							|| op1_ID_w == OP1_LW         // we write to rt in LW
              || is_rd_sys_ID_w) ? 1 : 0;   // we write to regular registers in an RSR instruction

  //wregno is the register number that will be written to
  // TODO: we must sign extend the system regnos, but still, we have to make sure forwarding doesn't work
  assign wregno_ID_w = (is_op2_ID || is_sys_instr_ID_w) ? rd_ID_w : rt_ID_w;

  // concatenates everything together to be put in buffers/registers later {4:0}
  assign ctrlsig_ID_w = {is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};

  // Stalling
  wire read_rt;
  assign read_rt = !(is_jmp_ID_w || rd_mem_ID_w|| is_alui_operation || is_sys_instr_ID_w); // rt is only read in these instructions
  // Check if one of the registers this instruction depends on is going to be written to by the instructions 
  // in EX or MEM. All instructions depend on Rs - only stall on write to Rt if this instruction actually
  // depends on it.
  wire rs_ex_hazard_w;
  wire rs_mem_hazard_w;
  wire rt_ex_hazard_w;
  wire rt_mem_hazard_w;
  
  assign rs_ex_hazard_w = (wregno_ID == rs_ID_w && ctrlsig_ID[0]); //==
  assign rs_mem_hazard_w = (wregno_EX == rs_ID_w && ctrlsig_EX[0]);
  assign rt_ex_hazard_w = (read_rt && wregno_ID == rt_ID_w  && ctrlsig_ID[0]);
  assign rt_mem_hazard_w = (read_rt && wregno_EX == rt_ID_w && ctrlsig_EX[0]);
  assign stall_pipe = (rs_ex_hazard_w || rt_ex_hazard_w) && (ctrlsig_ID[2] || is_rd_sys_EX_w); // if rd_mem or is rsr in ex
  
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
			regval1_ID  <= output_value_MEM_w;
		end else begin
			regval1_ID  <=  regval1_ID_w;  
		end
		
		if (rt_ex_hazard_w) begin
			regval2_ID  <= aluout_EX_r;
		end else if (rt_mem_hazard_w) begin
			regval2_ID  <= output_value_MEM_w;
		end else begin
			regval2_ID  <= regval2_ID_w;
		end
	 end
	end

  //*** AGEN/EXEC STAGE ***//
  wire is_br_EX_w;
  wire is_jmp_EX_w;

  reg [INSTBITS-1:0] inst_EX; /* This is for debugging */
  reg br_cond_EX;
  reg [2:0] ctrlsig_EX;
  // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
  reg signed [DBITS-1:0] aluout_EX_r;
  reg [DBITS-1:0] aluout_EX;
  reg [DBITS-1:0] regval1_EX;
  reg [DBITS-1:0] regval2_EX;
  reg [OP1BITS-1:0] op1_EX;
  reg [OP2BITS-1:0] op2_EX;
  reg [DBITS-1:0] next_pc_EX;

  wire is_rd_sys_EX_w;
  wire is_wr_sys_EX_w;
  reg is_rd_sys_EX;
  reg is_wr_sys_EX;
  reg is_sys_inst_EX;
  reg is_reti_EX;

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
  
  assign is_rd_sys_EX_w = is_sys_inst_ID && (op2_ID == OP2_RSR);
  assign is_wr_sys_EX_w = is_sys_inst_ID && (op2_ID == OP2_WSR);

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
      aluout_EX	<= {DBITS{1'b0}};
      wregno_EX	<= {REGNOBITS{1'b0}};
      ctrlsig_EX 	<= 3'h0;
    regval1_EX  <= {DBITS{1'b0}};
		regval2_EX	<= {DBITS{1'b0}};
		is_sys_inst_EX <= 1'b0;
		is_wr_sys_EX 	<= 1'b0;
      is_rd_sys_EX 	<= 1'b0;
      is_reti_EX 	<= 1'b0;
		next_pc_EX    <= {DBITS{1'b0}};
	 end else if (IRQ || RETI) begin // Flush out EX state after branch or Jump
		inst_EX   <= {INSTBITS{1'b0}};
      aluout_EX <= {DBITS{1'b0}};
      wregno_EX <= {REGNOBITS{1'b0}};
      ctrlsig_EX  <= 3'h0;
    regval1_EX  <= {DBITS{1'b0}};
    regval2_EX  <= {DBITS{1'b0}};
    is_sys_inst_EX <= 1'b0;
    is_wr_sys_EX   <= 1'b0;
      is_rd_sys_EX   <= 1'b0;
      is_reti_EX  <= 1'b0;
    next_pc_EX    <= {DBITS{1'b0}};
    end else begin
		inst_EX	 	<= inst_ID;
      aluout_EX	<= aluout_EX_r;
      wregno_EX	<= wregno_ID;
      ctrlsig_EX 	<= {ctrlsig_ID[2], ctrlsig_ID[1], ctrlsig_ID[0]}; // MEM stage needs: read mem, write mem, and write reg
		regval1_EX	<= regval1_ID;
		regval2_EX 	<= regval2_ID; // pass this along for SW
		is_sys_inst_EX <= is_sys_inst_ID;
		is_wr_sys_EX 	<= is_wr_sys_EX_w;
      is_rd_sys_EX 	<= is_rd_sys_EX_w;
      is_reti_EX 	<= is_reti_ID;
		next_pc_EX    <= mispred_EX_w ? pcgood_EX_w : PC_ID;
    end
  end


  //*** MEM STAGE ***//

  wire [DBITS-1:0] abus;
  wire [DBITS-1:0] dbus;
  wire we;

  wire [DBITS-1:0] memaddr_MEM_w;
  wire [DBITS-1:0] rd_val_MEM_w;

  reg [INSTBITS-1:0] inst_MEM; /* This is for debugging */
  reg [DBITS-1:0] regval_MEM;
  reg ctrlsig_MEM;
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
  assign we = wr_mem_MEM_w;

  // Interupts
  assign IRQ = inst_EX != NOP && !is_sys_inst_EX && PCS[0] && (intr_key || intr_sw || intr_timer);
  assign RETI = is_reti_EX;

  // System Latch
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      PCS <= {DBITS{1'b0}};
      IHA <= {{DBITS-12{1'b0}},{12'h200}};
      IRA <= {DBITS{1'b0}};
      IDN <= {DBITS{1'b0}};
   end else begin
    if (is_wr_sys_EX) begin
      case (wregno_EX)
        4'h1: IRA <= regval1_EX;
        4'h2: IHA <= regval1_EX;
        4'h3: IDN <= regval1_EX;
        4'h4: PCS <= regval1_EX;
      endcase
    end else if (IRQ) begin 
      IRA <= next_pc_EX;
      PCS[1] <= PCS[0];
      PCS[0] <= 0;
      IDN <= intr_timer ? 32'h1 :
           intr_key   ? 32'h2 :
           intr_sw   ? 32'h3 :
                        32'h0 ;
    end else if (RETI) begin
      PCS[0] <= PCS[1];
    end
   end
  end

  wire [REGNOBITS-1:0] sys_regno_MEM_w;
  wire [DBITS-1:0] sys_regval_MEM_w;
  assign sys_regno_MEM_w = inst_EX[7:4];
  assign sys_regval_MEM_w = 
                 sys_regno_MEM_w == 4'h1 ? IRA :
                 sys_regno_MEM_w == 4'h2 ? IHA :
                 sys_regno_MEM_w == 4'h3 ? IDN :
                 sys_regno_MEM_w == 4'h4 ? PCS :
                 {DBITS{1'b0}};
  
  assign output_value_MEM_w = rd_mem_MEM_w ? rd_val_MEM_w : 
              is_rd_sys_EX ? sys_regval_MEM_w :
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
      regval_MEM  <= output_value_MEM_w;
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

   LEDR_DEVICE #(.BITS(DBITS), .BASE(ADDRLEDR)) ledr(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .WE(we),
	 .OUT(LEDR),
	 .CLK(clk),
	 .RESET(reset)
  );
  
   HEX_DEVICE #(.BITS(DBITS), .BASE(ADDRHEX)) hex(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .WE(we),
   .HEX({HEX5, HEX4, HEX3, HEX2, HEX1, HEX0}),
	 .CLK(clk),
	 .RESET(reset)
  );
  
  KEY_DEVICE #(.BITS(DBITS), .BASE(ADDRKEY)) key(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .KEY(KEY),
	 .WE(we),
	 .INTR(intr_key),
	 .CLK(clk),
	 .RESET(reset)
  );
  
  SWITCH_DEVICE #(.BITS(DBITS), .BASE(ADDRSW)) switch(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .SW(SW),
	 .WE(we),
	 .INTR(intr_sw),
	 .CLK(clk),
	 .RESET(reset)
  );
  
  TIMER_DEVICE #(.BITS(DBITS), .BASE(ADDRTIMER)) timer(
	 .ABUS(abus), 
	 .DBUS(dbus),
	 .WE(we),
	 .INTR(intr_timer),
	 .CLK(clk),
	 .RESET(reset)
  );
  
endmodule

module SevenSeg(output [6:0] OUT, input [3:0] IN, input OFF);
  assign OUT =
    (OFF)        ? 7'b1111111 :
    (IN == 4'h0) ? 7'b1000000 :
    (IN == 4'h1) ? 7'b1111001 :
    (IN == 4'h2) ? 7'b0100100 :
    (IN == 4'h3) ? 7'b0110000 :
    (IN == 4'h4) ? 7'b0011001 :
    (IN == 4'h5) ? 7'b0010010 :
    (IN == 4'h6) ? 7'b0000010 :
    (IN == 4'h7) ? 7'b1111000 :
    (IN == 4'h8) ? 7'b0000000 :
    (IN == 4'h9) ? 7'b0010000 :
    (IN == 4'hA) ? 7'b0001000 :
    (IN == 4'hb) ? 7'b0000011 :
    (IN == 4'hc) ? 7'b1000110 :
    (IN == 4'hd) ? 7'b0100001 :
    (IN == 4'he) ? 7'b0000110 :
    /*IN == 4'hf*/ 7'b0001110 ;
endmodule

module SXT(IN, OUT);
  parameter IN_BITS = 16;
  parameter OUT_BITS = 32;

  input  [IN_BITS-1:0] IN;
  output [OUT_BITS-1:0] OUT;

  assign OUT = {{(OUT_BITS-IN_BITS){IN[IN_BITS-1]}}, IN};
endmodule

module LEDR_DEVICE(ABUS, DBUS, WE, OUT, CLK, RESET);
	parameter BASE;
	parameter BITS;

	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire WE, CLK, RESET;
	output wire [9:0] OUT;

	reg [9:0] LEDR_STATUS;
	wire is_data_address = (ABUS === BASE);
	wire is_read_data = (!WE) && is_data_address; 

	always @ (posedge CLK or posedge RESET) begin
		if (RESET) begin
			LEDR_STATUS <= 10'd0;
		end else begin // Writing to LEDR:
			if (WE && is_data_address) begin
				LEDR_STATUS <= DBUS[9:0];
			end
		end
	end

  // Reading from LEDR onto dbus:
	assign OUT = LEDR_STATUS;
	assign DBUS = is_read_data ? {22'b0,LEDR_STATUS} :
					  {BITS{1'bz}};
endmodule


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

module KEY_DEVICE(ABUS, DBUS, KEY, WE, INTR, CLK, RESET);
	parameter BASE;
	parameter BITS;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire [3:0] KEY;
	input wire CLK, WE, RESET;
	output wire INTR;
	
	wire is_DATA_address = (ABUS === BASE);
	wire is_CTL_address = (ABUS === BASE + 4);

	reg [(BITS-1):0] KEY_CTRL;
	reg [3:0] KEY_STATUS;
	reg [3:0] current_poll;
	reg [3:0] previous_poll;
	reg [3:0] count;

	always @ (posedge CLK or posedge RESET) begin 
		if (RESET) begin
			KEY_STATUS <= 4'h0;
			KEY_CTRL <= {(BITS-1){1'b0}};
			current_poll <= 4'h0;
			previous_poll <= 4'h0;
			count <= 4'h0;
		end else begin
			if (is_DATA_address && !WE) begin
				KEY_CTRL[0] <= 0;
			end

			if (count === 4'hF) begin
				if (previous_poll === current_poll && KEY_STATUS !== current_poll) begin
					KEY_STATUS <= current_poll;
					if (KEY_CTRL[0]) begin
						KEY_CTRL[1] <= 1;
					end else begin
						KEY_CTRL[0] <= 1;
					end
				end
				current_poll <= ~KEY;
				previous_poll <= current_poll;
				count <= 4'h0;
			end
			count <= count + 4'h1;
			if (is_CTL_address && WE) begin
				if (DBUS[1] === 0) begin
					KEY_CTRL[1] <= 0;
				end
				KEY_CTRL[4] <= DBUS[4];
			end
		end
	end
	
  // Reading from Key onto dbus:
	assign INTR = KEY_CTRL[0] && KEY_CTRL[4];
	assign DBUS = (is_DATA_address && !WE) ? {{(BITS-4){1'b0}},KEY_STATUS} : 
					  (is_CTL_address && !WE) ? KEY_CTRL : 
					  {BITS{1'bz}};
endmodule

module SWITCH_DEVICE(ABUS, DBUS, SW, WE, INTR, CLK, RESET);
	parameter BASE;
	parameter BITS;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire [9:0] SW;
	input wire CLK, WE, RESET;
	output wire INTR;
	
	wire is_DATA_address = (ABUS === BASE);
	wire is_CTL_address = (ABUS === BASE + 4);

	reg [9:0] SW_STATUS;
	reg [(BITS-1):0] SW_CTRL;
	reg [9:0] current_poll;
	reg [9:0] previous_poll;
	reg [11:0] count;

	always @ (posedge CLK or posedge RESET) begin 
		if (RESET) begin
			SW_STATUS <= 10'h0;
			SW_CTRL <= {(BITS-1){1'b0}};
			current_poll <= 10'h0;
			previous_poll <= 10'h0;
			count <= 12'h0;
		end else begin
			if (is_DATA_address && !WE) begin
				SW_CTRL[0] <= 0;
			end
			if (count === 12'hFFF) begin
				if (previous_poll === current_poll && SW_STATUS !== current_poll) begin
					SW_STATUS <= current_poll;
					SW_CTRL[0] <= 1;
				end
				current_poll <= SW;
				previous_poll <= current_poll;
				count <= 12'h0;
			end
			count <= count + 12'h1;
			if (WE && is_CTL_address) begin
				if (DBUS[1] === 0) begin
					SW_CTRL[1] <= 0;
				end
				SW_CTRL[4] <= DBUS[4];
			end
		end
	end
	
  // Reading from SW onto dbus:
	assign INTR = SW_CTRL[0] && SW_CTRL[4];
	assign DBUS = (is_DATA_address && !WE) ? {{(BITS-10){1'b0}},SW_STATUS} : 
					  (is_CTL_address && !WE) ? SW_CTRL : 
					  {BITS{1'bz}};
endmodule

module TIMER_DEVICE(ABUS, DBUS, WE, INTR, CLK, RESET);
	parameter BASE;
	parameter BITS;
	// parameter CLOCK_FREQ = 100000000;
	parameter CLOCK_FREQ = 50000000;
	
	input wire [(BITS-1):0] ABUS;
	inout wire [(BITS-1):0] DBUS;
	input wire WE, CLK, RESET;
	output wire INTR;
	
	wire is_CNT_address = (ABUS === BASE);
	wire is_LIM_address = (ABUS === BASE + 4);
	wire is_CTL_address = (ABUS === BASE + 8); // timer control address
	
	wire is_LIM_read = (!WE) && is_LIM_address;
	wire is_CTL_read = (!WE) && is_CTL_address;
	wire is_CNT_read = (!WE) && is_CNT_address;
	
	reg [(BITS - 1):0] TCNT; // current value of counter
	reg [(BITS - 1):0] TLIM; // counter limit
	reg [(BITS - 1):0] TCTL; // control/status reg
	reg [(BITS - 1):0] counter; // counts number of elapsed milliseconds
	
	always @ (posedge CLK or posedge RESET) begin
		if (RESET) begin 
			TCNT <= {(BITS - 1){1'b0}};
			TLIM <= {(BITS - 1){1'b0}};
			TCTL <= {(BITS - 1){1'b0}};
			counter <= {(BITS-1){1'b0}};
		end else begin 
			if (WE) begin
				if (is_CNT_address) begin
					TCNT <= DBUS;
				end else if (is_LIM_address) begin
					TCNT <= 0;
					TLIM <= DBUS;
				end
			
				if (is_CTL_address) begin
					if (DBUS[0] == 0) 
						TCTL[0] <= DBUS[0];	// disable the ready bit
					
					if (DBUS[1] == 0) 
						TCTL[1] <= DBUS[1];	// disable the overflow bit
					
					TCTL[4] <= DBUS[4]; 	// copy over to interrupt bit
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
			counter <= counter + 1;
			if (counter >= CLOCK_FREQ / 1000) begin
				TCNT <= TCNT + 1;
				counter <= 0;
			end
		end
	end
	
  // Reading from Timer onto dbus:
	assign INTR = TCTL[4] && TCTL[0];
	assign DBUS = is_CTL_read ? {TCTL} :
					  is_CNT_read ? {TCNT} :
					  is_LIM_read ? {TLIM} :
					  {BITS{1'bz}};
endmodule