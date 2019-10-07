`define ENABLE_EXCEPTIONS
//`define ENABLE_IDS
`define ENABLE_TIMER

/*
module RegisterSet(
    input clk, 
    input we,
    input [5:0] wa,
    input [31:0] wd,
    input [5:0] ra1,
    input [5:0] ra2,
    output reg [31:0] rd1,
    output reg [31:0] rd2
);
    reg [31:0] regs [0:63];

    initial begin
        regs[0] <= 0;
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= regs[ra1];
        rd2 <= regs[ra2];
    end
endmodule


// IGLOO2 cannot initialize block RAM at statup
module RegisterSetMicrosemi(
    input clk, 
    input we,
    input [5:0] wa,
    input [31:0] wd,
    input [5:0] ra1,
    input [5:0] ra2,
    output reg [31:0] rd1,
    output reg [31:0] rd2
);
    reg [31:0] regs [0:63];

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= ra1 ? regs[ra1] : 0;
        rd2 <= ra2 ? regs[ra2] : 0;
    end
endmodule


module RegisterSetGrubbyMicrosemi(
    input clk, 
    input we,
    input [5:0] wa,
    input [31:0] wd,
    input wg,
    input [5:0] ra1,
    input [5:0] ra2,
    output reg [31:0] rd1,
    output reg rg1,
    output reg [31:0] rd2,
    output reg rg2
);
    reg [35:0] regs [0:63];

    always @(posedge clk) begin
        if (we) regs[wa] <= {3'b0, wg, wd};
        rd1 <= ra1 ? regs[ra1][31:0] : 0;
        rg1 <= ra1 ? regs[ra1][32] : 0;
        rd2 <= ra2 ? regs[ra2][31:0] : 0;
        rg2 <= ra1 ? regs[ra2][32] : 0;
    end
endmodule


module RegisterSetGrubby(
    input clk, 
    input we,
    input [5:0] wa,
    input [31:0] wd,
    input wg,
    input [5:0] ra1,
    input [5:0] ra2,
    output reg [31:0] rd1,
    output reg rg1,
    output reg [31:0] rd2,
    output reg rg2
);
    reg [31:0] regs [0:63];
    reg grubby[0:63];

    initial begin
        regs[0] <= 0;
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= regs[ra1];
        rd2 <= regs[ra2];
    end

    always @(posedge clk) begin
        if (we) grubby[wa] <= wg;
        rg1 <= grubby[ra1];
        rg2 <= grubby[ra2];
    end

endmodule
*/



module Pipeline #(
    parameter [31:0] START_PC = 0
) (
    input  clk,
    input  rstn,

    input  irq_timer,
    output retired,

    output csr_read,            // can be ignored if there are no side-effects
    output [2:0] csr_modify,    // 01=write 10=set 11=clear
    output [31:0] csr_wdata,
    output [11:0] csr_addr,
    input [31:0] csr_rdata,
    input csr_valid,            // CSR addr is valid, not necessarily rdata

    output mem_valid,
    output mem_write,
    output [3:0] mem_wmask,
    output [31:0] mem_wdata,
    output mem_wgrubby,
    output [31:0] mem_addr,
    input [31:0] mem_rdata,
    input mem_rgrubby
);
    localparam integer WORD_WIDTH = 32;

    localparam [5:0] REG_CSR_MTVEC    = 6'b100101;
    localparam [5:0] REG_CSR_MSCRATCH = 6'b110000;
    localparam [5:0] REG_CSR_MEPC     = 6'b110001;
    localparam [5:0] REG_CSR_MCAUSE   = 6'b110010;
    localparam [5:0] REG_CSR_MTVAL    = 6'b110011;



// ---------------------------------------------------------------------
// real registers
// ---------------------------------------------------------------------


    // fetch
    reg [WORD_WIDTH-1:0] f_PC;

    // decode
    reg [31:0] d_Insn;
    reg [5:0] d_RdNo1;
    reg [5:0] d_RdNo2;

    reg [WORD_WIDTH-1:0] d_PC;
    reg [31:0] d_DelayedInsn;
    reg d_SaveFetch;
    reg d_Bubble;

    reg [5:0] d_MultiCycleCounter;
    reg e_StartMC;
    reg [WORD_WIDTH:0] q_MulB;
    reg [2*WORD_WIDTH+1:0] q_MulC;
    reg e_FromMul;
    reg e_FromMulH;
    reg e_MulASigned;
    reg e_MulBSigned;

    reg [WORD_WIDTH-1:0] q_DivQuot;
    reg [WORD_WIDTH:0] q_DivRem;
    reg e_FromDivOrRem;
    reg e_SelRemOrDiv;
    reg e_DivSigned;
    reg e_DivResultSign;
    reg m_SelDivOrMul;

    // execute
    reg e_InsnJALR;
    reg e_InsnBEQ;
    reg e_InsnBLTorBLTU;
    reg e_InvertBranch;
    reg [1:0] e_SelLogic;
    reg e_EnShift;
    reg e_ShiftArith;
    reg e_ReturnPC;
    reg e_ReturnUI;
    reg e_LUIorAUIPC;
    reg e_InsnJALorFENCEI;

    reg e_SetCond;
    reg e_LTU;
    reg e_SelSum;
    reg e_MemAccess;
    reg e_MemWr;
    reg [1:0] e_MemWidth;

    reg [WORD_WIDTH-1:0] e_A;
    reg [WORD_WIDTH-1:0] e_B;
    reg [WORD_WIDTH-1:0] e_Imm;
    reg [WORD_WIDTH-1:0] e_PCImm;
    reg e_AGrubby;
    reg e_BGrubby;

    reg e_Carry;
    reg e_WrEn;
    reg [5:0] e_WrNo;

    reg  e_InsnBit14;
    wire e_ShiftRight      = e_InsnBit14;
    wire e_MemUnsignedLoad = e_InsnBit14;
    wire e_CsrSelImm       = e_InsnBit14;

    // mem stage
    reg m_Kill; // to decode and execute stage
    reg w_Kill;
    reg m_WrEn;
    reg [5:0] m_WrNo;
    reg [WORD_WIDTH-1:0] m_WrData;
    reg [4:0] m_MemByte;
    reg [2:0] m_MemSign;

    // write back
    reg w_WrEn;
    reg [5:0] w_WrNo;
    reg [WORD_WIDTH-1:0] w_WrData;
    reg w_WrGrubby;


`ifdef ENABLE_EXCEPTIONS
    // exceptions
    reg [WORD_WIDTH-1:0] e_ExcWrData2;
    reg [WORD_WIDTH-1:0] m_ExcWrData;
    reg e_ExcUser;
    reg m_ExcUser;
    reg e_ExcGrubbyInsn;
    reg m_ExcGrubbyInsn;
    reg d_ExcJump;
    reg e_ExcJump;
    reg f_ExcGrubbyJump;
    reg d_ExcGrubbyJump;
    reg e_ExcGrubbyJump;
    reg m_ExcMem;
    reg w_ExcMem;
//    reg e_EBREAKorECALL;
    reg m_MemWr;
    reg m_WriteMCAUSE;
    reg m_WriteMTVAL;
    reg [4:0] e_Cause1;
    reg [4:0] m_Cause;

    // CSRs for exceptions
    reg [WORD_WIDTH-1:0] m_CsrUpdate;
    reg [1:0] e_CsrOp;
    reg [1:0] m_CsrOp;

    reg e_CsrFromReg;
    reg m_CsrFromReg;

    reg w_CsrFromReg;
    reg [WORD_WIDTH-1:0] m_CsrModified;
`endif



`ifdef ENABLE_COUNTER
    // CSRs for counters
    reg        e_CarryCYCLE;
    reg [32:0] e_CounterCYCLE;
    reg [31:0] e_CounterCYCLEH;
    reg        e_CarryINSTRET;
    reg [32:0] e_CounterINSTRET;
    reg [31:0] e_CounterINSTRETH;
    reg        e_CsrSelHighWord;
    reg        m_CsrSelHighWord;
`endif

    reg        e_CsrFromExt;
    reg        e_CsrRead;
    reg        m_CsrRead;
    reg        m_CsrValid;
    reg [WORD_WIDTH-1:0] m_CsrRdData;



`ifdef ENABLE_TIMER
    reg        f_MModeIntEnable;        // mstatus.mie
    reg        f_MModePriorIntEnable;   // mstatus.mpie
    reg        d_TimerInt;
    reg        e_TimerInt;

    reg [2:0]  e_CsrModify;
    reg [31:0] e_CsrWData;
    reg [11:0] e_CsrAddr;
`endif

    reg DecodeGrubbyInsn;
    reg d_GrubbyInsn;



// ---------------------------------------------------------------------
// combinational circuits
// ---------------------------------------------------------------------




    // fetch

`ifdef ENABLE_TIMER
    reg MModeIntEnable;
    reg MModePriorIntEnable;
    reg TimerInt;
`endif


    reg [31:0] Insn;
    always @* begin
        DecodeGrubbyInsn <= 0;
`ifdef ENABLE_TIMER
        TimerInt <= 0;
        MModeIntEnable <= f_MModeIntEnable;
        MModePriorIntEnable <= f_MModePriorIntEnable;
        if (e_CsrAddr==12'h300) begin  // mstatus
            case (e_CsrModify)
                3'b001: begin // write
                    MModeIntEnable      <= e_CsrWData[3];
                    MModePriorIntEnable <= e_CsrWData[7];
                end
                3'b010: begin // set
                    MModeIntEnable      <= f_MModeIntEnable      | e_CsrWData[3];
                    MModePriorIntEnable <= f_MModePriorIntEnable | e_CsrWData[7];
                end
                3'b011: begin // clear
                    MModeIntEnable      <= f_MModeIntEnable      & ~e_CsrWData[3];
                    MModePriorIntEnable <= f_MModePriorIntEnable & ~e_CsrWData[7];
                end
            endcase
        end
`endif
        if (Bubble | ExcJump | f_ExcGrubbyJump | ExcGrubbyInsn) begin
            //if (~ExcUser & ~ExcJump & ~MemAccess & ~InsnMRET)
            if (CsrPart & ~f_ExcGrubbyJump)
// better: SysOpcode & CsrPart 

                Insn <= {7'b0000000, 5'b00000,
                         vCsrTranslate,
//                         3'b110, d_Insn[11:7], 7'b0110011}; // OR
                         3'b110, vCsrTranslate, 7'b0110011}; // OR
            else if (~ExcUser & ~ExcGrubbyInsn & ~ExcJump & ~f_ExcGrubbyJump 
                & ~MemAccess & ~InsnMRET) begin // WFI
//                Insn <= {7'b0000000, 5'b00000, 5'b00000,
//                         3'b110, 5'b00000, 7'b0110011}; // OR X0, X0, X0

                Insn <= {12'hbff, 5'b00001, 3'b101, 5'b00000, 7'b1110011}; 
                    // CSRWI 0xbff, 1 to signal simulation end
                //Insn <= 32'hbff0d073

            end else begin
                Insn <= {7'b0000000, 5'b00000, 
                         (InsnMRET ? REG_CSR_MEPC[4:0] : REG_CSR_MTVEC[4:0]), 
                         3'b110, 5'b00000, 7'b1100111}; // JALR
                if (InsnMRET) begin
                    MModeIntEnable <= f_MModePriorIntEnable;
                    MModePriorIntEnable <= 1;
                end else begin // exception
                    MModeIntEnable <= 0;
                    MModePriorIntEnable <= f_MModeIntEnable;
                end
                // KNOWN BUG:
                // If irq_timer ist still 1 when the MRET restores MIE to 1,
                // d_TimerInt will be killed because it is in the delay slots
                // of the MRET instructions.
            end
        end
        else if (MULDIVOpcode & ~MultiCycle & ~m_Kill)
            // start of multicycle execution
            // the next instruction is currently beeing fetched and must
            // be overwritten.
            Insn <= 32'h13; // NOP
`ifdef ENABLE_TIMER
        else if (irq_timer & f_MModeIntEnable & ~MultiCycle) begin
            // not the absoultely correct priority, but works
            TimerInt <= 1;
            MModeIntEnable <= 0;
            MModePriorIntEnable <= f_MModeIntEnable;
            Insn <= {7'b0000000, 5'b00000,
                     REG_CSR_MTVEC[4:0],
                     3'b110, 5'b00000, 7'b1100111}; // JALR
        end
`endif
        else if ((d_Bubble | d_SaveFetch) & ~m_Kill)
            Insn <= d_DelayedInsn;
        else begin
            DecodeGrubbyInsn <= mem_rgrubby;
            Insn <= mem_rdata;
        end
    end
    wire RdNo1Aux = Bubble | ExcJump | f_ExcGrubbyJump | ExcGrubbyInsn
                           | (irq_timer & f_MModeIntEnable);





    // decode


    wire [WORD_WIDTH-1:0] ImmI = {{21{d_Insn[31]}}, d_Insn[30:20]};


    //                               31|30..12|11..5 | 4..0
    // ImmI for JALR  (opcode 11011) 31|  31  |31..25|24..20
    // ImmI for load  (opcode 00000) 31|  31  |31..25|24..20
    // ImmS for store (opcode 01000) 31|  31  |31..25|11..7
    // ImmU for LUI   (opcode 01101) 31|30..12|   ?  |  ?
    //      for CSR   (opcode 11100) ? |   ?  |21..15|  ?

    // Optimisation: For LUI the lowest 12 bits must not be set correctly to 0,
    // since ReturnImm clears them in the ALU.
    // In fact, this should reduce LUT-size and not harm clock rate, but it does.
    // TRY: check later in more complex design
    wire [WORD_WIDTH-1:0] ImmISU = { // 31 LE
        d_Insn[31],                                                     // 31
        (d_Insn[4] & d_Insn[2]) ? d_Insn[30:12] : {19{d_Insn[31]}},     // 30..12
        d_Insn[4] ? d_Insn[21:15] : d_Insn[31:25],                      // 11..5
        (d_Insn[6:5]==2'b01) ? d_Insn[11:7] : d_Insn[24:20]};           // 4..0

    //                                31|30..20|19..13|12|11|10..5 | 4..1 |0
    // ImmB for branch (opcode 11000) 31|  31  |  31  |31| 7|30..25|11..8 |-
    // ImmJ for JAL    (opcode 11011) 31|  31  |19..13|12|20|30..25|24..21|-
    // ImmU for AUIPC  (opcode 00101) 31|30..20|19..13|12| -|   -  |   -  |-
    //  4 for  FENCE.I (opcode 00011)                   -|         | --X- |-
    wire [WORD_WIDTH-1:0] ImmBJU = { // 30 LE
        d_Insn[31],                                                     // 31
        d_Insn[4] ? d_Insn[30:20] : {11{d_Insn[31]}},                   // 30..20
        d_Insn[2] ? d_Insn[19:13] : {7{d_Insn[31]}},                    // 19..13
        d_Insn[2] ? ((d_Insn[4] | d_Insn[5]) & d_Insn[12]) : d_Insn[31], // 12
        ~d_Insn[4] & (d_Insn[2] ? d_Insn[20] : d_Insn[7]),              // 11
        d_Insn[4] ? 6'b000000 : d_Insn[30:25],                          // 10..5
        //{4{~d_Insn[4]}} & (d_Insn[2] ? d_Insn[24:21] : d_Insn[11:8]),   // 4..1
        d_Insn[4] ? 4'b0000 : (d_Insn[2] ? (d_Insn[5] ? d_Insn[24:21] : 4'b0010) : d_Insn[11:8]),
        1'b0};                                                          // 0

    wire [WORD_WIDTH-1:0] PCImm = d_PC + ImmBJU;



    // LUT4 at level 1

    wire InvertBranch   =  d_Insn[6] & d_Insn[12]; 
        // set for BNE or BGE or BGEU, but not for SLT..SLTIU
    wire LTU = d_Insn[6] ? d_Insn[13] : d_Insn[12];
        // select unsigned or signed comparison
        //     funct3 opcode  LTU InvertBranch
        // BLT    100 1100011  0   0
        // BGE    101 1100011  0   1
        // BLTU   110 1100011  1   0
        // BGEU   111 1100011  1   1
        // SLTI   010 0010011  0   0
        // SLTIU  011 0010011  1   0
        // SLTI   010 0110011  0   0
        // SLTIU  011 0110011  1   0
        //         ^^ ^
        // for all other opcodes, LTU and InverBranch don't mind


    wire BranchOpcode   = (d_Insn[6:3]==4'b1100);
    wire UpperOpcode    = ~d_Insn[6] && d_Insn[4:2]==3'b101;
    wire ArithOpcode    = ~d_Insn[6] && d_Insn[4:2]==3'b100;
    wire MemAccess      = ~d_Insn[6] && d_Insn[4:2]==3'b000; // ST or LD
    wire SysOpcode      =  d_Insn[6] && d_Insn[4:2]==3'b100;
    wire PrivOpcode     =  d_Insn[5] && (d_Insn[14:12]==0);
    wire InsnJALorJALR  =  d_Insn[6:4]==3'b110 && d_Insn[2];
    wire MRETOpcode     = (d_Insn[23:20]==4'b0010);

    wire SUBorSLL       =  d_Insn[13] | d_Insn[12] | (d_Insn[5] & d_Insn[30]);
    wire SUBandSLL      = ~d_Insn[14] & ~d_Insn[6] & d_Insn[4];
    wire PartBranch     = (d_Insn[6:4]==3'b110);
    wire LowPart        = (d_Insn[3:0]==4'b0011);
    wire CsrPart        = (d_Insn[6] & d_Insn[5] & (d_Insn[13] | d_Insn[12]));
    wire MULDIVPart     = d_Insn[5] & (d_Insn[31:25]==7'b0000001);

    wire vMemOrSys      = (d_Insn[6]==d_Insn[4]) & ~d_Insn[3] & ~d_Insn[2];
        // CAUTION: also true for opcode 1010011 (OP-FP, 32 bit floating point) 

    // LUT4 at level 2



    // possible jumps
    wire InsnJALR       = (BranchOpcode &  d_Insn[2]) & ~m_Kill
        & (~e_MemAccess | MemMisaligned); 
            // disable JALR, if it is the memory bubble and there is no memory exception
    wire InsnBEQ        =  BranchOpcode & ~d_Insn[2] & ~d_Insn[14] & ~d_Insn[13];
    wire InsnBLTorBLTU  =  BranchOpcode & ~d_Insn[2] &  d_Insn[14];
    wire InsnJALorFENCEI= (d_Insn[6]==d_Insn[5]) && d_Insn[4:2]==3'b011;

    wire InsnMRET       =  SysOpcode & PrivOpcode & MRETOpcode; // check more bits?
    wire InsnCSR        =  SysOpcode & CsrPart & ~m_Kill;

    // control signals for the ALU that are set in the decode stage
    wire SelSum         = ArithOpcode & ~MULDIVPart
                            & ~d_Insn[14] & ~d_Insn[13] & ~d_Insn[12]; // ADD/SUB
    wire SetCond        = ArithOpcode & ~MULDIVPart
                            & ~d_Insn[14] &  d_Insn[13]; // SLT or SLTU
    wire SelImm         = ArithOpcode & ~MULDIVPart
                            & ~d_Insn[5]; // arith imm, only for forwarding
    wire EnShift        = ArithOpcode & ~MULDIVPart
                            & ~d_Insn[13] &  d_Insn[12];
    wire [1:0] SelLogic = (ArithOpcode & d_Insn[14])
        ? (MULDIVPart ? 2'b01 : d_Insn[13:12])
        : {1'b0, ~InsnBEQ};

    wire CsrFromExt     = InsnCSR & ~CsrFromReg;
    wire CsrRead        = CsrFromExt & ~(DestReg0Part & ~d_Insn[7]);

    wire MemWr          = MemAccess & d_Insn[5];

    wire [1:0] MemWidth = (MemAccess & ~m_Kill & ~m_ExcMem) ? d_Insn[13:12] : 2'b11;  // = no mem access
        //                                       ~~~~~~~~~ 
        // If a load follows a excepting memory access, it must be disabled
        //  to allow the writing of MTVAL
        // Maybe it can be removed, because m_Kill is now also checked in the
        // next stage, wenn MemSignal is computed

    wire ShiftArith     = d_Insn[30];
    wire NegB           = ((SUBorSLL & SUBandSLL & ~MULDIVPart) | PartBranch) & LowPart;
    wire SaveFetch      = (d_Bubble | (vMemOrSys & ~d_SaveFetch)) & ~m_Kill;
    wire Bubble         = ~m_Kill & vMemOrSys; 










    // external CSR interface
    assign retired    = ~d_Bubble & ~m_Kill & ~w_Kill;
    assign csr_read   = CsrRead;
    assign csr_modify = {Kill, (InsnCSR & (~d_Insn[13] | (d_Insn[19:15]!=0))) ? d_Insn[13:12] : 2'b00};
    assign csr_wdata  = d_Insn[14] ? d_Insn[19:15] : ForwardAE;
    assign csr_addr   = d_Insn[31:20];


    // internal CSRs

    reg CsrValidInternal;
    reg [31:0] CsrRDataInternal;
    always @* case (e_CsrAddr)
        12'h300: begin // mstatus
            CsrValidInternal = 1;
            CsrRDataInternal = {28'b0, f_MModeIntEnable, 3'b0};
        end
        default: begin
            CsrValidInternal = 0;
            CsrRDataInternal = 0;
        end
    endcase








    // write enable


    // level 1
    wire ArithOrUpper   = ~d_Insn[6] & d_Insn[4] & ~d_Insn[3];
    wire DestReg0Part   = (d_Insn[11:8] == 4'b0000); // x0 as well as unknown CSR (aka x32)
    // level 2
    wire EnableWrite    = ArithOrUpper | InsnJALorJALR | (MemAccess & ~d_Insn[5]);
    wire EnableWrite2   = CsrFromReg;

//    wire DisableWrite   = (DestReg0Part & ~d_Insn[7]) | (e_CsrFromCounter!=0) | m_Kill;
        //                                           ~~~~~~~~~~~~~~~~~~~
        // don't write in second cycle of a CSR insn, if readonly counter
//    wire DisableWrite   = (DestReg0Part & ~d_Insn[7]) | (InsnCSR & ~CsrFromReg) | m_Kill;
    wire DisableWrite   = (DestReg0Part & ~d_Insn[7]) | CsrFromExt | e_CsrFromExt | m_Kill;



    // level 3
    wire DecodeWrEn     = (EnableWrite | EnableWrite2) & ~DisableWrite;

//    wire [5:0] DecodeWrNo = CsrFromReg ? vCsrTranslate : {1'b0, d_Insn[11:7]};
    wire [5:0] DecodeWrNo = {e_CsrFromReg, d_Insn[11:7]};
        // For a CSR instruction, WrNo is set, but not WrEn.
        // If WrEn was set, the following second cycle of the CSR insn would
        // get the wrong value, because the forwarding condition for the execute
        // bypass was fulfilled.

////    wire ExecuteWrEn = ~m_Kill & (e_WrEn | e_CsrFromReg);
//    wire ExecuteWrEn = ~m_Kill & (e_WrEn);
    wire ExecuteWrEn   = ~m_Kill & 
        ((e_CsrRead & (csr_valid | CsrValidInternal)) ? 1'b1 : e_WrEn); 
        // part of the regset, even if rd=0 and therefore e_WrEn==0



    wire [5:0] ExecuteWrNo = e_WrNo;
////    wire [5:0] ExecuteWrNo = e_CsrFromReg ? {1'b1, e_WrNo[4:0]} : e_WrNo;






    // forwarding

    // 6*(4 + 32) LC = 216 LC
    wire FwdAE = e_WrEn & (d_RdNo1 == e_WrNo);
    wire FwdAM = m_WrEn & (d_RdNo1 == m_WrNo);
    wire FwdAW = w_WrEn & (d_RdNo1 == w_WrNo);
    wire [WORD_WIDTH-1:0] ForwardAR = (FwdAE | FwdAM | FwdAW) ? 0 : RdData1;
    wire [WORD_WIDTH-1:0] ForwardAM = FwdAM ? MemResult : (FwdAW ? w_WrData : 0);
    wire [WORD_WIDTH-1:0] ForwardAE = FwdAE ? ALUResult : (ForwardAR | ForwardAM);

    wire FwdBE = e_WrEn & (d_RdNo2 == e_WrNo) & ~SelImm;
    wire FwdBM = m_WrEn & (d_RdNo2 == m_WrNo) & ~SelImm;
    wire FwdBW = w_WrEn & (d_RdNo2 == w_WrNo);
    wire [WORD_WIDTH-1:0] ForwardImm = SelImm ? ImmI : 0;
    wire [WORD_WIDTH-1:0] ForwardBR = SelImm ?    0 : (FwdBW ? w_WrData : RdData2);
    wire [WORD_WIDTH-1:0] ForwardBM =  FwdBM ? MemResult : (ForwardBR | ForwardImm);
    wire [WORD_WIDTH-1:0] ForwardBE = (FwdBE ? ALUResult : ForwardBM) ^ {WORD_WIDTH{NegB}};


    wire ALUResultGrubby = 0;
    wire ForwardAWGrubby =  FwdAW ? w_WrGrubby : RdGrubby1;
    wire ForwardAMGrubby =  FwdAM ? MemResultGrubby : ForwardAWGrubby;
    wire ForwardAEGrubby = (FwdAE ? ALUResultGrubby : ForwardAMGrubby)
        & ~d_RdNo1[5];
    wire ForwardBRGrubby = ~SelImm & (FwdBW ? w_WrGrubby : RdGrubby2);
    wire ForwardBMGrubby =  FwdBM ? MemResultGrubby : ForwardBRGrubby;
    wire ForwardBEGrubby = (FwdBE ? ALUResultGrubby : ForwardBMGrubby)
        & ~d_RdNo2[5]; 
        // No grubby flag for CSR regsiters, because this would complicate
        // exception handling (for all exceptions, not just the grubby ones)







    // ALU




    wire [WORD_WIDTH-1:0] vLogicResult = ~e_SelLogic[1]
        ? (~e_SelLogic[0] ? (e_A ^ e_B) : 32'h0)
        : (~e_SelLogic[0] ? (e_A | e_B) : (e_A & e_B));
    wire [WORD_WIDTH-1:0] vPCResult =
          (e_ReturnPC ? d_PC : 0);
    wire [WORD_WIDTH-1:0] vUIResult =
        e_ReturnUI ? (e_LUIorAUIPC ? {e_Imm[31:12], 12'b0} : e_PCImm) : 0;

    // OPTIMIZE? vFastResult has one input left
    wire [WORD_WIDTH-1:0] vFastResult = 
        vLogicResult | vUIResult | vPCResult | vMulResult;
    wire [WORD_WIDTH-1:0] Sum = e_A + e_B + e_Carry;
    wire [WORD_WIDTH-1:0] vShiftAlternative = {
        e_SelSum ? Sum[WORD_WIDTH-1:1] :  vFastResult[WORD_WIDTH-1:1],
        e_SelSum ? Sum[0]              : (vFastResult[0] | vCondResultBit)};

    //                         62|61..32|31|30..0
    // SLL (funct3 001)        31|30..1 | 0|  -
    // SRL (funct3 101, i30 0)  -|   -  |31|30..0
    // SRA (funct3 101, i30 1) 31|  31  |31|30..0
    wire [62:0] vShift0 = {
        (e_ShiftRight & ~e_ShiftArith) ? 1'b0 : e_A[31],
        ~e_ShiftRight ? e_A[30:1] : (e_ShiftArith ? {30{e_A[31]}} :  30'b0),
        ~e_ShiftRight ? e_A[0] : e_A[31],
        ~e_ShiftRight ? 31'b0 : e_A[30:0]};

    wire [46:0] vShift1 = e_B[4] ? vShift0[62:16] : vShift0[46:0];
    wire [38:0] vShift2 = e_B[3] ? vShift1[46:8]  : vShift1[38:0];
    wire [34:0] vShift3 = e_B[2] ? vShift2[38:4]  : vShift2[34:0];
    wire [32:0] vShift4 = e_EnShift ? (e_B[1] ? vShift3[34:2] : vShift3[32:0]) : 0;
    wire [WORD_WIDTH-1:0] ALUResult = (e_B[0] ? vShift4[32:1] : vShift4[31:0]) | vShiftAlternative;








    // branch unit

    wire vEqual = (vLogicResult == ~0);

    wire DualKill = m_Kill | w_Kill;
    wire vLessXor = e_InvertBranch ^ ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));

    wire vLess = (Sum[31] & (e_A[31] ^ e_B[31])) ^ vLessXor;
    wire vBEQ = e_InsnBEQ & (e_InvertBranch ^ vEqual) & ~DualKill;

    wire vNotBEQ = ((e_InsnBLTorBLTU & vLess) | e_InsnJALorFENCEI) & ~DualKill;
    wire vCondResultBit = e_SetCond & vLess;

    wire Kill = vBEQ | vNotBEQ | (e_InsnJALR & ~DualKill);
        // any jump or exception


    wire [WORD_WIDTH-1:0] AddrSum = e_A + e_Imm;
    wire [WORD_WIDTH-1:0] NextPC = f_PC + 4;
    wire [WORD_WIDTH-1:0] NextOrSum = ((e_MemAccess | e_InsnJALR) & ~DualKill)
        //? {AddrSum[WORD_WIDTH-1:2], 2'b00} : NextPC;
        ? {AddrSum[WORD_WIDTH-1:1], 1'b0} : NextPC;

/*
    wire [WORD_WIDTH-1:0] MemAddr   = (vBEQ | vNotBEQ)     ? e_PCImm : NextOrSum;
    wire [WORD_WIDTH-1:0] NoBranch  = (d_Bubble & ~m_Kill) ? f_PC    : NextOrSum;
    wire [WORD_WIDTH-1:0] FetchPC   = (vBEQ | vNotBEQ)     ? e_PCImm : NoBranch;
    wire [WORD_WIDTH-1:0] DecodePC  = (d_Bubble & ~m_Kill) ? d_PC    : f_PC;
*/

    // within multicycle, but not the last one
    wire NotLastMultiCycle = ~m_Kill &
     (((MULDIVOpcode & ~m_Kill) & (d_MultiCycleCounter==0)) | (d_MultiCycleCounter>2));
//     ~~~~~~~~~~~~~~~~                                         ~~~~~~~~~~
//     first                                                 second to one befor last

    wire [WORD_WIDTH-1:0] NextOrSum_2 = NotLastMultiCycle ? d_PC : NextOrSum;

    wire [WORD_WIDTH-1:0] MemAddr   = (vBEQ | vNotBEQ)     ? e_PCImm : NextOrSum_2;
    wire [WORD_WIDTH-1:0] NoBranch  = (d_Bubble & ~m_Kill) ? f_PC    : NextOrSum_2;
    wire [WORD_WIDTH-1:0] FetchPC   = (vBEQ | vNotBEQ)     ? e_PCImm : NoBranch;
//    wire [WORD_WIDTH-1:0] DecodePC  = (d_Bubble & ~m_Kill) ? d_PC    : f_PC;
    wire [WORD_WIDTH-1:0] DecodePC  = 
        ( (d_Bubble|NotLastMultiCycle) & ~m_Kill) ? d_PC    : f_PC;










    // multi cycle instructions

    wire UnkilledStartMC = e_StartMC & ~m_Kill;
    wire MULDIVOpcode = ArithOpcode & MULDIVPart;
    wire MulASigned = (d_Insn[13:12] != 2'b11);
    wire MulBSigned = ~d_Insn[13];
    wire DivSigned = ~d_Insn[12];
    wire DivResultSign = UnkilledStartMC
        ? (e_DivSigned & (e_A[31] ^ (~e_SelRemOrDiv & e_B[31])) 
            & (e_B!=0 || e_SelRemOrDiv))
        : e_DivResultSign;

    wire MultiCycle = (d_MultiCycleCounter != 0);
    wire StartMC = MULDIVOpcode & ~MultiCycle & ~m_Kill;
    wire [5:0] MultiCycleCounter = 
        m_Kill ? 0 : (
        MultiCycle ? (d_MultiCycleCounter-1)
                   : (MULDIVOpcode ? 33 : 0));

    // MULDIV unit
    //wire [2*WORD_WIDTH-1:0] DivDiff = {32'b0, q_DivRem} - q_MulC;
    //wire DivNegative = DivDiff[2*WORD_WIDTH-1];
    wire [WORD_WIDTH:0] DivDiff = {1'b0, q_DivRem} - {1'b0, q_MulC[WORD_WIDTH-1:0]};
    wire DivNegative = (q_MulC[2*WORD_WIDTH-1:WORD_WIDTH]!=0) | DivDiff[WORD_WIDTH];

    wire [WORD_WIDTH:0] DivRem = UnkilledStartMC
        ? (m_SelDivOrMul ? {1'b0, (e_DivSigned & e_A[31]) ? (-e_A) : e_A}
                         : {e_MulASigned & e_A[WORD_WIDTH-1], e_A})
        : ((DivNegative | ~m_SelDivOrMul) ? q_DivRem 
                                          : {1'b0, DivDiff[WORD_WIDTH-1:0]});

//    wire [WORD_WIDTH:0] MulB = UnkilledStartMC
//        ? {e_MulBSigned & e_B[WORD_WIDTH-1], e_B} 
//        : {1'b0, q_MulB[WORD_WIDTH:1]};
    wire [WORD_WIDTH:0] MulB = {e_MulBSigned & e_B[WORD_WIDTH-1],
        UnkilledStartMC ? e_B : q_MulB[WORD_WIDTH:1]};
    wire [WORD_WIDTH-1:0] DivQuot = {q_DivQuot, ~DivNegative};

    wire [2*WORD_WIDTH+1:0] MulC = UnkilledStartMC
        ? {3'b0,
           m_SelDivOrMul ? ((e_DivSigned & e_B[WORD_WIDTH-1]) ? (-e_B) : e_B)
                         : {(WORD_WIDTH){1'b0}},
           {(WORD_WIDTH-1){1'b0}}}
        : { {q_MulC[2*WORD_WIDTH+1], q_MulC[2*WORD_WIDTH+1:WORD_WIDTH+1]}
            + ((~m_SelDivOrMul & q_MulB[0]) ? {q_DivRem[WORD_WIDTH], q_DivRem}
                                            : {(WORD_WIDTH+2){1'b0}}),
            q_MulC[WORD_WIDTH:1]};

    // result selection for MULDIV
    wire FromMul  = MULDIVOpcode & (d_Insn[14:12]==3'b000);
    wire FromMulH = MULDIVOpcode & ~d_Insn[14] & (d_Insn[13] | d_Insn[12]);
    wire FromDivOrRem = MULDIVOpcode & d_Insn[14];
    wire SelRemOrDiv = d_Insn[13];
    wire SelDivOrMul = StartMC ? d_Insn[14] : m_SelDivOrMul;

    wire [WORD_WIDTH-1:0] vMulResLo =
        {q_MulC[WORD_WIDTH:1]};
    wire [WORD_WIDTH-1:0] vMulResHi_C =
        {q_MulC[2*WORD_WIDTH:WORD_WIDTH+1]};
    wire [WORD_WIDTH-1:0] vMulResHi = q_MulB[0]
        ? (vMulResHi_C - q_DivRem[WORD_WIDTH-1:0])
        :  vMulResHi_C;
    wire [WORD_WIDTH-1:0] vDivRem = e_SelRemOrDiv ? q_DivRem : q_DivQuot;
    wire [WORD_WIDTH-1:0] vMulResult =
          (e_FromMul ? vMulResLo : 0)
          | (e_FromMulH ? vMulResHi : 0)
          | (e_FromDivOrRem ? (e_DivResultSign ? (-vDivRem) : vDivRem) : 0);






    // exception handling

`ifdef ENABLE_EXCEPTIONS
    wire ExcUser        = ((SysOpcode & PrivOpcode & ~d_Insn[22] & ~d_Insn[21])
                            & ~m_Kill); // REVERT m_Kill?
    wire ExcGrubbyInsn  = d_GrubbyInsn;
    wire ExcGrubbyJump  = e_InsnJALR & e_AGrubby & ~m_Kill;
    wire ExcJump        = m_Kill & f_PC[1];

    wire vWriteMEPC     = ExcJump | f_ExcGrubbyJump | m_ExcUser | m_ExcGrubbyInsn | m_ExcMem;
    wire WriteMCAUSE    = ExcJump | f_ExcGrubbyJump | m_ExcUser | m_ExcGrubbyInsn | w_ExcMem;
    wire WriteMTVAL     = d_ExcJump | d_ExcGrubbyJump | m_ExcMem;

    wire vExcOverwrite  = vWriteMEPC | m_WriteMTVAL | m_WriteMCAUSE;
    wire MemWrEn        = m_WrEn | vExcOverwrite;
//    wire MemWrEn        = ((m_CsrRead & m_CsrValid) ? 1'b1 : m_WrEn) | vExcOverwrite;

    wire [5:0] MemWrNo  =
        vWriteMEPC      ? REG_CSR_MEPC :
        (m_WriteMTVAL   ? REG_CSR_MTVAL :
        (m_WriteMCAUSE  ? REG_CSR_MCAUSE :
        m_WrNo));


    wire [4:0] Cause = m_ExcMem        ? (m_MemWr ? 6 : 4) :
                      (e_TimerInt      ? 5'b10111 :
                      (e_ExcUser       ? e_Cause1 :
                      (e_ExcGrubbyInsn ? 14 : 
                      (ExcGrubbyJump ? 10 : 
                      /* e_ExcJump */    0)))); // -> m_Cause
    wire [4:0] Cause1 = d_Insn[20] ? 3 : 11; // -> e_Cause1


    wire [WORD_WIDTH-1:0] ExcWrData2 =
        MemMisaligned   ? AddrSum       // MTVAL for mem access
                        : d_PC;         // MEPC 
    wire [WORD_WIDTH-1:0] ExcWrData =
        WriteMCAUSE     ? {m_Cause[4], {(WORD_WIDTH-5){1'b0}}, m_Cause[3:0]}  // MCAUSE
                        : e_ExcWrData2;

    wire [WORD_WIDTH-1:0] CsrResult =
        vExcOverwrite   ? ((e_ExcJump | e_ExcGrubbyJump) ? e_ExcWrData2 // MTVAL for jump
                                                         : m_ExcWrData)
                        : vCsrOrALU;

    // CSRs
/*
    reg [5:0] vCsrTranslate;
    reg CsrFromReg;
    always @* begin
        CsrFromReg <= InsnCSR;
        case (d_Insn[31:20])
            12'h305: vCsrTranslate <= REG_CSR_MTVEC;
            12'h340: vCsrTranslate <= REG_CSR_MSCRATCH;
            12'h341: vCsrTranslate <= REG_CSR_MEPC;
            12'h342: vCsrTranslate <= REG_CSR_MCAUSE;
            12'h343: vCsrTranslate <= REG_CSR_MTVAL;
            default: begin
                vCsrTranslate <= 0; // cannot be written, always 0
                CsrFromReg <= 0;
            end
        endcase
        vCsrTranslate <= {1'b1, d_Insn[26], d_Insn[23:20]};
    end
*/
    wire [4:0] vCsrTranslate = {d_Insn[26], d_Insn[23:20]};
    wire CsrFromReg = InsnCSR && (d_Insn[31:27]==5'b00110) && 
        (d_Insn[25:23]==3'b0) &&
        (
         (d_Insn[22:20]==3'b100) || // mie, mip
         ((~d_Insn[26]) && (d_Insn[22:20]==3'b101)) || // mtvec
         ((d_Insn[26]) && (~d_Insn[22]))); // mscratch, mepc, mcause, mtval

    wire [1:0] CsrOp = ((SysOpcode & d_Insn[5]) ? d_Insn[13:12] : 2'b00);

    wire [WORD_WIDTH-1:0] CsrUpdate = e_CsrSelImm ? {27'b0, e_Imm[9:5]} : e_A;

    wire [WORD_WIDTH-1:0] CsrModified = ~m_CsrOp[1]
        ? (~m_CsrOp[0] ? 32'h0 : m_CsrUpdate)
        : (~m_CsrOp[0] ? (e_A | m_CsrUpdate) : (e_A & ~m_CsrUpdate));
            // TRY: e_A instead of e_B would also be possible, if vCsrInsn is adjusted
            // e_A is a bypass from the execute stage of the next cycle


/*
    wire [WORD_WIDTH-1:0] vCsrOrALU =  
        m_CsrFromReg ? e_A 
                     : (w_CsrFromReg ? m_CsrModified 
                                     : ((m_CsrRead & m_CsrValid) ? m_CsrRdData 
                                                                 : m_WrData));
*/
    wire [WORD_WIDTH-1:0] vCsrOrALU_2 =
        (m_CsrFromReg             ? e_A           : 0) |
        (w_CsrFromReg             ? m_CsrModified : 0) |
        ((m_CsrRead & m_CsrValid) ? m_CsrRdData   : 0) |
        ((m_CsrFromReg | w_CsrFromReg | (m_CsrRead & m_CsrValid)) ? 0 : m_WrData);
    wire [WORD_WIDTH-1:0] vCsrOrALU = 
        vCsrOrALU_2;


`else
    wire ExcJump = 0;
    wire m_ExcMem = 0;
    wire CsrFromReg = 0;
    wire e_CsrFromReg = 0;
    wire [4:0] vCsrTranslate = 0;

    wire [WORD_WIDTH-1:0] CsrResult = m_WrData;
    wire MemWrEn        = m_WrEn;
    wire [5:0] MemWrNo  = m_WrNo;
`endif






    // memory signals, generated in execute stage

    wire [1:0] AddrOfs = AddrSum[1:0];
    //wire [1:0] AddrOfs = {
    //    e_A[1] ^ e_Imm[1] ^ (e_A[0] & e_Imm[0]),
    //    e_A[0] ^ e_Imm[0]};

    reg [12:0] MemSignals;
    always @* case ({e_MemWidth, AddrOfs})
        4'b0000: MemSignals = 13'b0_00010_001_0001;
        4'b0001: MemSignals = 13'b0_00100_110_0010;
        4'b0010: MemSignals = 13'b0_00011_001_0100;
        4'b0011: MemSignals = 13'b0_00101_110_1000;
        4'b0100: MemSignals = 13'b0_01010_100_0011;
        4'b0101: MemSignals = 13'b1_00000_000_0000;
        4'b0110: MemSignals = 13'b0_01011_100_1100;
        4'b0111: MemSignals = 13'b1_00000_000_0000;
        4'b1000: MemSignals = 13'b0_11010_000_1111;
        4'b1001: MemSignals = 13'b1_00000_000_0000;
        4'b1010: MemSignals = 13'b1_00000_000_0000;
        4'b1011: MemSignals = 13'b1_00000_000_0000;
        default: MemSignals = 0;
    endcase

    wire MemMisaligned = MemSignals[12] & ~m_Kill;
    wire [4:0] MemByte = m_Kill ? 0 : MemSignals[11:7];
    wire [2:0] MemSign = (m_Kill | e_MemUnsignedLoad) ? 0 : MemSignals[6:4];



    // memory stage

    wire [7:0] LoRData = (m_MemByte[0] ? mem_rdata[23:16] : mem_rdata[ 7:0]);
    wire [7:0] HiRData = (m_MemByte[0] ? mem_rdata[31:24] : mem_rdata[15:8]);

    // OPTIMIZE: combine m_MemByte[0] and  m_MemSign[0]
    wire vHiHalfSigned = (m_MemSign[0] & LoRData[7]) | (m_MemSign[2] & HiRData[7]);
    wire vHiByteSigned = (m_MemSign[0] & LoRData[7]) | (m_MemSign[1] & HiRData[7]);

    wire [15:0] HiHalf = (m_MemByte[4] ? mem_rdata[31:16] : (vHiHalfSigned ? 16'hFFFF : 16'b0)) | CsrResult[31:16];
    wire  [7:0] HiByte = (m_MemByte[3] ? HiRData          : (vHiByteSigned ?  8'hFF   :  8'b0)) | CsrResult[15:8];
    wire  [7:0] LoByte = (m_MemByte[1] ? LoRData : 8'b0) | (m_MemByte[2] ? HiRData : 8'b0)      | CsrResult[7:0];

    wire [31:0] MemResult = {HiHalf, HiByte, LoByte};
    wire MemResultGrubby = m_MemByte[4] & mem_rgrubby;
        // grubby if 32 bit load of grubby word

    wire [WORD_WIDTH-1:0] MemWriteData = {
         e_MemWidth[1] ? e_B[31:24] : (e_MemWidth[0]  ? e_B[15:8] : e_B[7:0]),
         e_MemWidth[1] ? e_B[23:16]                               : e_B[7:0],
        (e_MemWidth[1] |               e_MemWidth[0]) ? e_B[15:8] : e_B[7:0],
                                                                    e_B[7:0]};
    wire MemWriteDataGrubby = e_AGrubby | e_BGrubby;

    assign mem_valid = 1;
    assign mem_write = e_MemWr & ~DualKill;
    assign mem_wmask = MemSignals[3:0];
    assign mem_wdata = MemWriteData;
    assign mem_wgrubby = MemWriteDataGrubby;
    assign mem_addr  = MemAddr;









// ---------------------------------------------------------------------
// sequential logic
// ---------------------------------------------------------------------



    wire [5:0] RdNo1 = {RdNo1Aux, Insn[19:15]};
    wire [5:0] RdNo2 = {1'b0, Insn[24:20]};
    wire [WORD_WIDTH-1:0] RdData1;
    wire [WORD_WIDTH-1:0] RdData2;
    wire RdGrubby1;
    wire RdGrubby2;

    RegisterSet RegSet(
        .clk(clk),
        .we(MemWrEn),
        .wa(MemWrNo),
        .wd(MemResult),
        .wg(MemResultGrubby),
        .ra1(RdNo1),
        .ra2(RdNo2),
        .rd1(RdData1),
        .rg1(RdGrubby1),
        .rd2(RdData2),
        .rg2(RdGrubby2)
    );
/*
    RegisterSet RegSet(
        .clk(clk),
        .we(MemWrEn),
        .wa(MemWrNo),
        .wd(MemResult),
        .ra1(RdNo1),
        .ra2(RdNo2),
        .rd1(RdData1),
        .rd2(RdData2)
    );
*/
    always @(posedge clk) begin

        // fetch
        d_Insn <= Insn;
        d_RdNo1 <= RdNo1;
        d_RdNo2 <= RdNo2;
        if (SaveFetch) d_DelayedInsn <= mem_rdata;
        d_SaveFetch <= SaveFetch;
        d_Bubble <= Bubble;




        d_MultiCycleCounter <= MultiCycleCounter;
        e_StartMC <= StartMC;

        e_MulASigned <= MulASigned;
        e_MulBSigned <= MulBSigned;
        e_DivSigned <= DivSigned;
        e_DivResultSign <= DivResultSign;

        q_MulB <= MulB;
        q_MulC <= MulC;
        q_DivQuot <= DivQuot;
        q_DivRem <= DivRem;

        e_FromMul  <= FromMul;
        e_FromMulH <= FromMulH;
        e_FromDivOrRem <= FromDivOrRem;
        e_SelRemOrDiv <= SelRemOrDiv;

        m_SelDivOrMul <= SelDivOrMul;



        // decode
        d_PC <= DecodePC;
        e_A <= ForwardAE;
        e_B <= ForwardBE;
        e_AGrubby <= ForwardAEGrubby;
        e_BGrubby <= ForwardBEGrubby;
        e_Imm <= ImmISU;
        e_PCImm <= PCImm;

        e_WrEn <= DecodeWrEn;
        e_InsnJALR <= InsnJALR;
        e_InsnBEQ <= InsnBEQ;
        e_InsnBLTorBLTU <= InsnBLTorBLTU;
        e_InsnJALorFENCEI <= InsnJALorFENCEI;

        e_EnShift <= EnShift;
        e_ShiftArith <= ShiftArith;
        e_ReturnPC <= InsnJALorJALR;
        e_ReturnUI <= UpperOpcode;
        e_LUIorAUIPC <= d_Insn[5];

        e_SelSum <= SelSum;
        e_SetCond <= SetCond;
        e_LTU <= LTU;
        e_MemAccess <= MemAccess & ~m_Kill;
        e_MemWr <= MemWr;
        e_MemWidth <= MemWidth;

        e_SelLogic <= SelLogic;
        e_Carry <= NegB;

        e_WrNo <= DecodeWrNo;
        e_InvertBranch <= InvertBranch;
        w_Kill <= m_Kill; 

        e_InsnBit14 <= d_Insn[14];

        // execute
        m_WrEn <= ExecuteWrEn;
        m_WrNo <= ExecuteWrNo;
        m_WrData <= ALUResult;
        m_Kill <= Kill;
        m_MemSign <= MemSign;
        m_MemByte <= MemByte;
        f_PC <= FetchPC;

        // mem stage
        w_WrEn <= MemWrEn;
        w_WrNo <= MemWrNo;
        w_WrData <= MemResult;
        w_WrGrubby <= MemResultGrubby;



`ifdef ENABLE_EXCEPTIONS
        // exception handling
        e_ExcWrData2        <= ExcWrData2;
        m_ExcWrData         <= ExcWrData; 
        e_ExcUser           <= ExcUser;
        m_ExcUser           <= (e_ExcUser & ~m_Kill) | e_TimerInt;
        e_ExcGrubbyInsn     <= ExcGrubbyInsn;
        m_ExcGrubbyInsn     <= (e_ExcGrubbyInsn & ~m_Kill);
        d_ExcJump           <= ExcJump;
        e_ExcJump           <= d_ExcJump;
        f_ExcGrubbyJump     <= ExcGrubbyJump;
        d_ExcGrubbyJump     <= f_ExcGrubbyJump;
        e_ExcGrubbyJump     <= d_ExcGrubbyJump;
        m_ExcMem            <= MemMisaligned & ~m_Kill;
        w_ExcMem            <= m_ExcMem & ~m_Kill;
//        e_EBREAKorECALL     <= d_Insn[20];
        m_MemWr             <= e_MemWr;
        m_WriteMCAUSE       <= WriteMCAUSE;
        m_WriteMTVAL        <= WriteMTVAL;
        e_Cause1            <= Cause1;
        m_Cause             <= Cause;

        m_CsrUpdate         <= CsrUpdate;
        e_CsrOp             <= CsrOp;
        m_CsrOp             <= e_CsrOp;
        e_CsrFromReg        <= CsrFromReg;
        m_CsrFromReg        <= e_CsrFromReg;

        w_CsrFromReg        <= m_CsrFromReg;
        m_CsrModified       <= CsrModified;
`endif


`ifdef ENABLE_TIMER
        f_MModeIntEnable    <= MModeIntEnable;
        f_MModePriorIntEnable <= MModePriorIntEnable;
        d_TimerInt          <= TimerInt;
        e_TimerInt          <= d_TimerInt;

        e_CsrModify         <= csr_modify;
        e_CsrWData          <= csr_wdata;
        e_CsrAddr           <= csr_addr;
`endif


`ifdef ENABLE_COUNTER
        // counters
        e_CarryCYCLE        <= CounterCYCLE[32];
        e_CounterCYCLE      <= CounterCYCLE[31:0];
        e_CounterCYCLEH     <= CounterCYCLEH;
        e_CarryINSTRET      <= CounterINSTRET[32];
        e_CounterINSTRET    <= CounterINSTRET[31:0];
        e_CounterINSTRETH   <= CounterINSTRETH;
        e_CsrSelHighWord    <= d_Insn[27];
        m_CsrSelHighWord    <= e_CsrSelHighWord;
`endif

        e_CsrFromExt        <= CsrFromExt;
        e_CsrRead           <= CsrRead;
        m_CsrRead           <= e_CsrRead;
        m_CsrValid          <= csr_valid | CsrValidInternal;
        m_CsrRdData         <= csr_rdata;





        d_GrubbyInsn <= DecodeGrubbyInsn;

`ifdef DEBUG
        $display("F write=%b wmask=%b wdata=%h wgrubby=%b addr=%h rdata=%h rgrubby=%b",
            mem_write, mem_wmask, mem_wdata, mem_wgrubby, mem_addr, mem_rdata, mem_rgrubby);
        $display("D pc=\033[1;33m%h\033[0m PC%h d_Insn=%h Insn=%h",
            d_PC, d_PC, d_Insn, Insn);
        $display("R  0 %h %h %h %h %h %h %h %h", 
            RegSet.regs[0], RegSet.regs[1], RegSet.regs[2], RegSet.regs[3], 
            RegSet.regs[4], RegSet.regs[5], RegSet.regs[6], RegSet.regs[7]);
        $display("R  8 %h %h %h %h %h %h %h %h", 
            RegSet.regs[8], RegSet.regs[9], RegSet.regs[10], RegSet.regs[11], 
            RegSet.regs[12], RegSet.regs[13], RegSet.regs[14], RegSet.regs[15]);
        $display("R 16 %h %h %h %h %h %h %h %h", 
            RegSet.regs[16], RegSet.regs[17], RegSet.regs[18], RegSet.regs[19], 
            RegSet.regs[20], RegSet.regs[21], RegSet.regs[22], RegSet.regs[23]);
        $display("R 24 %h %h %h %h %h %h %h %h", 
            RegSet.regs[24], RegSet.regs[25], RegSet.regs[26], RegSet.regs[27], 
            RegSet.regs[28], RegSet.regs[29], RegSet.regs[30], RegSet.regs[31]);
        $display("R grubby %b%b%b%b %b%b%b%b %b%b%b%b %b%b%b%b",
            RegSet.regs[0][32], RegSet.regs[1][32], RegSet.regs[2][32], RegSet.regs[3][32], 
            RegSet.regs[4][32], RegSet.regs[5][32], RegSet.regs[6][32], RegSet.regs[7][32], 
            RegSet.regs[8][32], RegSet.regs[9][32], RegSet.regs[10][32], RegSet.regs[11][32], 
            RegSet.regs[12][32], RegSet.regs[13][32], RegSet.regs[14][32], RegSet.regs[15][32]);

        $display("D read x%d=%h x%d=%h", 
            d_RdNo1, RdData1, d_RdNo2, RdData2);
        $display("D Bubble=%b SaveFetch=%b f_PC=%h",
            d_Bubble, d_SaveFetch, f_PC);
        $display("E a=%h b=%h i=%h -> %h -> x%d wren=%b",
            e_A, e_B, e_Imm, ALUResult, e_WrNo, e_WrEn);
/*
        $display("E logic=%h pc=%h ui=%h e_SelSum=%b e_EnShift=%b",
            vLogicResult, vPCResult, vUIResult, e_SelSum, e_EnShift);

        $display("E MCCounter=%h rem=%h %h mul=%h e_FromMul=%b e_StartMC=%b",
            d_MultiCycleCounter, q_DivRem, q_MulB, q_MulC, e_FromMul,
            e_StartMC);
        $display("E vMulResLo=%h SelDivOrMul=%b m_SelDivOrMul=%b, e_DivSigned=%b",
            vMulResLo, SelDivOrMul, m_SelDivOrMul, e_DivSigned);
        $display("E div=%h e_FromDivOrRem=%b NotLastMC=%b",
            q_DivQuot, e_FromDivOrRem, NotLastMultiCycle);
*/


        $display("X Exc UserDEM %b%b%b GInsnDEM %b%b%b GJumpFDE %b%b%b JumpFDE %b%b%b MemEMW %b%b%b vWriteMEPC=%b",
            ExcUser,
            e_ExcUser,
            m_ExcUser,
            ExcGrubbyInsn,
            e_ExcGrubbyInsn,
            m_ExcGrubbyInsn,
            f_ExcGrubbyJump,
            d_ExcGrubbyJump,
            e_ExcGrubbyJump,
            ExcJump,
            d_ExcJump,
            e_ExcJump,
            MemMisaligned,
            m_ExcMem,
            w_ExcMem,
            vWriteMEPC
            );


        if (Kill) $display("B \033[1;35mjump %h\033[0m", FetchPC);

/*
        $display("B vBEQ=%b vNotBEQ=%b e_InsnJALR=%b KillEMW=%b%b%b",
            vBEQ, vNotBEQ, e_InsnJALR, Kill, m_Kill, w_Kill);
        $display("  e_InsnBLTorBLTU=%b vLess=%b e_InsnJALorFENCEI=%b",
            e_InsnBLTorBLTU, vLess, e_InsnJALorFENCEI);
        $display("  e_InsnBEQ=%b e_InvertBranch=%b vEqual=%b",
            e_InsnBEQ, e_InvertBranch, vEqual);
*/

        $display("G MemResultGrubby=%b e_AGrubby=%b",
            MemResultGrubby, e_AGrubby);
        $display("G FwdAEGrubby=%b d_RdNo1[5]=%b w_WrGrubby=%b RdGrubby1=%b",
            ForwardAEGrubby, d_RdNo1[5], w_WrGrubby, RdGrubby1);

/*
    wire ForwardAWGrubby =  FwdAW ? w_WrGrubby : RdGrubby1;
    wire ForwardAMGrubby =  FwdAM ? MemResultGrubby : ForwardAWGrubby;
    wire ForwardAEGrubby = (FwdAE ? ALUResultGrubby : ForwardAMGrubby)
        & ~d_RdNo1[5];
*/


        $display("F AE=%b AM=%b AW=%b AR=%h AM=%h AE=%h",
            FwdAE, FwdAM, FwdAW, ForwardAR, ForwardAM, ForwardAE);
        $display("F BE=%b BM=%b BW=%b BR=%h BM=%h BE=%h SelImm=%b",
            FwdBE, FwdBM, FwdBW, ForwardBR, ForwardBM, ForwardBE, SelImm);


/*
        $display("C MTVEC=%h MSCRATCH=%h MEPC=%h MCAUSE=%h MTVAL=%h",
            RegSet.regs[REG_CSR_MTVEC],
            RegSet.regs[REG_CSR_MSCRATCH],
            RegSet.regs[REG_CSR_MEPC],
            RegSet.regs[REG_CSR_MCAUSE],
            RegSet.regs[REG_CSR_MTVAL]);
*/

//        $display("Z AddrSum=%h NextOrSum=%h NoBranch=%h",
//            AddrSum, NextOrSum, NoBranch);


/*
        $display("C vCsrTranslate=%h CsrOp=%b e_%b m_%b",
            vCsrTranslate, CsrOp, e_CsrOp, m_CsrOp);
        $display("C vCsrOrALU=%h CsrResult=%h",
            vCsrOrALU, CsrResult);
        $display("C CsrFromReg %b e_%b m_%b w_%b m_CsrModified=%h m_WrData=%h", 
            CsrFromReg, e_CsrFromReg, m_CsrFromReg, w_CsrFromReg,
            m_CsrModified, m_WrData);
        $display("C m_CsrRead=%b m_CsrValid=%b m_CsrRdData=%h",
            m_CsrRead, m_CsrValid, m_CsrRdData);
        $display("C CSR addr=%h rdata=%h valid=%b",
            csr_addr, csr_rdata, csr_valid);
*/

        $display("M MemResult=%h m_MemSign=%b m_MemByte=%b",
            MemResult, m_MemSign, m_MemByte);

//        $display("  DecodeWrNo=%b e_WrNo=%d ExecuteWrNo=%d m_WrNo=%d",
//            DecodeWrNo, e_WrNo, ExecuteWrNo, m_WrNo);
//        $display("  DestReg0Part=%b DisableWrite=%b EnableWrite2=%b WrEnEMW=%b%b%b",
//            DestReg0Part, DisableWrite, EnableWrite2, DecodeWrEn, ExecuteWrEn, MemWrEn);
        $display("X vWriteMEPC=%b m_WriteMTVAL=%b m_WriteMCAUSE=%b m_Cause=%h",
            vWriteMEPC, m_WriteMTVAL, m_WriteMCAUSE, m_Cause);
        $display("X ExcWrData=%h e_ExcWrData2=%h m_ExcWrData=%h CsrResult=%h",
            ExcWrData, e_ExcWrData2, m_ExcWrData, CsrResult);
        $display("  MemWidth=%b e_MemWidth=%b m_MemByte=%b m_MemSign=%b",
            MemWidth, e_MemWidth,  m_MemByte, m_MemSign);






        $display("I MIE=%b MPIE=%b d_TimerInt=%b irq_timer=%b e_InsnJALR=%b",
            f_MModeIntEnable, f_MModePriorIntEnable, 
            d_TimerInt, irq_timer, e_InsnJALR);


        if (m_WrEn) $display("M x%d<-%h", m_WrNo, m_WrData);
        if (w_WrEn) $display("W x%d<-%h",w_WrNo, w_WrData);
`endif

        if (!rstn) begin
            e_WrEn <= 0;
            e_MemAccess <= 0;
            e_MemWr <= 0;
            e_MemWidth <= 2'b11; // remove?
            f_PC <= 32'hf0000000;

            d_Insn <= 32'h13;
            d_SaveFetch <= 0;
            d_Bubble <= 0;
            d_DelayedInsn <= 0;

            d_MultiCycleCounter <= 0;
            e_StartMC <= 0;
            e_FromMul <= 0;
            e_FromMulH <= 0;

`ifdef ENABLE_COUNTER
            // clear performance counters
            e_CarryCYCLE <= 0;
            e_CounterCYCLE <= 0;
            e_CounterCYCLEH <= 0;
            e_CarryINSTRET <= 0;
            e_CounterINSTRET <= 0;
            e_CounterINSTRETH <= 0;
`endif

            // fake a jump to address 0 on reset
            m_Kill <= 0;
            w_Kill <= 0;
            e_PCImm <= START_PC;
            e_InsnJALorFENCEI  <= 1;

`ifdef ENABLE_TIMER
            f_MModeIntEnable <= 0;
            f_MModePriorIntEnable <= 0;
            d_TimerInt <= 0;
`endif

        end

    end

endmodule



module CsrCounter #(
    parameter [31:0] VENDORID = 0,
    parameter [31:0] ARCHID = 0,
    parameter [31:0] IMPID = 0,
    parameter [31:0] HARTID = 0
)(
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input retired,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |modify | |wdata;

    reg Valid;
    reg [31:0] RData;
    reg [32:0] q_CounterCYCLE;
    reg [31:0] q_CounterCYCLEH;
    reg [32:0] q_CounterINSTRET;
    reg [31:0] q_CounterINSTRETH;

    always @(posedge clk) begin
        Valid <= 1;
        RData <= 0;
        case (addr)
            12'hB00: RData <= q_CounterCYCLE[31:0];     // MCYCLE
            12'hB02: RData <= q_CounterINSTRET[31:0];   // MINSTRET
            12'hB80: RData <= q_CounterCYCLEH;          // MCYCLEH
            12'hB82: RData <= q_CounterINSTRETH;        // MINSRETH
            12'hC00: RData <= q_CounterCYCLE[31:0];     // CYCLE
            12'hC80: RData <= q_CounterCYCLEH;          // CYCLEH
            12'hC01: RData <= q_CounterCYCLE[31:0];     // TIME
            12'hC81: RData <= q_CounterCYCLEH;          // TIMEH
            12'hC02: RData <= q_CounterINSTRET[31:0];   // INSTRET
            12'hC82: RData <= q_CounterINSTRETH;        // INSRETH

`ifdef ENABLE_IDS
            12'hF11: RData <= VENDORID;
            12'hF12: RData <= ARCHID;
            12'hF13: RData <= IMPID;
            12'hF14: RData <= HARTID;
`endif
            default: Valid <= 0;
        endcase

        q_CounterCYCLE    <= {1'b0, q_CounterCYCLE[31:0]} + 1;
        q_CounterCYCLEH   <= q_CounterCYCLEH + q_CounterCYCLE[32];
        q_CounterINSTRET  <= {1'b0, q_CounterINSTRET[31:0]} + {32'b0, retired};
        q_CounterINSTRETH <= q_CounterINSTRETH + q_CounterINSTRET[32];

        if (~rstn) begin
            q_CounterCYCLE    <= 0;
            q_CounterCYCLEH   <= 0;
            q_CounterINSTRET  <= 0;
            q_CounterINSTRETH <= 0;
        end
    end

    assign valid = Valid;
    assign rdata = RData;
endmodule



module CsrUartBitbang #(
    parameter [11:0]  BASE_ADDR  = 12'h7c0,    // CSR address
    parameter integer CLOCK_RATE = 12_000_000,
    parameter integer BAUD_RATE  = 115200
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input rx,
    output tx,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |wdata;

    reg q_TX;
    reg Valid;
    reg [31:0] RData;

    wire [30:0] PERIOD = CLOCK_RATE / BAUD_RATE;

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= {PERIOD, rx};
            case ({modify, wdata[0]})
                4'b0010: q_TX <= 0; // write 0
                4'b0011: q_TX <= 1; // write 1
                4'b0101: q_TX <= 1; // set
                4'b0111: q_TX <= 0; // clear
            endcase
        end
        if (~rstn) q_TX <= 1;
    end

    assign valid = Valid;
    assign rdata = RData;
    assign tx = q_TX;
endmodule




module CsrUartChar #(
    parameter [11:0]  BASE_ADDR  = 12'hbc0,    // CSR address
    parameter integer CLOCK_RATE = 12_000_000,
    parameter integer BAUD_RATE  = 115200
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input rx,
    output tx,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |wdata;

    localparam integer CLOCK_DIV = CLOCK_RATE / BAUD_RATE;

    reg Valid;
    reg [31:0] RData;

    reg  [3:0] q_UartRecvBitCounter;
    reg [15:0] q_UartRecvClkCounter;
    reg  [6:0] q_UartRecvBits;
    reg  [7:0] q_UartRecvChar;
    reg        q_UartRecvEmpty;
    reg        q_RX;
    reg  [3:0] q_UartSendBitCounter;
    reg [15:0] q_UartSendClkCounter;
    reg  [7:0] q_UartSendBits;
    reg        q_TX;


    wire UartSendFull = (q_UartSendBitCounter!=0);

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= {UartSendFull, q_UartRecvEmpty, q_UartRecvChar};
            case (modify)
                3'b001: begin // write
                    if (!UartSendFull) begin
                        q_TX <= 0;
                        q_UartSendClkCounter <= CLOCK_DIV;
                        q_UartSendBitCounter <= 10;
                        q_UartSendBits <= wdata[7:0];
                    end
                end
                3'b010: begin // set
                    q_UartRecvEmpty <= 1;
                end
            endcase
        end

        q_RX <= rx;
        if (q_UartRecvBitCounter) begin
            if (q_UartRecvClkCounter) begin
                q_UartRecvClkCounter <= q_UartRecvClkCounter - 1;
            end else begin
                q_UartRecvClkCounter <= CLOCK_DIV;
                q_UartRecvBits <= {q_RX, q_UartRecvBits[6:1]};
                if (q_UartRecvBitCounter==2) begin
                    q_UartRecvEmpty <= 0;
                    q_UartRecvChar <= {q_RX, q_UartRecvBits};
                end
                q_UartRecvBitCounter <= q_UartRecvBitCounter - 1;
            end
        end else if (~q_RX) begin
            q_UartRecvClkCounter <= CLOCK_DIV / 2;
            q_UartRecvBitCounter <= 10;
        end

        if (UartSendFull) begin
            if (q_UartSendClkCounter) begin
                q_UartSendClkCounter <= q_UartSendClkCounter - 1;
            end else begin
                q_UartSendClkCounter <= CLOCK_DIV;
                q_TX <= q_UartSendBits[0];
                q_UartSendBits <= {1'b1, q_UartSendBits[7:1]};
                q_UartSendBitCounter <= q_UartSendBitCounter - 1;
            end
        end

        if (!rstn) begin
            q_UartRecvEmpty <= 1;
            q_UartRecvBitCounter <= 0;
            q_UartSendBitCounter <= 0;
            q_TX <= 1;
        end
    end

    assign valid = Valid;
    assign rdata = RData;
    assign tx = q_TX;
endmodule





