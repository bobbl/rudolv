`define ENABLE_EXCEPTIONS

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
        regs[32] <= 0; // placeholder for unknown CSR
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= regs[ra1];
        rd2 <= regs[ra2];
    end
endmodule


module Pipeline #(
    parameter [31:0] START_PC = 0
) (
    input  clk,
    input  rstn,

    output mem_wren,
    output [3:0] mem_wmask,
    output [31:0] mem_wdata,
    output [31:0] mem_addr,
    input [31:0] mem_rdata
);
    localparam integer WORD_WIDTH = 32;

    localparam integer REG_CSR_NONE     = 6'b100000; // used for any unknown CSR
    localparam integer REG_CSR_MTVEC    = 6'b100101;
    localparam integer REG_CSR_MSCRATCH = 6'b101000;
    localparam integer REG_CSR_MEPC     = 6'b101001;
    localparam integer REG_CSR_MCAUSE   = 6'b101010;
    localparam integer REG_CSR_MTVAL    = 6'b101011;

    localparam integer CSR_COUNTER_MCYCLE       = 4'b0010;
    localparam integer CSR_COUNTER_MCYCLEH      = 4'b0011;
    localparam integer CSR_COUNTER_CYCLE        = 4'b0010;
    localparam integer CSR_COUNTER_CYCLEH       = 4'b0011;
    localparam integer CSR_COUNTER_TIME         = 4'b0010;
    localparam integer CSR_COUNTER_TIMEH        = 4'b0011;
    localparam integer CSR_COUNTER_INSTRET      = 4'b0100;
    localparam integer CSR_COUNTER_INSTRETH     = 4'b0101;
    localparam integer CSR_FROM_REGSET          = 4'b1000;


// ---------------------------------------------------------------------
// real registers
// ---------------------------------------------------------------------


    // fetch
    reg [WORD_WIDTH-1:0] f_PC;
    reg f_ChangeInsn;

    // decode
    reg [31:0] d_Insn;
    reg [5:0] d_RdNo1;
    reg [5:0] d_RdNo2;

    reg [31:0] d_DelayedInsn;
    reg d_SaveFetch;
    reg d_Bubble;


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
    reg e_UncondJump;

    reg e_SetCond;
    reg e_LTU;
    reg e_SelSum;
    reg e_MemAccess;
    reg e_MemWr;
    reg [1:0] e_MemWidth;
    reg [3:0] e_CsrCounter;


    reg [WORD_WIDTH-1:0] d_PC;

    reg [WORD_WIDTH-1:0] e_A;
    reg [WORD_WIDTH-1:0] e_B;
    reg [WORD_WIDTH-1:0] e_Imm;
    reg [WORD_WIDTH-1:0] e_PCImm;


    reg e_Carry;
    reg e_WrEn;
    reg [5:0] e_WrNo;
    reg w_Kill;

    // mem stage
    reg m_Kill; // to decode and execute stage
    reg m_WrEn;
    reg [5:0] m_WrNo;
    reg [WORD_WIDTH-1:0] m_WrData;
    reg [6:0] m_MemByte;
    reg [7:0] m_MemSign;
    reg [3:0] m_CsrCounter;

    // write back
    reg w_WrEn;
    reg [5:0] w_WrNo;
    reg [WORD_WIDTH-1:0] w_WrData;


    // exceptions
    reg [WORD_WIDTH-1:0] e_ExcWrData2;
    reg [WORD_WIDTH-1:0] m_ExcWrData;

//    reg m_ExcWrEn;
    reg e_ExcUser;
    reg m_ExcUser;
    reg d_ExcJump;
    reg e_ExcJump;
    reg m_ExcMem;
    reg w_ExcMem;
    reg e_EBREAKorECALL;
    reg m_MemWr;
    reg m_WriteMCAUSE;
    reg m_WriteMTVAL;
    reg [3:0] e_Cause;




    // CSR
    reg        e_CarryCYCLE;
    reg [32:0] e_CounterCYCLE;
    reg [31:0] e_CounterCYCLEH;
    reg        e_CarryINSTRET;
    reg [32:0] e_CounterINSTRET;
    reg [31:0] e_CounterINSTRETH;

    reg [1:0] e_CsrOp;
    reg [1:0] m_CsrOp;
    reg [4:0] e_CsrImm;
    reg [WORD_WIDTH-1:0] m_CsrUpdate;
    reg [5:0] e_CsrWrNo;

    reg  e_InsnBit14;
    wire e_ShiftRight      = e_InsnBit14;
    wire e_MemUnsignedLoad = e_InsnBit14;
    wire e_CsrSelImm       = e_InsnBit14;



// ---------------------------------------------------------------------
// combinational circuits
// ---------------------------------------------------------------------


    // decode


    wire [WORD_WIDTH-1:0] ImmI = {{21{d_Insn[31]}}, d_Insn[30:20]};


    //                               31|30..12|11..5 | 4..0
    // ImmI for JALR  (opcode 11011) 31|  31  |31..25|24..20
    // ImmI for load  (opcode 00000) 31|  31  |31..25|24..20
    // ImmS for store (opcode 01000) 31|  31  |31..25|11..7
    // ImmU for LUI   (opcode 01101) 31|30..12|   -  |   -

    // Optimisation: For LUI the lowest 12 bits must not be set correctly to 0,
    // since ReturnImm clears them in the ALU.
    // In fact, this should reduce LUT-size and not harm clock rate, but it does.
    // TRY: check later in more complex design
    wire [WORD_WIDTH-1:0] ImmISU = { // 31 LE
        d_Insn[31],                                                     // 31
        (d_Insn[4] & d_Insn[2]) ? d_Insn[30:12] : {19{d_Insn[31]}},     // 30..12
        d_Insn[31:25],                                                  // 11..5
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
//        {4{~d_Insn[4]}} & (d_Insn[2] ? d_Insn[24:21] : d_Insn[11:8]),   // 4..1
        d_Insn[4] ? 4'b0000 : (d_Insn[2] ? (d_Insn[5] ? d_Insn[24:21] : 4'b0010) : d_Insn[11:8]),
        1'b0};                                                          // 0

    wire [WORD_WIDTH-1:0] PCImm = d_PC + ImmBJU;





// 30 14 13 12 6 5 4 3 2 Ki MB
//             1 1 0 0 1  0  0  InsnJALR
//     0  0    1 1 0 0 0  0  0  InsnBEQ
//     1       1 1 0 0 0  0  0  InsnBLTorBLTU
//     1  1    1 1 0 0 0  0  0  InsnBLTU
//             1 1 0 1 1  0  0  (JAL)          \
//           1 1 1 0 0 0  0  0  (BNE,BGE,BGEU)  InvertBranch
//             0 0 0 1 1  0  0  (FENCE.I)      /
//             1 1 0   1        ReturnPC = InsnJALorJALR 
//             0 1 1 0 1        ReturnImm = InsnLUI
//             0 0 1 0 1        ReturnPCImm = InsnAUIPC

//     0  0  1 0   1 0 0        InsnSLL
//  0  1  0  1 0   1 0 0        InsnSRL
//  1  1  0  1 0   1 0 0        InsnSRA
//     0  0  0 0   1 0 0        SelSum = InsnADDorSUB
//             0 0 1 0 0        SelImm

//     1  0  0 0   1 0 0        SelLogic
//     1  0  1 0   1 0 0
//     1  1  0 0   1 0 0
//     1  1  1 0   1 0 0

//             0   1 0 0
//             0   1 0 0

//             1 1 0 0 0        branch \
//     0  0  1 0   1 0 0        SLL     \
//     0  1  0 0   1 0 0        SLT      NegB
//     0  1  1 0   1 0 0        SLTU    /
//  1  0  0    0 1 1 0 0        SUB    /




    // LUT4 at level 1

    wire LTU = d_Insn[6] ? d_Insn[13] : d_Insn[12];
        // select unsigned or signed comparison
        //     funct3 opcode  LTU
        // BLT    100 1100011  0
        // BGE    101 1100011  0
        // BLTU   110 1100011  1
        // BGE    111 1100011  1
        // SLTI   010 0010011  0
        // SLTIU  011 0010011  1
        // SLTI   010 0110011  0
        // SLTIU  011 0110011  1
        //         ^^ ^
        // for all other opcodes, LTU does not mind

    wire ShiftArith     = d_Insn[30];

    wire BranchOpcode   = (d_Insn[6:3]==4'b1100);
    wire UpperOpcode    = ~d_Insn[6] && d_Insn[4:2]==3'b101;
    wire ArithOpcode    = ~d_Insn[6] && d_Insn[4:2]==3'b100;
    wire MemAccess      = ~d_Insn[6] && d_Insn[4:2]==3'b000; // ST or LD
    wire SysOpcode      =  d_Insn[6] && d_Insn[4:2]==3'b100;
    wire PrivOpcode     =  d_Insn[5] && (d_Insn[14:12]==0);
    wire JumpOpcode     =  d_Insn[6:4]==3'b110 && d_Insn[2]; // JAL or JALR
    wire MRETOpcode     = (d_Insn[23:20]==4'b0010);

    wire SUBorSLL       =  d_Insn[13] | d_Insn[12] | (d_Insn[5] & d_Insn[30]);
    wire SUBandSLL      = ~d_Insn[14] & ~d_Insn[6] & d_Insn[4];
    wire PartBranch     = (d_Insn[6:4]==3'b110);
    wire LowPart        = (d_Insn[3:0]==4'b0011);
    wire CsrPart        = (d_Insn[5] & (d_Insn[13] | d_Insn[12]));

    wire vKillOrBubble  = m_Kill | d_Bubble;
    wire vMemOrSys      = (d_Insn[6]==d_Insn[4]) & ~d_Insn[3] & ~d_Insn[2];
        // CAUTION: also true for opcode 1010011 (OP-FP, 32 bit floating point) 

    // LUT4 at level 2

    wire InsnJALR       = (BranchOpcode &  d_Insn[2])
        & (~e_MemAccess | MemMisaligned); 
            // disable JALR, if it is the memory bubble and there is no memory exception

    wire InsnBEQ        =  BranchOpcode & ~d_Insn[2] & ~d_Insn[14] & ~d_Insn[13];
    wire InsnBLTorBLTU  =  BranchOpcode & ~d_Insn[2] &  d_Insn[14];
    wire InvertBranch   =  BranchOpcode & ~d_Insn[2] &  d_Insn[12]; // BNE or BGE or BGEU

    wire SelSum         = (ArithOpcode & ~d_Insn[14] & ~d_Insn[13] & ~d_Insn[12]); // ADD or SUB
    wire SetCond        =  ArithOpcode & ~d_Insn[14] &  d_Insn[13]; // SLT or SLTU
    wire SelImm         =  ArithOpcode & ~d_Insn[5]; // arith imm, only for forwarding
    wire EnShift        = (ArithOpcode               & ~d_Insn[13] &  d_Insn[12]);

    wire MemWr          = MemAccess & d_Insn[5];
    wire [1:0] MemWidth = (MemAccess & ~m_Kill & ~m_ExcMem) ? d_Insn[13:12] : 2'b11;  // = no mem access
//                                               ~~~~~~~~~ 
// if a load follows a excepting memory access, it must be disabled to allow the writing of MTVAL








    wire InsnMRET       =  SysOpcode & PrivOpcode & MRETOpcode; // check more bits?
    wire UncondJump     = (d_Insn[6]==d_Insn[5]) && d_Insn[4:2]==3'b011;
        // JAL or FENCE.I

    wire NegB           = ((SUBorSLL & SUBandSLL) | PartBranch) & LowPart;
    wire SaveFetch      = (d_Bubble | (vMemOrSys & ~d_SaveFetch)) & ~m_Kill;
    wire Bubble         = ~m_Kill & vMemOrSys; 

    // level 1
    wire ArithOrUpper   = ~d_Insn[6] & d_Insn[4] & ~d_Insn[3];
    wire DestReg0       = (d_Insn[11:8] == 4'b0000); // x0 as well as unknown CSR (aka x32)
    // level 2
    wire EnableWrite    = ArithOrUpper | JumpOpcode | (MemAccess & ~d_Insn[5]);
    wire EnableWrite2   = SysOpcode & CsrPart;
    wire DisableWrite   = (DestReg0 & ~d_Insn[7]) | m_Kill;
//    wire DisableWrite = (DestReg0 & ~d_Insn[7]) | m_Kill | (e_CsrCounter[2:1]!=0);
// doesn't work, don't know why

    // level 3
    wire DecodeWrEn = (EnableWrite | EnableWrite2) & ~DisableWrite;

    wire [5:0] DecodeWrNo = {1'b0, d_Insn[11:7]};


    // control signals for the ALU that are set in the decode stage
    wire [1:0] SelLogic = (ArithOpcode & d_Insn[14]) 
        ? d_Insn[13:12] 
        : {1'b0, ~InsnBEQ};










    // forwarding

    wire FwdAE = e_WrEn & (d_RdNo1 == e_WrNo); // 4 LE
    wire FwdAM = m_WrEn & (d_RdNo1 == m_WrNo); // 4 LE
    wire FwdAW = w_WrEn & (d_RdNo1 == w_WrNo); // 4 LE
    wire [WORD_WIDTH-1:0] ForwardAR = (FwdAE | FwdAM | FwdAW) ? 0 : RdData1; // 32 LE
    wire [WORD_WIDTH-1:0] ForwardAM = FwdAM ? MemResult : (FwdAW ? w_WrData : 0); // 32 LE
    wire [WORD_WIDTH-1:0] ForwardAE = FwdAE ? ALUResult : (ForwardAR | ForwardAM); // 32 LE

    wire FwdBE = e_WrEn & (d_RdNo2 == e_WrNo) & ~SelImm; // 4 LE
    wire FwdBM = m_WrEn & (d_RdNo2 == m_WrNo) & ~SelImm; // 4 LE
    wire FwdBW = w_WrEn & (d_RdNo2 == w_WrNo); // 4 LE
    wire [WORD_WIDTH-1:0] ForwardImm = SelImm ? ImmI : 0; // 32 LE
    wire [WORD_WIDTH-1:0] ForwardBR = SelImm ?    0 : (FwdBW ? w_WrData : RdData2); // 32 LE
    wire [WORD_WIDTH-1:0] ForwardBM =  FwdBM ? MemResult : (ForwardBR | ForwardImm); // 32 LE
    wire [WORD_WIDTH-1:0] ForwardBE = (FwdBE ? ALUResult : ForwardBM) ^ {WORD_WIDTH{NegB}}; // 32 LE








    // ALU

    wire [WORD_WIDTH-1:0] vLogicResult = ~e_SelLogic[1]
        ? (~e_SelLogic[0] ? (e_A ^ e_B) : 32'h0)
        : (~e_SelLogic[0] ? (e_A | e_B) : (e_A & e_B));
    wire [WORD_WIDTH-1:0] vPCResult =
          (e_ReturnPC ? d_PC : 0);
    wire [WORD_WIDTH-1:0] vUIResult =
        e_ReturnUI ? (e_LUIorAUIPC ? {e_Imm[31:12], 12'b0} : e_PCImm) : 0;

    // OPTIMIZE? vFastResult has one input left
    wire [WORD_WIDTH-1:0] vFastResult = vLogicResult | vUIResult | vPCResult;
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
    wire [32:0] vShift4 = e_EnShift ? (e_B[1] ? vShift3[34:2]  : vShift3[32:0]) : 0;
    wire [WORD_WIDTH-1:0] ALUResult = (e_B[0] ? vShift4[32:1]  : vShift4[31:0]) | vShiftAlternative;








    // branch unit

// TRY: more LCs, less PLBs, slightly lower MHz
//    wire [WORD_WIDTH:0] vLogicPlus1 = {1'b0, vLogicResult} + 1;
//    wire Equal = vLogicPlus1[WORD_WIDTH];
    wire vEqual = (vLogicResult == ~0);

    wire DualKill = m_Kill | w_Kill;
    wire vLessXor = e_InvertBranch ^ ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));

    wire vLess = (Sum[31] & (e_A[31] ^ e_B[31])) ^ vLessXor;
    wire vBEQ = e_InsnBEQ & (e_InvertBranch ^ vEqual) & ~DualKill;

    wire vNotBEQ = ((e_InsnBLTorBLTU & vLess) | e_UncondJump) & ~DualKill;
    wire vCondResultBit = e_SetCond & vLess;

    wire vJump = vBEQ | vNotBEQ;
        // taken conditional branch or direct jump or exception
    wire Kill = vBEQ | vNotBEQ | (e_InsnJALR & ~DualKill);
        // taken conditional branch or direct jump or indirect jump = any jump or exception


    wire [WORD_WIDTH-1:0] AddrSum = e_A + e_Imm;
    wire [WORD_WIDTH-1:0] NextPC = f_PC + 4;
    wire [WORD_WIDTH-1:0] NextOrSum = ((e_MemAccess | e_InsnJALR) & ~DualKill)
//        ? {AddrSum[WORD_WIDTH-1:2], 2'b00} : NextPC;
        ? {AddrSum[WORD_WIDTH-1:1], 1'b0} : NextPC;

    wire [WORD_WIDTH-1:0] MemAddr   = (vBEQ | vNotBEQ)                   ? e_PCImm : NextOrSum;
    wire [WORD_WIDTH-1:0] NoBranch  = (d_Bubble & ~m_Kill & ~e_InsnJALR) ? f_PC    : NextOrSum;
    wire [WORD_WIDTH-1:0] FetchPC   = (vBEQ | vNotBEQ)                   ? e_PCImm : NoBranch;
    wire [WORD_WIDTH-1:0] DecodePC  = (d_Bubble & ~m_Kill)               ? d_PC    : f_PC;







    // CSRs

/*  Doesn't work because MSTATUS=300h may be modified by the CRT
    // 300h..307h -> x32..x39
    // 340h..347h -> x40..x47
    wire [5:0] vCsrTranslate =
        (d_Insn[31:27]==5'b00110 && d_Insn[25:23]==3'b000)
            ? {2'b10, d_Insn[26], d_Insn[22:20]}
            : 0;
*/

    reg [5:0] vCsrTranslate;
    always @* begin
        case (d_Insn[31:20])
            12'h305: vCsrTranslate <= REG_CSR_MTVEC;
            12'h340: vCsrTranslate <= REG_CSR_MSCRATCH;
            12'h341: vCsrTranslate <= REG_CSR_MEPC;
            12'h342: vCsrTranslate <= REG_CSR_MCAUSE;
            12'h343: vCsrTranslate <= REG_CSR_MTVAL;
            default: vCsrTranslate <= 0; // cannot be written, always 0
        endcase
    end

    reg [3:0] CsrCounter;
    always @* begin
        case (d_Insn[31:20])
            12'hB00: CsrCounter <= CSR_COUNTER_MCYCLE;
            12'hB80: CsrCounter <= CSR_COUNTER_MCYCLEH;
            12'hC00: CsrCounter <= CSR_COUNTER_CYCLE;
            12'hC01: CsrCounter <= CSR_COUNTER_TIME;
            12'hC02: CsrCounter <= CSR_COUNTER_INSTRET;
            12'hC80: CsrCounter <= CSR_COUNTER_CYCLEH;
            12'hC81: CsrCounter <= CSR_COUNTER_TIMEH;
            12'hC82: CsrCounter <= CSR_COUNTER_INSTRETH;
            default: CsrCounter <= CSR_FROM_REGSET;
        endcase
    end

    wire [1:0] CsrOp    = ((SysOpcode & d_Insn[5]) ? d_Insn[13:12] : 2'b00);
    wire InsnCSR = SysOpcode & d_Insn[5] & (d_Insn[13:12]!=0);

    wire Retired = ~d_Bubble & ~m_Kill & ~w_Kill;
        // For the number of retired instructions, do not count bubbles or
        // killed instructions. In the execute stage that can be decided.
    wire [32:0] CounterCYCLE    = {1'b0, e_CounterCYCLE} + 1;
    wire [31:0] CounterCYCLEH   = e_CounterCYCLEH + e_CarryCYCLE;
    wire [32:0] CounterINSTRET  = {1'b0, e_CounterINSTRET} + {62'b0, Retired};
    wire [31:0] CounterINSTRETH = e_CounterINSTRETH + e_CarryINSTRET;

    wire [WORD_WIDTH-1:0] CsrUpdate = e_CsrSelImm ? {27'b0, e_CsrImm} : e_A;

    wire [WORD_WIDTH-1:0] vCsrCYCLE   = m_CsrCounter[1] 
        ? (m_CsrCounter[0] ? e_CounterCYCLEH : e_CounterCYCLE) : 0;
    wire [WORD_WIDTH-1:0] vCsrINSTRET = m_CsrCounter[2] 
        ? (m_CsrCounter[0] ? e_CounterINSTRETH : e_CounterINSTRET) : 0;
    wire [WORD_WIDTH-1:0] vCsrRegSet = ~m_CsrOp[1]
        ? (~m_CsrOp[0] ? 32'h0 : m_CsrUpdate)
        : (~m_CsrOp[0] ? (e_B | m_CsrUpdate) : (e_B & ~m_CsrUpdate));
            // TRY: e_A instead of e_B would also be possible, if vCsrInsn is adjusted
            // e_B is a bypass from the execute stage of the next cycle
    wire [WORD_WIDTH-1:0] vFromALU    = (m_CsrCounter[3:1]==0) ? m_WrData : 0;

    wire [WORD_WIDTH-1:0] vCsrOrALU = vCsrCYCLE | vCsrINSTRET | vCsrRegSet | vFromALU;



    wire ExecuteWrEn = ~m_Kill &
        ((e_WrEn && m_CsrCounter[2:1]==2'b00) |
            //      ~~~~~~~~~~~~~~~~~~~~~~~~
            // don't write in second cycle of a CSR insn, if readonly counter
        (e_CsrCounter[3] && e_CsrWrNo!=0));

    wire [5:0] ExecuteWrNo = e_CsrCounter[3] ? e_CsrWrNo : e_WrNo;






    // exception handling


    wire ExcUser        = (SysOpcode & PrivOpcode & ~d_Insn[22] & ~d_Insn[21]);
    wire ExcJump        = m_Kill & f_PC[1];
    wire vWriteMEPC     = ExcJump | m_ExcUser | m_ExcMem;
    wire WriteMCAUSE    = ExcJump | m_ExcUser            | w_ExcMem;
    wire WriteMTVAL     = d_ExcJump           | m_ExcMem;

    wire vExcOverwrite  = vWriteMEPC | m_WriteMTVAL | m_WriteMCAUSE;
    wire MemWrEn        = m_WrEn | vExcOverwrite;

    wire [5:0] MemWrNo =
        vWriteMEPC      ? REG_CSR_MEPC :
        (m_WriteMTVAL   ? REG_CSR_MTVAL :
        (m_WriteMCAUSE  ? REG_CSR_MCAUSE :
        m_WrNo));


/*
    wire [3:0] Cause = 
         m_ExcMem  ? (m_MemWr         ? 6 : 4) :
        (e_ExcUser ? (e_EBREAKorECALL ? 3 : 11)
                   : 0);
*/
    wire [3:0] Cause = {
        e_ExcUser & ~e_EBREAKorECALL,
        m_ExcMem,
        (m_ExcMem & m_MemWr) | e_ExcUser,
        e_ExcUser};

    wire [WORD_WIDTH-1:0] ExcWrData2 =
        MemMisaligned   ? AddrSum       // MTVAL for mem access
                        : d_PC;         // MEPC 
    wire [WORD_WIDTH-1:0] ExcWrData =
        WriteMCAUSE     ? e_Cause       // MCAUSE
                        : e_ExcWrData2;

    wire [WORD_WIDTH-1:0] CsrResult =
        vExcOverwrite   ? (e_ExcJump ? e_ExcWrData2 // MTVAL for jump
                                     : m_ExcWrData)
                        : vCsrOrALU;

    wire ReturnPC = JumpOpcode | ExcUser;




    // memory signals, generated in execute stage


/*
    wire [1:0] AddrOfs = {
        e_A[1] ^ e_Imm[1] ^ (e_A[0] & e_Imm[0]),
        e_A[0] ^ e_Imm[0]};
*/
    wire [1:0] AddrOfs = AddrSum[1:0];

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

    wire MemMisaligned = MemSignals[12];
    wire [4:0] MemByte = MemSignals[11:7];
    wire [2:0] MemSign = e_MemUnsignedLoad ? 0 : MemSignals[6:4];
    wire [3:0] MemWriteMask = MemSignals[3:0];

    // memory stage

    wire [7:0] LoRData = (m_MemByte[0] ? mem_rdata[23:16] : mem_rdata[ 7:0]);
    wire [7:0] HiRData = (m_MemByte[0] ? mem_rdata[31:24] : mem_rdata[15:8]);

    wire vHiHalfSigned = (m_MemSign[0] & LoRData[7]) | (m_MemSign[2] & HiRData[7]);
    wire vHiByteSigned = (m_MemSign[0] & LoRData[7]) | (m_MemSign[1] & HiRData[7]);

    wire [15:0] HiHalf = vHiHalfSigned ? 16'hFFFF : ((m_MemByte[4] ? mem_rdata[31:16] : 8'b0) | CsrResult[31:16]);
    wire  [7:0] HiByte = vHiByteSigned ?  8'hFF   : ((m_MemByte[3] ? HiRData          : 8'b0) | CsrResult[15:8] );
    wire  [7:0] LoByte = (m_MemByte[1] ? LoRData : 8'b0) | (m_MemByte[2] ? HiRData : 8'b0)    | CsrResult[7:0];

    wire [31:0] MemResult = {HiHalf, HiByte, LoByte};

    wire [WORD_WIDTH-1:0] MemWriteData = {
          e_MemWidth==0  ? e_B[7:0] : (e_MemWidth==1 ? e_B[15:8] : e_B[31:24]),
        (~e_MemWidth[1]) ? e_B[7:0] : e_B[23:16],
          e_MemWidth==0  ? e_B[7:0] : e_B[15:8],
        e_B[7:0]};

    assign mem_wren  = e_MemWr & ~DualKill;
    assign mem_wmask = MemWriteMask;
    assign mem_wdata = MemWriteData;
    assign mem_addr  = MemAddr;









    reg [31:0] vCsrInsn;
    always @* begin
//        if (~ExcUser & ~ExcJump & ~MemAccess & ~InsnMRET)
        if (InsnCSR)
            vCsrInsn <= {7'b0000000, vCsrTranslate[4:0],
                         5'b00000,
                         3'b110, d_Insn[11:7], 7'b0110011};
//            vCsrInsn <= {7'b0000000, 5'b00000, vCsrTranslate[4:0], 3'b110, d_Insn[11:7], 7'b0110011};
        else
            vCsrInsn <= {7'b0000000, 5'b00000, 
                         (InsnMRET ? REG_CSR_MEPC[4:0] : REG_CSR_MTVEC[4:0]), 
                         3'b000, 5'b00000, 7'b1100111};
    end


/* TRY: much slower
    wire vNotCSRBubble = ~vCsrInsn;
    wire [31:0] vCsrInsn = {
        7'b0000000,
        InsnCSR ? vCsrTranslate[4:0] : 5'b00000 ,
        InsnCSR ? 5'b00000 : (InsnMRET ? REG_CSR_MEPC[4:0] : REG_CSR_MTVEC[4:0]),
        InsnCSR,
        InsnCSR,
        1'b0,
        InsnCSR ? d_Insn[11:7] : 5'b00000,
        InsnCSR ? 7'b0110011   : 7'b1100111};
*/

//    wire RdNo1Aux = ExcUser | ExcJump | ((InsnMRET | MemAccess) & ~m_Kill) | Bubble;
    wire RdNo1Aux = Bubble | ExcJump;
////    wire RdNo1Aux = (Bubble & ~InsnJALR) | ExcUser;
    wire RdNo2Aux = Bubble;



    wire [31:0] Insn = (Bubble | ExcJump) ? vCsrInsn : (
        ((d_Bubble | d_SaveFetch) & ~m_Kill) ? d_DelayedInsn : mem_rdata);










// ---------------------------------------------------------------------
// sequential logic
// ---------------------------------------------------------------------



    wire [5:0] RdNo1 = {RdNo1Aux, Insn[19:15]};
    wire [5:0] RdNo2 = {RdNo2Aux, Insn[24:20]};
    wire [WORD_WIDTH-1:0] RdData1;
    wire [WORD_WIDTH-1:0] RdData2;

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

    always @(posedge clk) begin

        // fetch
        d_Insn <= Insn;
        d_RdNo1 <= RdNo1;
        d_RdNo2 <= RdNo2;
        if (SaveFetch) d_DelayedInsn <= mem_rdata;
        d_SaveFetch <= SaveFetch;
        d_Bubble <= Bubble;

        // decode
        d_PC <= DecodePC;
        e_A <= ForwardAE;
        e_B <= ForwardBE;
        e_Imm <= ImmISU;
        e_PCImm <= PCImm;

        e_WrEn <= DecodeWrEn;
        e_InsnJALR <= InsnJALR;
        e_InsnBEQ <= InsnBEQ;
        e_InsnBLTorBLTU <= InsnBLTorBLTU;

        e_EnShift <= EnShift;
        e_ShiftArith <= ShiftArith;
        e_ReturnPC <= ReturnPC;
        e_ReturnUI <= UpperOpcode;
        e_LUIorAUIPC <= d_Insn[5];

        e_SelSum <= SelSum;
        e_SetCond <= SetCond;
        e_LTU <= LTU;
        e_MemAccess <= MemAccess;
        e_MemWr <= MemWr;
        e_MemWidth <= MemWidth;

        e_SelLogic <= SelLogic;
        e_Carry <= NegB;

        e_WrNo <= DecodeWrNo;
        e_InvertBranch <= InvertBranch;
        w_Kill <= m_Kill; 
            // don't kill in execute stage if it is an exception that sets MCAUSE

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


            // exception handling
            e_ExcWrData2        <= ExcWrData2;
            m_ExcWrData         <= ExcWrData; 
            e_ExcUser           <= ExcUser;
            m_ExcUser           <= e_ExcUser;
            d_ExcJump           <= ExcJump;
            e_ExcJump           <= d_ExcJump;
            m_ExcMem            <= MemMisaligned;
            w_ExcMem            <= m_ExcMem;
            e_EBREAKorECALL     <= d_Insn[20];
            m_MemWr             <= e_MemWr;
            m_WriteMCAUSE       <= WriteMCAUSE;
            m_WriteMTVAL        <= WriteMTVAL;
            e_Cause             <= Cause;




            e_UncondJump        <= UncondJump;

//`ifdef ENABLE_COUNTER
            // CSR decode
            e_CarryCYCLE        <= CounterCYCLE[32];
            e_CounterCYCLE      <= CounterCYCLE[31:0];
            e_CounterCYCLEH     <= CounterCYCLEH;
            e_CarryINSTRET      <= CounterINSTRET[32];
            e_CounterINSTRET    <= CounterINSTRET[31:0];
            e_CounterINSTRETH   <= CounterINSTRETH;
            e_CsrCounter        <= (CsrOp==0 || m_Kill) ? 0 : CsrCounter;
            m_CsrCounter        <= (m_Kill) ? 0 : e_CsrCounter;
//`endif

            e_CsrWrNo           <= vCsrTranslate;
            e_CsrImm            <= d_Insn[19:15];
            m_CsrUpdate         <= CsrUpdate;

            e_CsrOp             <= CsrOp;
            m_CsrOp             <= e_CsrOp;





`ifdef DEBUG
        $display("F wren=%b wmask=%b wdata=%h addr=%h rdata=%h",
            mem_wren, mem_wmask, mem_wdata, mem_addr, mem_rdata);
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

        $display("D read x%d=%h x%d=%h", 
            d_RdNo1, RdData1, d_RdNo2, RdData2);

        $display("D Bubble=%b SaveFetch=%b",
            d_Bubble, d_SaveFetch);
//        $display("Y InsnJALR=%b InsnMRET=%b vFirstCsrCycle=%b vCsrInsn=%h d_DelayedInsn=%h",
//            InsnJALR, [1;5HInsnMRET, vFirstCsrCycle, vCsrInsn, d_DelayedInsn);
//        $display("Y Bubble=%b SaveFetch=%b ExceptionDecode=%b MemMisaligned=%b d_ExcBranch=%b",
//            Bubble, SaveFetch, ExceptionDecode, MemMisaligned, d_ExcBranch);


        $display("E a=%h b=%h i=%h -> %h -> x%d wren=%b",
            e_A, e_B, e_Imm, ALUResult, e_WrNo, e_WrEn);

        $display("E logic=%h pc=%h ui=%h e_SelSum=%b e_EnShift=%b",
            vLogicResult, vPCResult, vUIResult, e_SelSum, e_EnShift);

//        $display("X Throw d%be%bm%bw%b e_Cause=%d e_MEPC=%h m_MEPC=%h",
//            d_Throw, e_Throw, m_Throw, w_Throw, e_Cause, e_MEPC, m_MEPC);


        $display("X ExcUser %c%c%c ExcJump %c%c%c ExcMem %c%c%c vWriteMEPC=%b",
            ExcUser ? "d" : ".",
            e_ExcUser ? "e" : ".",
            m_ExcUser ? "m" : ".",
            ExcJump ? "f" : ".",
            d_ExcJump ? "d" : ".",
            e_ExcJump ? "e" : ".",
            MemMisaligned ? "e" : ".",
            m_ExcMem ? "m" : ".",
            w_ExcMem ? "w" : ".",
            vWriteMEPC
            );


        if (vJump || e_InsnJALR) $display("B jump %h", FetchPC);


        $display("F AE=%b AM=%b AW=%b AR=%h AM=%h AE=%h",
            FwdAE, FwdAM, FwdAW, ForwardAR, ForwardAM, ForwardAE);
        $display("F BE=%b BM=%b BW=%b BR=%h BM=%h BE=%h SelImm=%b",
            FwdBE, FwdBM, FwdBW, ForwardBR, ForwardBM, ForwardBE, SelImm);


        $display("C MTVEC=%h MSCRATCH=%h MEPC=%h MCAUSE=%h MTVAL=%h",
            RegSet.regs[REG_CSR_MTVEC],
            RegSet.regs[REG_CSR_MSCRATCH],
            RegSet.regs[REG_CSR_MEPC],
            RegSet.regs[REG_CSR_MCAUSE],
            RegSet.regs[REG_CSR_MTVAL]);
/*
        $display("C m_MEPC=%h vFastResult=%h vNotSh=%h m_ThrowE=%b",
            m_MEPC, vFastResult, vNotShiftResult, m_ThrowException);
        $display("C vLogicResult=%h vPCResult=%h vImmOrCsrResult=%h vCsrResult=%h e_CsrOp=%b",
            vLogicResult, vPCResult, vImmOrCsrResult, vCsrResult, e_CsrOp);
        $display("C vOverwriteByCsrRead=%h m_CsrOp=%b CsrResult=%h",
            vOverwriteByCsrRead, m_CsrOp, CsrResult);
*/
        $display("Z AddrSum=%h NextOrSum=%h NoBranch=%h",
            AddrSum, NextOrSum, NoBranch);




//        $display("C w_Kill=%b m_Kill=%b m_ThrowException=%b", 
//            w_Kill, m_Kill, m_ThrowException); 

        $display("C vCsrTranslate=%h RdNo2Aux=%b CsrOp=%b m_CsrCounter=%b",
            vCsrTranslate, RdNo2Aux, CsrOp, m_CsrCounter);
        $display("C vCsrOrALU=%h CsrResult=%h",
            vCsrOrALU, CsrResult);
        $display("M MemResult=%h m_MemSign=%b m_MemByte=%b",
            MemResult, m_MemSign, m_MemByte);

//        $display("M ExecuteWrEn=%b e_WrEn=%b m_CsrCounter[2:1]=%b DualKill=%b",
//            ExecuteWrEn, e_WrEn, m_CsrCounter[2:1], DualKill);

/*
        $display("X d_ExcBranch=%b ExcBranch=%b m_ExcIfBranchTaken=%b m_MEPCorMTVAL=%h",
            d_ExcBranch, ExcBranch, m_ExcIfBranchTaken, m_MEPCorMTVAL);
        $display("X e_ExceptionDecode=%b ExceptionDecode=%b m_Kill=%b w_Kill=%b",
            e_ExceptionDecode, ExceptionDecode, m_Kill, w_Kill);
        $display("X e_MemWidth=%b AddrOfs=%b MemMisaligned=%b",
            e_MemWidth, AddrOfs, MemMisaligned);
*/


        $display("  e_WrNo=%d e_CsrWrNo=%d e_CsrCounter=%b ExecuteWrNo=%d m_WrNo=%d",
            e_WrNo, e_CsrWrNo, e_CsrCounter, ExecuteWrNo, m_WrNo);



//        if (e_WrEn) $display("E x%d", e_WrNo);
        if (m_WrEn) $display("M x%d<-%h", m_WrNo, m_WrData);
        if (w_WrEn) $display("W x%d<-%h",w_WrNo, w_WrData);
`endif


        if (!rstn) begin
            e_WrEn <= 0;
            e_MemAccess <= 0;
            e_MemWr <= 0;
            e_MemWidth <= 2'b11; // remove?
            f_PC <= 32'h80000000;

            d_Insn <= 32'h13;
            d_SaveFetch <= 0;
            d_Bubble <= 0;
            d_DelayedInsn <= 0;

            // clear performance counters
            e_CarryCYCLE <= 0;
            e_CounterCYCLE <= 0;
            e_CounterCYCLEH <= 0;
            e_CarryINSTRET <= 0;
            e_CounterINSTRET <= 0;
            e_CounterINSTRETH <= 0;

            // fake a jump to address 0 on reset
            m_Kill <= 0;
            w_Kill <= 0;
            e_PCImm <= START_PC;
            e_UncondJump  <= 1;
        end
    end

endmodule
