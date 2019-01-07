/* verilator lint_off INITIALDLY */

//`define DISABLE_ADD

`ifndef EQUAL_COMPERATOR
`define EQUAL_COMPERATOR EqualParAdd
`endif


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




module FastAdder(
    input [31:0] a,
    input [31:0] b,
    input carry,
    output [31:0] sum
);
`ifdef DISABLE_ADD
    assign sum = a ^ b ^ {31'b0, carry};
`else
    assign sum = a + b + {31'b0, carry};
`endif
endmodule



module ArithAdder(
    input [31:0] a,
    input [31:0] b,
    input carry,
    output [31:0] sum
);
`ifdef DISABLE_ADD
    assign sum = a ^ b ^ {31'b0, carry};
`else
    assign sum = a + b + {31'b0, carry};
`endif
endmodule








// ---------------------------------------------------------------------
// equality check alternatives
// ---------------------------------------------------------------------


// do not check equality (only for debug purposes)
module EqualDisable #(
    parameter WORD_WIDTH = 32
) (
    input [WORD_WIDTH-1:0] a,
    input [WORD_WIDTH-1:0] nb,
    output equal
);
    assign equal = (a[0] == ~nb[0]);
endmodule


// let the tool infere a solution (often not the best result)
module EqualInfere #(
    parameter WORD_WIDTH = 32
) (
    input [WORD_WIDTH-1:0] a,
    input [WORD_WIDTH-1:0] nb,
    output equal
);
    assign equal = (a == ~nb);
endmodule


// logarithmic tree: slow
module EqualLog #(
    parameter WORD_WIDTH = 32
) (
    input [WORD_WIDTH-1:0] a,
    input [WORD_WIDTH-1:0] nb,
    output equal
);
    wire [0:15] Eq0;
    wire [0:3] Eq1;
    assign Eq0[0] = a[1:0] == ~nb[1:0];
    assign Eq0[1] = a[3:2] == ~nb[3:2];
    assign Eq0[2] = a[5:4] == ~nb[5:4];
    assign Eq0[3] = a[7:6] == ~nb[7:6];
    assign Eq0[4] = a[9:8] == ~nb[9:8];
    assign Eq0[5] = a[11:10] == ~nb[11:10];
    assign Eq0[6] = a[13:12] == ~nb[13:12];
    assign Eq0[7] = a[15:14] == ~nb[15:14];
    assign Eq0[8] = a[17:16] == ~nb[17:16];
    assign Eq0[9] = a[19:18] == ~nb[19:18];
    assign Eq0[10] = a[21:20] == ~nb[21:20];
    assign Eq0[11] = a[23:22] == ~nb[23:22];
    assign Eq0[12] = a[25:24] == ~nb[25:24];
    assign Eq0[13] = a[27:26] == ~nb[27:26];
    assign Eq0[14] = a[29:28] == ~nb[29:28];
    assign Eq0[15] = a[31:30] == ~nb[31:30];

    assign Eq1[0] = Eq0[0] & Eq0[1] & Eq0[2] & Eq0[3];
    assign Eq1[1] = Eq0[4] & Eq0[5] & Eq0[6] & Eq0[7];
    assign Eq1[2] = Eq0[8] & Eq0[9] & Eq0[10] & Eq0[11];
    assign Eq1[3] = Eq0[12] & Eq0[13] & Eq0[14] & Eq0[15];

    assign equal = Eq1[0] & Eq1[1] & Eq1[2] & Eq1[3];
endmodule


// sequential adders: same speed as inference
module EqualSeqAdd #(
    parameter WORD_WIDTH = 32
) (
    input [WORD_WIDTH-1:0] a,
    input [WORD_WIDTH-1:0] nb,
    output equal
);
    wire [WORD_WIDTH-1:0] Sum = a + nb + 1;
    wire [WORD_WIDTH:0] EqSum = {1'b0, Sum} + {1'b0, {WORD_WIDTH{1'b1}}};
    assign equal = ~EqSum[WORD_WIDTH];
endmodule


// parallel adders: fastest solution
module EqualParAdd #(
    parameter WORD_WIDTH = 32
) (
    input [WORD_WIDTH-1:0] a,
    input [WORD_WIDTH-1:0] nb,
    output equal
);
    wire [WORD_WIDTH:0] AminusB = {1'b0, a} + {1'b0, nb} + 1;
    wire [WORD_WIDTH:0] BminusA = {1'b0, ~a} + {1'b0, ~nb} + 1;
    assign equal = AminusB[WORD_WIDTH] & BminusA[WORD_WIDTH];
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
    localparam integer REG_CSR_MTVEC    = 6'b100101;
    localparam integer REG_CSR_MSCRATCH = 6'b100000;
    localparam integer REG_CSR_MEPC     = 6'b100001;
    localparam integer REG_CSR_MCAUSE   = 6'b100010;
    localparam integer REG_CSR_MTVAL    = 6'b100011;



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
    reg e_InsnBLTU;
    reg e_InvertBranch;
    reg [1:0] e_SelLogic;
    reg e_EnShift;
    reg e_ShiftLeft;
    reg e_ShiftArith;
    reg e_ReturnPC;
    reg e_ReturnImm;
    reg e_ReturnPCImm;
    reg e_SetCond;
    reg e_SetUnsigned;
    reg e_SelSum;
    reg e_MemAccess;
    reg e_MemWr;
    reg [1:0] e_MemWidth;
    reg e_MemUnsignedLoad;


    reg [WORD_WIDTH-1:0] d_PC;

    reg [WORD_WIDTH-1:0] e_A;
    reg [WORD_WIDTH-1:0] e_B;
    reg [WORD_WIDTH-1:0] e_Imm;
    reg [WORD_WIDTH-1:0] e_PCImm;
    reg [WORD_WIDTH-1:0] e_Target;

    reg e_Carry;
    reg e_WrEn;
    reg [5:0] e_WrNo;
    reg e_Kill;

    // mem stage
    reg m_Kill; // to decode and execute stage
    reg m_WrEn;
    reg [5:0] m_WrNo;
    reg [WORD_WIDTH-1:0] m_WrData;
    reg [6:0] m_MemByte;
    reg [7:0] m_MemSign;

    // write back
    reg w_WrEn;
    reg [5:0] w_WrNo;
    reg [WORD_WIDTH-1:0] w_WrData;


    // exceptions
    reg e_ExceptionDecode;
    reg e_ExceptionDirectJump;
    reg e_InsnMRET;
    reg [3:0] e_Cause;
    reg [WORD_WIDTH-1:0] e_PC;
    reg [3:0] m_Cause;
    reg [WORD_WIDTH-1:0] m_PC;
    reg [WORD_WIDTH-1:0] m_NewMTVAL;
    reg m_ThrowExceptionMTVAL;
    reg m_ThrowException;


    // CSR
    reg e_Nop;
    reg [63:0] e_CounterCycle;
    reg [63:0] e_CounterRetired;
    reg [1:0] e_CsrOp;
    reg e_ReturnCsr;
    reg e_CsrSelImm;
    reg [4:0] e_CsrImm;
    reg [WORD_WIDTH-1:0] e_CsrRdData;
    reg [WORD_WIDTH-1:0] m_CsrWrData;

    reg [WORD_WIDTH-1:0] d_CsrMTVEC;
    reg [WORD_WIDTH-1:0] d_CsrMSCRATCH;
    reg [WORD_WIDTH-1:0] d_CsrMEPC;
    reg [WORD_WIDTH-1:0] d_CsrMCAUSE;
    reg [WORD_WIDTH-1:0] d_CsrMTVAL;
    reg e_WrCsrMTVEC;
    reg e_WrCsrMSCRATCH;
    reg e_WrCsrMEPC;
    reg e_WrCsrMCAUSE;
    reg e_WrCsrMTVAL;
    reg m_WrCsrMTVEC;
    reg m_WrCsrMSCRATCH;
    reg m_WrCsrMEPC;
    reg m_WrCsrMCAUSE;
    reg m_WrCsrMTVAL;
    





// ---------------------------------------------------------------------
// combinational circuits
// ---------------------------------------------------------------------


    // decode


    wire [WORD_WIDTH-1:0] ImmI = {{21{d_Insn[31]}}, d_Insn[30:20]};
    //wire [WORD_WIDTH-1:0] ImmS = {{21{d_Insn[31]}}, d_Insn[30:25], d_Insn[11:8], d_Insn[7]};
    //wire [WORD_WIDTH-1:0] ImmB = {{20{d_Insn[31]}}, d_Insn[7], d_Insn[30:2OD5], d_Insn[11:8], 1'b0};
    //wire [WORD_WIDTH-1:0] ImmU = {d_Insn[31:12], 12'b0};
    //wire [WORD_WIDTH-1:0] ImmJ = {{12{d_Insn[31]}}, d_Insn[19:12], d_Insn[20], d_Insn[30:21], 1'b0};


    //                                31|30..20|19..12|11|10..5 | 4..1 |0
    // ImmB for branch (opcode 11000) 31|  31  |  31  | 7|30..25|11..8 |-
    // ImmJ for JAL    (opcode 11011) 31|  31  |19..12|20|30..25|24..21|-
    // ImmU for AUIPC  (opcode 00101) 31|30..20|19..12| -|   -  |   -  |-
    wire [WORD_WIDTH-1:0] ImmBJU = { // 30 LE
        d_Insn[31],                                                     // 31
        d_Insn[4] ? d_Insn[30:20] : {11{d_Insn[31]}},                   // 30..20
        d_Insn[2] ? d_Insn[19:12] : {8{d_Insn[31]}},                    // 19..12
        ~d_Insn[4] & (d_Insn[2] ? d_Insn[20] : d_Insn[7]),              // 11
        d_Insn[4] ? 6'b000000 : d_Insn[30:25],                          // 10..5
        {4{~d_Insn[4]}} & (d_Insn[2] ? d_Insn[24:21] : d_Insn[11:8]),   // 4..1
        1'b0};                                                          // 0

    //                               31|30..12|11..5 | 4..0
    // ImmI for JALR  (opcode 11011) 31|  31  |31..25|24..20
    // ImmI for load  (opcode 00000) 31|  31  |31..25|24..20
    // ImmS for store (opcode 01000) 31|  31  |31..25|11..7
    // ImmU for LUI   (opcode 01101) 31|30..12|   -  |   -
    // 
    // Optimisation: For LUI the lowest 12 bits must not be set correctly to 0,
    // since ReturnImm clears them in the ALU.
    // In fact, this should reduce LUT-size and not harm clock rate, but it does.
    // TRY: check later in more complex design
    wire [WORD_WIDTH-1:0] ImmISU = { // 31 LE
        d_Insn[31],                                                     // 31
        d_Insn[4] ? d_Insn[30:12] : {19{d_Insn[31]}},                   // 30..12
        d_Insn[31:25],                                                  // 11..5
        (d_Insn[6:5]==2'b01 ? d_Insn[11:7] : d_Insn[24:20])};           // 4..0
        //
        // clear ImmU
        //d_Insn[4] ? 7'b0000000 : d_Insn[31:25],                         // 11..5
        //d_Insn[4] ? 5'b0 : (d_Insn[6:5]==2'b01 ? d_Insn[11:7] : d_Insn[24:20])}; // 4..0

    wire [WORD_WIDTH-1:0] PCImm; // = d_PC + ImmBJU;
    FastAdder AddPCImm(
        .a(d_PC),
        .b(ImmBJU),
        .carry(1'b0),
        .sum(PCImm)
    );









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

    wire BranchOpcode = (d_Insn[6:3]==4'b1100);
    wire BEQOpcode = ~d_Insn[2] & ~d_Insn[14] & ~d_Insn[13];

    wire JALorJALR = (d_Insn[6:4]==3'b110 && d_Insn[2]==1'b1); // JAL or JALR
    wire UpperOpcode = (~d_Insn[6] && d_Insn[4:2]==3'b101);
    wire ArithOpcode = (~d_Insn[6] && d_Insn[4:2]==3'b100);
    wire MemOpcode   = (~d_Insn[6] && d_Insn[4:2]==3'b000); // ST or LD

    // LUT4 at level 2
    wire InsnJALR      = BranchOpcode & d_Insn[2];
    wire InsnBEQ       = BranchOpcode & BEQOpcode;
    wire InsnBLTorBLTU = BranchOpcode & ~d_Insn[2] & d_Insn[14];
    wire InsnBLTU      = BranchOpcode & ~d_Insn[2] & d_Insn[13];
    wire InvertBranch  = 
        (   d_Insn[6:2]==5'b11011 // JAL
        ||  d_Insn[6:2]==5'b00011 // FENCE.I*/
        || (d_Insn[6:2]==5'b11000 && d_Insn[12]==1'b1)); // BNE or BGE or BGEU

    wire ReturnPC    = JALorJALR; // JAL or JALR
    wire ReturnImm   = UpperOpcode & d_Insn[5]; // LUI
    wire ReturnPCImm = UpperOpcode & ~d_Insn[5]; // AUIPC

    wire EnShift = ArithOpcode & ~d_Insn[13] & d_Insn[12];
    wire ShiftLeft = ~d_Insn[14];
    wire ShiftArith = d_Insn[30];

    wire SelSum  = ArithOpcode & ~d_Insn[14] & ~d_Insn[13] & ~d_Insn[12]; // ADD or SUB
    wire SetCond = ArithOpcode & ~d_Insn[14] & d_Insn[13]; // SLT or SLTU
    wire SetUnsigned = d_Insn[12];
    wire SelImm = ArithOpcode & ~d_Insn[5]; // arith imm, only for forwarding
    wire [1:0] SelLogic = (ArithOpcode & d_Insn[14]) ? d_Insn[13:12] : 2'b01;

    wire MemAccess       = MemOpcode;
    wire MemWr           = MemOpcode & d_Insn[5];
    wire [1:0] MemWidth  = MemOpcode ? d_Insn[13:12] : 2'b11;
    wire MemUnsignedLoad = d_Insn[14];

    // OPTIMIZE
    wire SysOpcode = d_Insn[6] & (d_Insn[4:2]==3'b100);
    wire CsrOpcode = (d_Insn[13] | d_Insn[12]) & d_Insn[5] & SysOpcode;
    wire PrivOpcode = d_Insn[5] & (d_Insn[14:12]==0);

    wire MemOrCsr  = (MemOpcode | CsrOpcode);
    wire SaveFetch =  (d_Bubble | (MemOrCsr & ~d_SaveFetch)) & ~m_Kill;
    wire Bubble    = (~d_Bubble & MemOrCsr) & ~m_Kill;

    wire SUBorSLL   = d_Insn[13] |
        (~d_Insn[13] & d_Insn[12]) |
        (~d_Insn[13] & d_Insn[5] & d_Insn[30]);
    wire SUBandSLL  = ~d_Insn[14] & ~d_Insn[6] & d_Insn[4];
    wire PartBranch = d_Insn[6] & d_Insn[5] & ~d_Insn[4];
    wire LowPart    = (d_Insn[3:0] == 4'b0011);
    wire NegB = ((SUBorSLL & SUBandSLL) | PartBranch) & LowPart;

    // OPTIMIZE
    wire [1:0] CsrOp = (d_Insn[6:2]==5'b11100) ? d_Insn[13:12] : 2'b00;





    // level 1
    wire ArithOrUpper = ~d_Insn[6] & d_Insn[4] & ~d_Insn[3];
    wire JumpOpcode = d_Insn[6] & d_Insn[5] & ~d_Insn[4] & d_Insn[2];
    wire DestReg0 = (d_Insn[11:8] == 4'b0000);
    // level 2
    wire EnableWrite = ArithOrUpper | JumpOpcode | (MemOpcode & ~d_Insn[5]);
    wire DisableWrite = (DestReg0 & ~d_Insn[7]) | m_Kill;
    // level 3
    // OPTIMIZE
    wire DecodeWrEn = (EnableWrite | (CsrOp!=2'b00)) & ~DisableWrite ;


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
    wire [WORD_WIDTH-1:0] ForwardBRW = SelImm ?    0 : (FwdBW ? w_WrData : RdData2); // 32 LE
    wire [WORD_WIDTH-1:0] ForwardBM =  FwdBM ? MemResult : (ForwardBRW | ForwardImm); // 32 LE
    wire [WORD_WIDTH-1:0] ForwardBE = (FwdBE ? ALUResult : ForwardBM) ^ {WORD_WIDTH{NegB}}; // 32 LE



    // ALU

    wire [WORD_WIDTH-1:0] LogicResult = ~e_SelLogic[1]
        ? (~e_SelLogic[0] ? (e_A ^ e_B) : 32'h0)
        : (~e_SelLogic[0] ? (e_A | e_B) : (e_A & e_B));
    wire [WORD_WIDTH-1:0] PCResult =
        (e_ReturnPC ? d_PC : 0) | (e_ReturnPCImm ? e_PCImm : 0);
    wire [WORD_WIDTH-1:0] ImmOrCsrResult =
        {(e_ReturnImm ? e_Imm[31:12] : 20'b0), 12'b0}
        | (e_ReturnCsr ? e_CsrRdData : 0);
    wire [WORD_WIDTH-1:0] FastResult = LogicResult | PCResult | ImmOrCsrResult;
    wire [WORD_WIDTH-1:0] ExceptionResult =
        m_ThrowException ? m_PC : FastResult;


    wire SetAnd = e_SetCond & (e_A[31] ^ e_B[31]);
    wire SetXor = e_SetCond & ((e_A[31] ^ e_SetUnsigned) & (e_B[31] ^ e_SetUnsigned));
//    wire SetXor = e_SetCond & (e_SetUnsigned ? (~e_A[31] & ~e_B[31]) : (e_A[31] & e_B[31]));
    wire CondResultBit = ((Sum[31] & SetAnd) ^ SetXor) & ~m_ThrowException;
    wire SumOrFast = e_SelSum & ~m_ThrowException;
    wire [WORD_WIDTH-1:0] NotShResult = {
        SumOrFast ? Sum[WORD_WIDTH-1:1] : ExceptionResult[WORD_WIDTH-1:1],
        SumOrFast ? Sum[0]              : (ExceptionResult[0] | CondResultBit)};



    //                         62|61..32|31|30..0
    // SLL (funct3 001)        31|30..1 | 0|  -
    // SRL (funct3 101, i30 0)  -|   -  |31|30..0
    // SRA (funct3 101, i30 1) 31|  31  |31|30..0
    wire [62:0] Shift0 = {
        (~e_ShiftLeft & ~e_ShiftArith) ? 1'b0 : e_A[31],
        e_ShiftLeft ? e_A[30:1] : (e_ShiftArith ? {30{e_A[31]}} :  30'b0),
        e_ShiftLeft ? e_A[0] : e_A[31],
        e_ShiftLeft ? 31'b0 : e_A[30:0]};
    wire EnableShift = e_EnShift & ~m_ThrowException;

    wire [46:0] Shift1 = e_B[4] ? Shift0[62:16] : Shift0[46:0];
    wire [38:0] Shift2 = e_B[3] ? Shift1[46:8]  : Shift1[38:0];
    wire [34:0] Shift3 = e_B[2] ? Shift2[38:4]  : Shift2[34:0];
    wire [32:0] Shift4 = EnableShift ? (e_B[1] ? Shift3[34:2]  : Shift3[32:0]) : 0;
    wire [WORD_WIDTH-1:0] ALUResult = (e_B[0] ? Shift4[32:1]  : Shift4[31:0]) | NotShResult;

    wire ExecuteWrEn = m_ThrowException | (e_WrEn & ~ExecuteKill);
    wire [5:0] ExecuteWrNo = m_ThrowException ? REG_CSR_MEPC : e_WrNo;



    wire [1:0] AddrOfs = AddrSum[1:0];
/* TRY + 50LC +0.5ns
    wire [1:0] AddrOfs = {
        e_A[1] ^ e_Imm[1] ^ (e_A[0] & e_Imm[0]),
        e_A[0] ^ e_Imm[0]};
*/

    wire MemMisaligned =
m_ThrowException | 
// VERY VERY DIRTY - MUST BE OPTIMIZED
// if a exception is thrown and the next instruction is a load or store
// then set MemByte and MemSign to 0
        (((e_MemWidth==2) & (AddrOfs[1] | AddrOfs[0])) |
         ((e_MemWidth==1) &  AddrOfs[0]));

    wire SignedLB = ~e_MemWidth[1] & ~e_MemWidth[0] & ~e_MemUnsignedLoad & ~MemMisaligned;
    wire [6:0] MemByte = {
         ~MemMisaligned & (  e_MemWidth[1] & ~e_MemWidth[0]),
         ~MemMisaligned & ( ~e_MemWidth[1] &  e_MemWidth[0] &  AddrOfs[1]),
         ~MemMisaligned & ((~e_MemWidth[1] &  e_MemWidth[0] & ~AddrOfs[1])               | (e_MemWidth[1] & ~e_MemWidth[0])),
         ~MemMisaligned & ((~e_MemWidth[1] & ~e_MemWidth[0] &  AddrOfs[1] &  AddrOfs[0])),
         ~MemMisaligned & ((~e_MemWidth[1] & ~e_MemWidth[0] &  AddrOfs[1] & ~AddrOfs[0]) | (~e_MemWidth[1] & e_MemWidth[0] &  AddrOfs[1])),
         ~MemMisaligned & ((~e_MemWidth[1] & ~e_MemWidth[0] & ~AddrOfs[1] &  AddrOfs[0])),
         ~MemMisaligned & ((~e_MemWidth[1] & ~e_MemWidth[0] & ~AddrOfs[1] & ~AddrOfs[0]) | (~e_MemWidth[1] & e_MemWidth[0] & ~AddrOfs[1]) | (e_MemWidth[1] & ~e_MemWidth[0]))};
    wire [7:0] MemSign = {
         ~MemMisaligned & ((~e_MemWidth[1] & ~e_MemWidth[0] &  AddrOfs[1] &  AddrOfs[0]) | (~e_MemWidth[1] & e_MemWidth[0] &  AddrOfs[1])) & ~e_MemUnsignedLoad,
        SignedLB &  AddrOfs[1] & ~AddrOfs[0],
         ~MemMisaligned & ((~e_MemWidth[1] & ~e_MemWidth[0] & ~AddrOfs[1] &  AddrOfs[0]) | (~e_MemWidth[1] & e_MemWidth[0] & ~AddrOfs[1])) & ~e_MemUnsignedLoad,
        SignedLB & ~AddrOfs[1] & ~AddrOfs[0],
        SignedLB &  AddrOfs[1] &  AddrOfs[0],
        SignedLB &  AddrOfs[1] & ~AddrOfs[0],
        SignedLB & ~AddrOfs[1] &  AddrOfs[0],
        SignedLB & ~AddrOfs[1] & ~AddrOfs[0]};
        // MemSign[6]==MemSign[2] and MemSign[4]==MemSign[0]
        // TRY: using this fact increases the clock rate
        // m_MemSign[0] and m_MemSign[2] unused

    // OPTIMIZE
    // disable if misaligned!
    wire [3:0] MemWriteMask = {
        (((e_MemWidth==0) &  AddrOfs[1] &  AddrOfs[0]) | ((e_MemWidth==1) &  AddrOfs[1])
            | (e_MemWidth==2)) & ~MemMisaligned,
        (((e_MemWidth==0) &  AddrOfs[1] & ~AddrOfs[0]) | ((e_MemWidth==1) &  AddrOfs[1])
            | (e_MemWidth==2)) & ~MemMisaligned,
        (((e_MemWidth==0) & ~AddrOfs[1] &  AddrOfs[0]) | ((e_MemWidth==1) & ~AddrOfs[1])
            | (e_MemWidth==2)) & ~MemMisaligned,
        (((e_MemWidth==0) & ~AddrOfs[1] & ~AddrOfs[0]) | ((e_MemWidth==1) & ~AddrOfs[1])
            | (e_MemWidth==2)) & ~MemMisaligned
    };




    // memory stage
    wire MemWrEn = m_WrEn | m_ThrowExceptionMTVAL;
    wire [5:0] MemWrNo = m_ThrowExceptionMTVAL ? REG_CSR_MTVAL : m_WrNo;

    wire [WORD_WIDTH-1:0] ResultOrMTVAL = m_ThrowExceptionMTVAL ? m_NewMTVAL : m_WrData;
//    wire [WORD_WIDTH-1:0] ResultOrMTVAL = m_WrData;


    //                                     6         54         3210
    //          31..16  15..8    7..0      HiHalf    HiByte   LoByte    MemByte MemSign
    // lb  00    7..7    7..7    7..0      0 0001    00 0001    0001    0000001 00010001
    // lb  01   15..15  15..15  15..8      0 0010    00 0010    0010    0000010 00100010
    // lb  10   23..23  23..23  23..16     0 0100    00 0100    0100    0000100 01000100
    // lb  11   31..31  31..31  31..24     0 1000    00 1000    1000    0001000 10001000
    // lbu 00     -       -      7..0      0 0000    00 0000    0001    0000001 00000000
    // lbu 01     -       -     15..8      0 0000    00 0000    0010    0000010 00000000
    // lbu 10     -       -     23..16     0 0000    00 0000    0100    0000100 00000000
    // lbu 11     -       -     31..24     0 0000    00 0000    1000    0001000 00000000
    // lh  0.   15..15  15..8    7..0      0 0010    01 0000    0001    0010001 00100000
    // lh  1.   31..31  31..24  23..16     0 1000    10 0000    0100    0100100 10000000
    // lhu 0.     -     15..8    7..0      0 0000    01 0000    0001    0010001 00000000
    // lhu 1.     -     31..24  23..16     0 0000    10 0000    0100    0100100 00000000
    // lw  ..   31..16  15..8    7..0      1 0000    01 0000    0001    1010001 00000000


    wire SignHH1 = (m_MemSign[7] ? mem_rdata[31] : 1'b0) |                      // 1 LE
                   (m_MemSign[5] ? mem_rdata[15] : 1'b0);
    wire SignHH0 = (m_MemSign[6] ? mem_rdata[23] : 1'b0) |                      // 1 LE
                   (m_MemSign[4] ? mem_rdata[7]  : 1'b0);
    wire [15:0] VectorHH = m_MemByte[6] ? mem_rdata[31:16] : 0;
    wire [15:0] HiHalf = (SignHH1|SignHH0) ? 16'hFFFF : VectorHH | ResultOrMTVAL[31:16];


    wire SignHB1 = (m_MemSign[3] ? mem_rdata[31] : 1'b0) |                      // 1 LE
                   (m_MemSign[1] ? mem_rdata[15] : 1'b0);
    wire [7:0] SelByteHB = (m_MemByte[5] ? mem_rdata[31:24] : 8'b0) |           // 8 LE
                           (m_MemByte[4] ? mem_rdata[15:8]  : 8'b0);
    wire [7:0] HiByte = (SignHH0 | SignHB1) ? 8'hFF : (SelByteHB | ResultOrMTVAL[15:8]);        // 8 LE
    // m_MemSign[0] and m_MemSign[2] unused

    wire [7:0] SelByteLB1 = (m_MemByte[3] ? mem_rdata[31:24] : 8'b0) |          // 8 LE
                            (m_MemByte[2] ? mem_rdata[23:16] : 8'b0);
    wire [7:0] SelByteLB0 = (m_MemByte[1] ? mem_rdata[15:8]  : 8'b0) |          // 8 LE
                            (m_MemByte[0] ? mem_rdata[ 7:0]  : 8'b0);
    wire [7:0] LoByte = SelByteLB1 | SelByteLB0 | ResultOrMTVAL[7:0];

    // OPTIMIZE
    wire [31:0] MemResult = {HiHalf, HiByte, LoByte};

    wire [WORD_WIDTH-1:0] MemWriteData = {
        e_MemWidth==0 ? e_B[7:0] : (e_MemWidth==1 ? e_B[15:8] : e_B[31:24]),
        (~e_MemWidth[1]) ? e_B[7:0] : e_B[23:16],
        e_MemWidth==0 ? e_B[7:0] : e_B[15:8],
        e_B[7:0]};



    // PC generation


    wire Equal;
    `EQUAL_COMPERATOR #(
        .WORD_WIDTH(WORD_WIDTH)
    ) EqualityCheck(e_A, e_B, Equal);

    wire [WORD_WIDTH-1:0] Sum; // = e_A + e_B + e_Carry; // 32 LE
    ArithAdder ALUAdder(
        .a(e_A),
        .b(e_B),
        .carry(e_Carry),
        .sum(Sum)
    );

    wire [WORD_WIDTH-1:0] AddrSum; // = e_A + e_Imm; // 32 LE
    FastAdder AddrAdder(
        .a(e_A),
        .b(e_Imm),
        .carry(1'b0),
        .sum(AddrSum)
    );

    wire [WORD_WIDTH-1:0] NextPC; // = f_PC + 4;
    FastAdder IncPC(
        .a(f_PC),
        .b(32'h00000004),
        .carry(1'b0),
        .sum(NextPC)
    );




    wire ExecuteKill = m_Kill | e_Kill;

    wire ExceptionDirectJump =
        ((d_Insn[6:2]==5'b11011) & d_Insn[21]) |
            // JAL with unaligned offset
        ((d_Insn[6:2]==5'b11000) & d_Insn[8]);
            // branch with unaligned offset

    wire InsnMRET  = SysOpcode & PrivOpcode & ~d_Insn[22] & d_Insn[21];
    wire ExceptionUser = SysOpcode & PrivOpcode & ~d_Insn[22] & ~d_Insn[21];
        // throw in decode: EBREAK or ECALL
    wire ExceptionMisaligned = e_MemAccess & ~e_Kill & MemMisaligned;
        // throw in execute: memory access misaligned
        // forward to decode stage of following instruction bubble
    wire ExceptionDecode = ExceptionUser | ExceptionMisaligned;


    wire AddrBit1 = e_A[1] ^ e_Imm[1] ^ (e_A[0] & e_Imm[0]);
    wire ExceptionJALR = e_InsnJALR & AddrBit1; 
        // throw in execute: JALR misaligned


    // In the case of a misaligned memory access, throw the exception in the mem
    // stage of the bubble instruction, but write MTVAL in the mem stage of the
    // actual instruction (one cycle earlier). Therefore ThrowException depends
    // on the registered signal e_ExceptionDecode, while ThrowExceptionMTVAL
    // depends on the unregistered signal ExceptionMisaligned.
    wire ThrowException = (e_ExceptionDirectJump & DirectJump) |
        ((e_ExceptionDecode | ExceptionJALR) & ~ExecuteKill);
        // true if there really is an exception (in mem stage)
        // DirectJump is already masked by ~ExecuteKill
    wire ThrowExceptionMTVAL = (e_ExceptionDirectJump & DirectJump) |
        ((ExceptionMisaligned | ExceptionJALR) & ~ExecuteKill);
        // true for exceptions that set MTVAL (in mem stage)

    wire ExcOrMRET = e_ExceptionDecode | ExceptionJALR | e_InsnMRET;
    wire Lower = (e_InsnBLTorBLTU) & (e_A[31] ^ e_InsnBLTU) & (e_B[31] ^ e_InsnBLTU);
    wire Xor31 = (~e_InsnBEQ & (e_InvertBranch ^ Lower)) | (ExcOrMRET);
    wire And31 = (e_InsnBLTorBLTU) & (e_A[31] ^ e_B[31]);
    wire Jump31 = ~ExecuteKill & (Xor31 ^ (And31 & Sum[31]));
    wire JumpBEQ = ~ExecuteKill & e_InsnBEQ & (e_InvertBranch ^ Equal);
    wire DirectJump = Jump31 | JumpBEQ;
        // taken conditional branch or direct jump
    wire Kill = DirectJump | (e_InsnJALR & ~ExecuteKill);
        // taken conditional branch or direct jump or indirect jump = any jump

    wire [WORD_WIDTH-1:0] JumpTarget = ExceptionJALR ? d_CsrMTVEC : e_Target;
    wire [WORD_WIDTH-1:0] NextOrSum = ((e_MemAccess | e_InsnJALR) & ~ExecuteKill)
        ? {AddrSum[WORD_WIDTH-1:1], 1'b0} : NextPC;
    wire [WORD_WIDTH-1:0] NoBranch  = (d_Bubble & ~m_Kill) ? f_PC       : NextOrSum;

    // feed a register
    wire [WORD_WIDTH-1:0] MemAddr   = DirectJump           ? JumpTarget : NextOrSum;
    wire [WORD_WIDTH-1:0] FetchPC   = DirectJump           ? JumpTarget : NoBranch;
    wire [WORD_WIDTH-1:0] DecodePC  = (d_Bubble & ~m_Kill) ? d_PC       : f_PC;

    wire [WORD_WIDTH-1:0] Target =
        (d_Insn[6:2]==5'b00011) 
            ? f_PC 
            : (InsnMRET 
                ? d_CsrMEPC
                : ((ExceptionUser | ExceptionMisaligned | ExceptionDirectJump) 
                    ? d_CsrMTVEC 
                    : PCImm));




// ---------------------------------------------------------------------
// sequential logic
// ---------------------------------------------------------------------



    assign mem_wren = e_MemWr & ~ExecuteKill;
    assign mem_wmask = MemWriteMask;
    assign mem_wdata = MemWriteData;
    assign mem_addr = MemAddr;


    wire [31:0] NextInsn = Bubble ? 32'h4013 : d_DelayedInsn;
    wire [31:0] Insn = (Bubble | ((d_Bubble | d_SaveFetch) & ~m_Kill)) ? NextInsn : mem_rdata;

/* TRY: 
    To reduce logic in the fetch stage, some can be moved to the decode stage.
    But then there is a dependency on the critical path via Jump.
    An additional register f_ChangeInsn is required for ChangeInsn.

    wire ChangeInsn = ((Bubble | SaveFetch) & 
        ~(DirectJump | (e_InsnJALR & ~m_Kill)));
        // The second part is necessary, because when there is a jump in the
        // execute stage, ChangeInsn and MemAddr are set before the end of
        // the cycle. During the next cycle, the instruction word of the branch
        // target arrives and f_ChangeInsn must be false to forward it to the
        // decode stage.
    wire [31:0] Insn = (Bubble | f_ChangeInsn) ? NextInsn : mem_rdata;
        // Here, m_Kill indicates that there is a branch and we have to take the
        // instruction from the memory as it is the first instruction from the
        // branch target. Therefore clearing in the case of m_Kill is wrong here.
*/



    wire RdNo1H = 0;
    wire RdNo2H = 0;
    wire [5:0] RdNo1 = {RdNo1H, Insn[19:15]};
    wire [5:0] RdNo2 = {RdNo2H, Insn[24:20]};
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

    wire [WORD_WIDTH-1:0] CsrUpdate = e_CsrSelImm ? {27'b0, e_CsrImm} : e_A;

    always @(posedge clk) begin
        if (!rstn) begin
            e_WrEn <= 0;
            e_MemAccess <= 0;
            e_MemWr <= 0;
            f_PC <= 32'h80000000;

            d_Insn <= 32'h13;
            d_SaveFetch <= 0;
            d_Bubble <= 0;
            d_DelayedInsn <= 0;

            e_CounterCycle <= 0;
            e_CounterRetired <= 0;
            e_Nop <= 0;


            // fake a jump to address 0 on reset
            e_ExceptionDecode <= 0;
            e_ExceptionDirectJump <= 0;
            e_InsnMRET <= 0;
            e_InsnJALR <= 0;
            e_InsnBEQ <= 0;
            e_InsnBLTorBLTU <= 0;
            e_InvertBranch <= 1;
            m_Kill <= 0;
            e_Kill <= 0;
            e_PCImm <= 0;
            e_Target <= START_PC;

        end else begin

//            f_ChangeInsn <= ChangeInsn;

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
        e_Target <= Target;

        e_WrEn <= DecodeWrEn;
        e_InsnJALR <= InsnJALR;
        e_InsnBEQ <= InsnBEQ;
        e_InsnBLTorBLTU <= InsnBLTorBLTU;
        e_InsnBLTU <= InsnBLTU;
        e_EnShift <= EnShift;
        e_ShiftLeft <= ShiftLeft;
        e_ShiftArith <= ShiftArith;
        e_ReturnPC <= ReturnPC;
        e_ReturnImm <= ReturnImm;
        e_ReturnPCImm <= ReturnPCImm;
        e_SelSum <= SelSum;
        e_SetCond <= SetCond;
        e_SetUnsigned <= SetUnsigned;
        e_MemAccess <= MemAccess;
        e_MemWr <= MemWr;
        e_MemWidth <= MemWidth;
        e_MemUnsignedLoad <= MemUnsignedLoad;

        e_SelLogic <= SelLogic;
        e_Carry <= NegB;

        e_WrNo <= d_Insn[11:7];
        e_InvertBranch <= InvertBranch;
        e_Kill <= m_Kill;


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
        e_PC <= d_Bubble ? e_PC : d_PC;

        // potential exception cause (only one possible per instruction class)
        if (~d_Bubble) begin
            // in a bubble the exception cause remains unchanged
            if (MemOpcode) begin
                e_Cause <= d_Insn[5] 
                    ? 6         // store address misaligned
                    : 4;        // load address misaligned
            end else if (d_Insn[6:4]==3'b110) begin
                 // jump or branch
                e_Cause <= 0;   // instruction address misaligned
            end else begin
                // software trap
                e_Cause <= d_Insn[20]
                    ? 3         // breakpoint
                    : 11;       // environment call from M-mode
            end
        end

        e_InsnMRET  <= InsnMRET;
        e_ExceptionDecode <= ExceptionDecode;
        e_ExceptionDirectJump <= ExceptionDirectJump;

        m_PC <= e_PC;
        m_Cause <= e_Cause;
        m_ThrowException <= ThrowException;
        m_ThrowExceptionMTVAL <= ThrowExceptionMTVAL;

        // CSR decode
        e_Nop <= d_Bubble | ExecuteKill;
        e_CounterCycle <= e_CounterCycle + 1;
        e_CounterRetired <= e_CounterRetired + {63'b0, (~e_Nop & ~ExecuteKill)};
        e_CsrRdData <= 0;
        e_WrCsrMTVEC <= 0;
        e_WrCsrMSCRATCH <= 0;
        e_WrCsrMEPC <= 0;
        e_WrCsrMCAUSE <= 0;
        e_WrCsrMTVAL <= 0;
        e_CsrOp <= CsrOp;
        e_ReturnCsr <= (d_Insn[6:2]==5'b11100) && (d_Insn[13:12]!=2'b00);
        e_CsrSelImm <= d_Insn[14];
        e_CsrImm <= d_Insn[19:15];
        case (d_Insn[31:20])
//            12'h304: begin
//                e_CsrRdData <= d_CsrMIE;
//                e_WrCsrMIE <= 1;
//            end
            12'h305: begin
                e_CsrRdData <= m_WrCsrMTVEC ? m_CsrWrData : d_CsrMTVEC;
                e_WrCsrMTVEC <= 1;
            end
            12'h340: begin
                e_CsrRdData <= m_WrCsrMSCRATCH ? m_CsrWrData : d_CsrMSCRATCH;
                e_WrCsrMSCRATCH <= 1;
            end
            12'h341: begin
//                e_CsrRdData <= m_ThrowException ? e_PC : (m_WrCsrMEPC ? m_CsrWrData : d_CsrMEPC);
                e_CsrRdData <= (m_WrCsrMEPC ? m_CsrWrData : d_CsrMEPC);
                e_WrCsrMEPC <= 1;
            end
            12'h342: begin
//                e_CsrRdData <= m_ThrowException ? e_Cause ? m_WrCsrMCAUSE ? m_CsrWrData : d_CsrMCAUSE;
                e_CsrRdData <= m_WrCsrMCAUSE ? m_CsrWrData : d_CsrMCAUSE;
                e_WrCsrMCAUSE <= 1;
            end
            12'h343: begin
                e_CsrRdData <= m_WrCsrMTVAL ? m_CsrWrData : d_CsrMTVAL;
                e_WrCsrMTVAL <= 1;
            end
//            12'h344: begin
//                e_CsrRdData <= d_CsrMIP;
//                e_WrCsrMIP <= 1;
//            end
            12'hB00: begin // MCYCLE
                e_CsrRdData <= e_CounterCycle[31:0];
            end
            12'hB80: begin // MCYCLEH
                e_CsrRdData <= e_CounterCycle[63:32];
            end
            12'hC00: begin // CYCLE
                e_CsrRdData <= e_CounterCycle[31:0];
            end
            12'hC01: begin // TIME
                e_CsrRdData <= e_CounterCycle[31:0];
            end
            12'hC02: begin // INSTRET
                e_CsrRdData <= e_CounterRetired[31:0];
            end
            12'hC80: begin // CYCLEH
                e_CsrRdData <= e_CounterCycle[63:32];
            end
            12'hC81: begin // TIMEH
                e_CsrRdData <= e_CounterCycle[63:32];
            end
            12'hC82: begin // INSTRETH
                e_CsrRdData <= e_CounterRetired[63:32];
            end
            default: begin
                e_CsrRdData <= 0;
            end
        endcase


        // CSR execute
        m_CsrWrData <= 0;
        case (e_CsrOp & {~ExecuteKill, ~ExecuteKill})
            2'b01: m_CsrWrData <= CsrUpdate;
            2'b10: m_CsrWrData <= e_CsrRdData | CsrUpdate;
            2'b11: m_CsrWrData <= e_CsrRdData & ~CsrUpdate;
            default: m_CsrWrData <= CsrUpdate; // don't care
        endcase
        m_WrCsrMTVEC    <= e_WrCsrMTVEC    & (e_CsrOp[0] | e_CsrOp[1]) & ~ExecuteKill;
        m_WrCsrMSCRATCH <= e_WrCsrMSCRATCH & (e_CsrOp[0] | e_CsrOp[1]) & ~ExecuteKill;
        m_WrCsrMEPC     <= e_WrCsrMEPC     & (e_CsrOp[0] | e_CsrOp[1]) & ~ExecuteKill;
        m_WrCsrMCAUSE   <= e_WrCsrMCAUSE   & (e_CsrOp[0] | e_CsrOp[1]) & ~ExecuteKill;
        m_WrCsrMTVAL    <= e_WrCsrMTVAL    & (e_CsrOp[0] | e_CsrOp[1]) & ~ExecuteKill;




        // CSR write (in memory stage)
        d_CsrMTVEC    <= m_WrCsrMTVEC    ? m_CsrWrData : d_CsrMTVEC;
        d_CsrMSCRATCH <= m_WrCsrMSCRATCH ? m_CsrWrData : d_CsrMSCRATCH;
        d_CsrMEPC     <= m_ThrowException
            ? m_PC 
            : (m_WrCsrMEPC     ? m_CsrWrData : d_CsrMEPC);
        d_CsrMCAUSE   <= m_ThrowException
            ? {28'b0, m_Cause}
            : (m_WrCsrMCAUSE   ? m_CsrWrData : d_CsrMCAUSE);

        m_NewMTVAL <= e_ExceptionDirectJump 
            ? e_PCImm
            : {AddrSum[WORD_WIDTH-1:1], AddrSum[0] & ~e_InsnJALR}; // TRY: AddrOfs[0]
        if (m_ThrowExceptionMTVAL)
            d_CsrMTVAL <= m_NewMTVAL;
        else if (m_WrCsrMTVAL)
            d_CsrMTVAL <= m_CsrWrData;


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

        $display("D Bubble=%b SaveFetch=%b",
            d_Bubble, d_SaveFetch);
/*
        $display("B DirectJump=%b Jump31=%b JumpBEQ=%b ExecuteKill=%b Xor31=%b And31=%b e_A[31]=%b e_B[31]=%b",
            DirectJump, Jump31, JumpBEQ, ExecuteKill, Xor31, And31, e_InsnBLTorBLTU, e_A[31], e_B[31]);
        $display("B e_InsnBEQ=%b e_InvertBranch=%b Lower=%b Exception=%b e_InsnBLTU=%b e_InsnBLTorBLTU=%b",
            e_InsnBEQ, e_InvertBranch, Lower, Exception, e_InsnBLTU, e_InsnBLTorBLTU);
*/
        $display("E a=%h b=%h -> %h",
            e_A, e_B, ALUResult);


        if (DirectJump || e_InsnJALR) $display("B jump %h", FetchPC);


        $display("C MTVEC=%h %h MSCRATCH=%h %h MEPC=%h %h",
            d_CsrMTVEC, RegSet.regs[REG_CSR_MTVEC],
            d_CsrMSCRATCH, RegSet.regs[REG_CSR_MSCRATCH],
            d_CsrMEPC, RegSet.regs[REG_CSR_MEPC]);
        $display("C CYCLE=%h INSTRET=%h MCAUSE=%h %h MTVAL=%h %h",
            e_CounterCycle, e_CounterRetired,
            d_CsrMCAUSE, RegSet.regs[REG_CSR_MCAUSE],
            d_CsrMTVAL, RegSet.regs[REG_CSR_MTVAL]);

        $display("C m_PC=%h FastResult=%h ExceptionResult=%h NotSh=%h m_ThrowE=%b",
            m_PC, FastResult, ExceptionResult, NotShResult, m_ThrowException);
        $display("C m_WrData=%h",
            m_WrData);

        if (w_WrEn) 
            $display("W x%d=%h",w_WrNo, w_WrData);


`endif


        end
    end

endmodule
