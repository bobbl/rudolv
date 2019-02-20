`define ENABLE_EXCEPTIONS
//`define ENABLE_COUNTER


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
    reg e_InsnJALorFENCEI;

    reg e_SetCond;
    reg e_LTU;
    reg e_SelSum;
    reg e_MemAccess;
    reg e_MemWr;
    reg [1:0] e_MemWidth;


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
    reg [4:0] m_MemByte;
    reg [2:0] m_MemSign;

    // write back
    reg w_WrEn;
    reg [5:0] w_WrNo;
    reg [WORD_WIDTH-1:0] w_WrData;


    // exceptions
    reg [WORD_WIDTH-1:0] e_ExcWrData2;
    reg [WORD_WIDTH-1:0] m_ExcWrData;

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
`ifdef ENABLE_COUNTER
    reg        e_CarryCYCLE;
    reg [32:0] e_CounterCYCLE;
    reg [31:0] e_CounterCYCLEH;
    reg        e_CarryINSTRET;
    reg [32:0] e_CounterINSTRET;
    reg [31:0] e_CounterINSTRETH;
    reg [1:0]  e_CsrFromCounter;
    reg [1:0]  m_CsrFromCounter;
    reg        e_CsrSelHighWord;
    reg        m_CsrSelHighWord;
`endif

    reg [4:0] e_CsrImm;
    reg [WORD_WIDTH-1:0] m_CsrUpdate;
    reg [1:0] e_CsrOp;
    reg [1:0] m_CsrOp;

    reg e_CsrFromReg;
    reg m_CsrFromReg;

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
    wire CsrPart        = (d_Insn[5] & (d_Insn[13] | d_Insn[12]));

    wire vMemOrSys      = (d_Insn[6]==d_Insn[4]) & ~d_Insn[3] & ~d_Insn[2];
        // CAUTION: also true for opcode 1010011 (OP-FP, 32 bit floating point) 

    // LUT4 at level 2

    wire InsnJALR       = (BranchOpcode &  d_Insn[2])
        & (~e_MemAccess | MemMisaligned); 
            // disable JALR, if it is the memory bubble and there is no memory exception
    wire InsnBEQ        =  BranchOpcode & ~d_Insn[2] & ~d_Insn[14] & ~d_Insn[13];
    wire InsnBLTorBLTU  =  BranchOpcode & ~d_Insn[2] &  d_Insn[14];
//    wire InvertBranch   =  BranchOpcode & ~d_Insn[2] &  d_Insn[12]; // BNE or BGE or BGEU
    wire InsnJALorFENCEI= (d_Insn[6]==d_Insn[5]) && d_Insn[4:2]==3'b011;
    wire InsnMRET       =  SysOpcode & PrivOpcode & MRETOpcode; // check more bits?

    // control signals for the ALU that are set in the decode stage
    wire SelSum         = (ArithOpcode & ~d_Insn[14] & ~d_Insn[13] & ~d_Insn[12]); // ADD or SUB
    wire SetCond        =  ArithOpcode & ~d_Insn[14] &  d_Insn[13]; // SLT or SLTU
    wire SelImm         =  ArithOpcode & ~d_Insn[5]; // arith imm, only for forwarding
    wire EnShift        = (ArithOpcode               & ~d_Insn[13] &  d_Insn[12]);
    wire [1:0] SelLogic = (ArithOpcode & d_Insn[14])
        ? d_Insn[13:12] 
        : {1'b0, ~InsnBEQ};

    wire MemWr          = MemAccess & d_Insn[5];
    wire [1:0] MemWidth = (MemAccess & ~m_Kill & ~m_ExcMem) ? d_Insn[13:12] : 2'b11;  // = no mem access
        //                                       ~~~~~~~~~ 
        // If a load follows a excepting memory access, it must be disabled
        //  to allow the writing of MTVAL

    wire ShiftArith     = d_Insn[30];
    wire NegB           = ((SUBorSLL & SUBandSLL) | PartBranch) & LowPart;
    wire SaveFetch      = (d_Bubble | (vMemOrSys & ~d_SaveFetch)) & ~m_Kill;
    wire Bubble         = ~m_Kill & vMemOrSys; 





    // write enable


    // level 1
    wire ArithOrUpper   = ~d_Insn[6] & d_Insn[4] & ~d_Insn[3];
    wire DestReg0       = (d_Insn[11:8] == 4'b0000); // x0 as well as unknown CSR (aka x32)
    // level 2
    wire EnableWrite    = ArithOrUpper | InsnJALorJALR | (MemAccess & ~d_Insn[5]);
    wire EnableWrite2   = (CsrFromCounter!=0);
    wire DisableWrite   = (DestReg0 & ~d_Insn[7]) | m_Kill;
    // level 3
    wire DecodeWrEn     = (EnableWrite | EnableWrite2) & ~DisableWrite;

    wire [5:0] DecodeWrNo = CsrFromReg ? vCsrTranslate : {1'b0, d_Insn[11:7]};
        // For a CSR instruction, WrNo is set, but not WrEn.
        // If WrEn was set, the following second cycle of the CSR insn would
        // get the wrong value, because the forwarding condition for the execute
        // bypass was fulfilled.


    wire ExecuteWrEn = ~m_Kill &
        ((e_WrEn && m_CsrFromCounter==2'b00)
            //      ~~~~~~~~~~~~~~~~~~~
            // don't write in second cycle of a CSR insn, if readonly counter
        | e_CsrFromReg);
            // In the first cycle, a regset CSR insn always writes the CSR
            // part of the regset, even if rd=0 and therefore e_WrEn==0

    wire [5:0] ExecuteWrNo = e_WrNo;









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

    wire vJump = vBEQ | vNotBEQ;
        // taken conditional branch or direct jump or exception
    wire Kill = vBEQ | vNotBEQ | (e_InsnJALR & ~DualKill);
        // any jump or exception


    wire [WORD_WIDTH-1:0] AddrSum = e_A + e_Imm;
    wire [WORD_WIDTH-1:0] NextPC = f_PC + 4;
    wire [WORD_WIDTH-1:0] NextOrSum = ((e_MemAccess | e_InsnJALR) & ~DualKill)
//        ? {AddrSum[WORD_WIDTH-1:2], 2'b00} : NextPC;
        ? {AddrSum[WORD_WIDTH-1:1], 1'b0} : NextPC;

    wire [WORD_WIDTH-1:0] MemAddr   = (vBEQ | vNotBEQ)     ? e_PCImm : NextOrSum;
    wire [WORD_WIDTH-1:0] NoBranch  = (d_Bubble & ~m_Kill) ? f_PC    : NextOrSum;
    wire [WORD_WIDTH-1:0] FetchPC   = (vBEQ | vNotBEQ)     ? e_PCImm : NoBranch;
    wire [WORD_WIDTH-1:0] DecodePC  = (d_Bubble & ~m_Kill) ? d_PC    : f_PC;






    // performance counters

`ifdef ENABLE_COUNTER
    reg [1:0] CsrFromCounter;
    always @* begin
        case ({d_Insn[31:28], 1'b0, d_Insn[26:20]}) // ignore bit 27 (low or high word)
            12'hB00: CsrFromCounter <= 2'b01; // MCYCLE
            12'hC00: CsrFromCounter <= 2'b01; // CYCLE;
            12'hC01: CsrFromCounter <= 2'b01; // TIME;
            12'hC02: CsrFromCounter <= 2'b10; // INSTRET;
            default: CsrFromCounter <= 0;
        endcase
    end

    wire Retired = ~d_Bubble & ~m_Kill & ~w_Kill;
        // For the number of retired instructions, do not count bubbles or
        // killed instructions. In the execute stage that can be decided.
    wire [32:0] CounterCYCLE    = {1'b0, e_CounterCYCLE} + 1;
    wire [31:0] CounterCYCLEH   = e_CounterCYCLEH + e_CarryCYCLE;
    wire [32:0] CounterINSTRET  = {1'b0, e_CounterINSTRET} + {62'b0, Retired};
    wire [31:0] CounterINSTRETH = e_CounterINSTRETH + e_CarryINSTRET;

    wire [WORD_WIDTH-1:0] vCsrCYCLE   = m_CsrFromCounter[0] 
        ? (m_CsrSelHighWord ? e_CounterCYCLEH : e_CounterCYCLE) : 0;
    wire [WORD_WIDTH-1:0] vCsrINSTRET = m_CsrFromCounter[1] 
        ? (m_CsrSelHighWord ? e_CounterINSTRETH : e_CounterINSTRET) : 0;

`else
    wire [WORD_WIDTH-1:0] vCsrCYCLE   = 0;
    wire [WORD_WIDTH-1:0] vCsrINSTRET = 0;
    wire [1:0]  CsrFromCounter = 0;
    wire [1:0]  m_CsrFromCounter = 0;
`endif







    // CSRs

/*  Doesn't work because MSTATUS=300h may be modified by the CRT of riscv-tests
    // 300h..307h -> x32..x39
    // 340h..347h -> x40..x47
    wire [5:0] vCsrTranslate =
        (d_Insn[31:27]==5'b00110 && d_Insn[25:23]==3'b000)
            ? {2'b10, d_Insn[26], d_Insn[22:20]}
            : 0;
*/

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
    end


    wire [1:0] CsrOp = ((SysOpcode & d_Insn[5]) ? d_Insn[13:12] : 2'b00);
    wire InsnCSR = SysOpcode & d_Insn[5] & (d_Insn[13:12]!=0);

    wire [WORD_WIDTH-1:0] CsrUpdate = e_CsrSelImm ? {27'b0, e_CsrImm} : e_A;

    wire [WORD_WIDTH-1:0] vCsrRegSet = ~m_CsrOp[1]
        ? (~m_CsrOp[0] ? 32'h0 : m_CsrUpdate)
        : (~m_CsrOp[0] ? (e_B | m_CsrUpdate) : (e_B & ~m_CsrUpdate));
            // TRY: e_A instead of e_B would also be possible, if vCsrInsn is adjusted
            // e_B is a bypass from the execute stage of the next cycle
    wire [WORD_WIDTH-1:0] vFromALU    = (m_CsrFromReg==0 && m_CsrFromCounter==0) ? m_WrData : 0;

    wire [WORD_WIDTH-1:0] vCsrOrALU = vCsrCYCLE | vCsrINSTRET | vCsrRegSet | vFromALU;




    // exception handling


    wire ExcUser        = (SysOpcode & PrivOpcode & ~d_Insn[22] & ~d_Insn[21]);
    wire ExcJump        = m_Kill & f_PC[1];
    wire vWriteMEPC     = ExcJump | m_ExcUser | m_ExcMem;
    wire WriteMCAUSE    = ExcJump | m_ExcUser            | w_ExcMem;
    wire WriteMTVAL     = d_ExcJump           | m_ExcMem;

    wire vExcOverwrite  = vWriteMEPC | m_WriteMTVAL | m_WriteMCAUSE;
//    wire vExcOverwrite  = m_Kill & f_PC[1] | m_ExcUser | m_ExcMem | m_WriteMTVAL | m_WriteMCAUSE;
    wire MemWrEn        = m_WrEn | vExcOverwrite;

    wire [5:0] MemWrNo  =
        vWriteMEPC      ? REG_CSR_MEPC :
        (m_WriteMTVAL   ? REG_CSR_MTVAL :
        (m_WriteMCAUSE  ? REG_CSR_MCAUSE :
        m_WrNo));


//    wire [3:0] Cause = m_ExcMem  ? (m_MemWr         ? 6 : 4) :
//                      (e_ExcUser ? (e_EBREAKorECALL ? 3 : 11) : 0);
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

    wire [WORD_WIDTH-1:0] MemWriteData = {
         e_MemWidth[1] ? e_B[31:24] : (e_MemWidth[0]  ? e_B[15:8] : e_B[7:0]),
         e_MemWidth[1] ? e_B[23:16]                               : e_B[7:0],
        (e_MemWidth[1] |               e_MemWidth[0]) ? e_B[15:8] : e_B[7:0],
                                                                    e_B[7:0]};

    assign mem_wren  = e_MemWr & ~DualKill;
    assign mem_wmask = MemSignals[3:0];
    assign mem_wdata = MemWriteData;
    assign mem_addr  = MemAddr;




    reg [31:0] Insn;
    always @* begin
        if (Bubble | ExcJump) begin
            //if (~ExcUser & ~ExcJump & ~MemAccess & ~InsnMRET)
            if (InsnCSR)
                Insn <= {7'b0000000, vCsrTranslate[4:0],
                         5'b00000,
                         3'b110, d_Insn[11:7], 7'b0110011};
            else
                Insn <= {7'b0000000, 5'b00000, 
                         (InsnMRET ? REG_CSR_MEPC[4:0] : REG_CSR_MTVEC[4:0]), 
                         3'b000, 5'b00000, 7'b1100111};
        end else if ((d_Bubble | d_SaveFetch) & ~m_Kill)
            Insn <= d_DelayedInsn;
        else
            Insn <= mem_rdata;
    end

    wire RdNo1Aux = Bubble | ExcJump;
    wire RdNo2Aux = InsnCSR;









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
        e_InsnJALorFENCEI <= InsnJALorFENCEI;

        e_EnShift <= EnShift;
        e_ShiftArith <= ShiftArith;
        e_ReturnPC <= InsnJALorJALR;
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





`ifdef ENABLE_COUNTER
            // CSR decode
            e_CarryCYCLE        <= CounterCYCLE[32];
            e_CounterCYCLE      <= CounterCYCLE[31:0];
            e_CounterCYCLEH     <= CounterCYCLEH;
            e_CarryINSTRET      <= CounterINSTRET[32];
            e_CounterINSTRET    <= CounterINSTRET[31:0];
            e_CounterINSTRETH   <= CounterINSTRETH;
            e_CsrSelHighWord    <= d_Insn[27];
            m_CsrSelHighWord    <= e_CsrSelHighWord;
            e_CsrFromCounter    <= InsnCSR ? CsrFromCounter : 0;
            m_CsrFromCounter    <= e_CsrFromCounter;
`endif

            e_CsrImm            <= d_Insn[19:15];
            m_CsrUpdate         <= CsrUpdate;
            e_CsrOp             <= CsrOp;
            m_CsrOp             <= e_CsrOp;
            e_CsrFromReg        <= CsrFromReg;
            m_CsrFromReg        <= e_CsrFromReg;



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
        $display("E a=%h b=%h i=%h -> %h -> x%d wren=%b",
            e_A, e_B, e_Imm, ALUResult, e_WrNo, e_WrEn);
        $display("E logic=%h pc=%h ui=%h e_SelSum=%b e_EnShift=%b",
            vLogicResult, vPCResult, vUIResult, e_SelSum, e_EnShift);

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

        $display("Z AddrSum=%h NextOrSum=%h NoBranch=%h",
            AddrSum, NextOrSum, NoBranch);


        $display("C vCsrTranslate=%h RdNo2Aux=%b CsrOp=%b m_CsrFromCounter=%b",
            vCsrTranslate, RdNo2Aux, CsrOp, m_CsrFromCounter);
        $display("C vCsrOrALU=%h CsrResult=%h",
            vCsrOrALU, CsrResult);
        $display("M MemResult=%h m_MemSign=%b m_MemByte=%b",
            MemResult, m_MemSign, m_MemByte);


        $display("  e_WrNo=%d e_CsrFromCounter=%b ExecuteWrNo=%d m_WrNo=%d",
            e_WrNo, e_CsrFromCounter, ExecuteWrNo, m_WrNo);
        $display("  DestReg0=%b DisableWrite=%b EnableWrite2=%b DecodeWrEn=%b ExecuteWrEn=%b MemWrEn=%b",
            DestReg0, DisableWrite, EnableWrite2, DecodeWrEn, ExecuteWrEn, MemWrEn);


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
        end
    end

endmodule
