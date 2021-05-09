module Pipeline #(
    parameter [31:0] START_PC = 0
) (
    input  clk,
    input  rstn,

    input  irq_software,
    input  irq_timer,
    input  irq_external,
    output retired,

`ifdef RISCV_FORMAL
    output reg        rvfi_valid,
    output reg [63:0] rvfi_order,
    output reg [31:0] rvfi_insn,
    output reg        rvfi_trap,
    output reg        rvfi_halt,
    output reg        rvfi_intr,
    output reg [ 1:0] rvfi_mode,
    output reg [ 1:0] rvfi_ixl,
    output reg [ 4:0] rvfi_rs1_addr,
    output reg [ 4:0] rvfi_rs2_addr,
    output reg [31:0] rvfi_rs1_rdata,
    output reg [31:0] rvfi_rs2_rdata,
    output reg [ 4:0] rvfi_rd_addr,
    output reg [31:0] rvfi_rd_wdata,
    output reg [31:0] rvfi_pc_rdata,
    output reg [31:0] rvfi_pc_wdata,
    output reg [31:0] rvfi_mem_addr,
    output reg [ 3:0] rvfi_mem_rmask,
    output reg [ 3:0] rvfi_mem_wmask,
    output reg [31:0] rvfi_mem_rdata,
    output reg [31:0] rvfi_mem_wdata,
`endif

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
    input mem_rgrubby,

    output        regset_we,
    output  [5:0] regset_wa,
    output [31:0] regset_wd,
    output        regset_wg,
    output  [5:0] regset_ra1,
    output  [5:0] regset_ra2,
    input  [31:0] regset_rd1,
    input         regset_rg1,
    input  [31:0] regset_rd2,
    input         regset_rg2

);
    localparam integer WORD_WIDTH = 32;

    localparam [5:0] REG_CSR_MTVEC    = 6'b100101;
    localparam [5:0] REG_CSR_MSCRATCH = 6'b110000;
    localparam [5:0] REG_CSR_MEPC     = 6'b110001;
    localparam [5:0] REG_CSR_MCAUSE   = 6'b110010;
    localparam [5:0] REG_CSR_MTVAL    = 6'b110011;


// suffixes
// _i input of module
// _o output of module
// _q register output
// _d signal that feeds a register input
// _v variable inside a process
// _w wire: intermediate signal that connects signals or registers


// ---------------------------------------------------------------------
// real registers
// ---------------------------------------------------------------------


    // fetch
    reg [WORD_WIDTH-1:0] FetchAddr_q;

    // decode
    reg [31:0] Insn_q;
    reg [31:0] InsnE_q;
    reg [31:0] InsnM_q;
    reg [5:0] d_RdNo1;
    reg [5:0] d_RdNo2;

    reg MC_q;
    reg [7:0] MCState_q;
    reg [5:0] MCAux_q;
    reg [4:0] MCRegNo_q;
    reg [4:0] CsrTranslateE_q;

    reg [WORD_WIDTH-1:0] PC_q;
    reg [31:0] DelayedInsn_q;
    reg BubbleM_q;
    reg BubbleE_q;

    reg [31:0] PartialInsn_q;
    reg OddPC_q;
    reg FetchAgainE_q;
    reg FetchAgainM_q;

    // mul/div
    reg StartMulDiv_q;

    reg DivSigned_q;            // control signals for first cycle
    reg MulASigned_q;
    reg InsnMULH_q;

    reg SelRemOrDiv_q;          // control signals for every cycle
    reg SelDivOrMul_q;

    reg SelMulLowOrHigh_q;      // control signals for last cycle
    reg MulDivResNeg_q;
    reg MulDivResAdd_q;

    reg [WORD_WIDTH-1:0] DivQuot_q;
    reg [WORD_WIDTH:0] DivRem_q;
    reg [2*WORD_WIDTH:0] Long_q;

    // execute
    reg e_InsnJALR;
    reg e_InsnBEQ;
    reg e_InsnBLTorBLTU;
    reg InvertBranch_q;
    reg [1:0] e_SelLogic;
    reg e_EnShift;
    reg ShiftRight_q;
    reg ShiftArith_q;
    reg e_ReturnPC;
    reg e_ReturnUI;
    reg e_BranchUncondPCRel;

    reg e_SetCond;
    reg e_LTU;
    reg e_SelSum;

    reg e_AddrFromSum;
    reg e_MemStore;
    reg [1:0] e_MemWidth;

    reg [WORD_WIDTH-1:0] e_A;
    reg [WORD_WIDTH-1:0] e_B;
    reg [WORD_WIDTH-1:0] ImmUpper_q;
    reg [11:0] Imm12PlusReg_q;
    reg [WORD_WIDTH-1:0] e_PCImm;

    reg e_NegB;
    reg e_WrEn;
    reg [5:0] e_WrNo;

    reg  e_InsnBit14;
    wire e_CsrSelImm       = e_InsnBit14;

    // mem stage
    reg m_Kill; // to decode and execute stage
    reg m_WrEn;
    reg [5:0] m_WrNo;
    reg [WORD_WIDTH-1:0] m_WrData;
    reg MemSignedLoadM_q;
    reg [1:0] MemWidthM_q;
    reg [1:0] AddrOfsM_q;
    reg [WORD_WIDTH-1:0] MemRData_q;
    reg [WORD_WIDTH-1:0] MemWData_q;
    //reg [WORD_WIDTH-1:0] MemRDataPrev_q;
    reg MemMisaligned_q;
    reg [WORD_WIDTH-1:0] AddrSum_q;

    // write back
    reg w_WrEn;
    reg [5:0] w_WrNo;
    reg [WORD_WIDTH-1:0] w_WrData;

    reg RetiredE_q;
    reg RetiredM_q;

    // exceptions
    reg [WORD_WIDTH-1:0] PrevPC_q;
    reg OverwriteE_q;
    reg [WORD_WIDTH-1:0] OverwriteValE_q;
    reg OverwriteM_q;
    reg [WORD_WIDTH-1:0] OverwriteValM_q;
    reg OverwriteSelE_q;
    reg InsnMRET_E_q;
    reg InsnMRET_M_q;
    reg ExcInvalidInsn_q;

    // CSRs for exceptions
    reg [WORD_WIDTH-1:0] m_CsrUpdate;
    reg [1:0] e_CsrOp;
    reg [1:0] m_CsrOp;
    reg [WORD_WIDTH-1:0] m_CsrModified;
    reg CsrFromRegE_q;
    reg CsrFromRegM_q;
    reg CsrFromRegW_q;
    reg CsrFromExtE_q;
    reg CsrFromExtM_q;
    reg [WORD_WIDTH-1:0] CsrRDataInternal_q;

    reg f_MModeIntEnable;        // mstatus.mie
    reg f_MModePriorIntEnable;   // mstatus.mpie
    reg IrqResponse_q;
    reg NoIrq_q;
    reg SoftwareIrq_q;
    reg TimerIrq_q;      // external pin high
    reg ExternalIrq_q;




// ---------------------------------------------------------------------
// combinational circuits
// ---------------------------------------------------------------------




    // fetch
    // set instruction word for decode
    wire BubbleE_d = BubbleE_q & ~m_Kill;






    // modify MSTATUS
    reg MModeIntEnable;
    reg MModePriorIntEnable;
    always @* begin
        MModeIntEnable = f_MModeIntEnable;
        MModePriorIntEnable = f_MModePriorIntEnable;
        if (e_CsrAddr==12'h300) begin  // mstatus
            case (e_CsrModify)
                3'b001: begin // write
                    MModeIntEnable      = e_CsrWData[3];
                    MModePriorIntEnable = e_CsrWData[7];
                end
                3'b010: begin // set
                    MModeIntEnable      = f_MModeIntEnable      | e_CsrWData[3];
                    MModePriorIntEnable = f_MModePriorIntEnable | e_CsrWData[7];
                end
                3'b011: begin // clear
//                    MModeIntEnable      = f_MModeIntEnable      & ~e_CsrWData[3];
//                    MModePriorIntEnable = f_MModePriorIntEnable & ~e_CsrWData[7];
// avoid Microsemi Libero bug
                    MModeIntEnable      = e_CsrWData[3] ? 1'b0 : f_MModeIntEnable;
                    MModePriorIntEnable = e_CsrWData[7] ? 1'b0 : f_MModePriorIntEnable;
                end
                default: begin  // do not alter
                end
            endcase
        end
        if (TrapEnter_d) begin
            MModeIntEnable = 0;
            MModePriorIntEnable = f_MModeIntEnable;
        end
        if (InsnMRET_M_q) begin
            MModeIntEnable = f_MModePriorIntEnable;
            MModePriorIntEnable = 1;
        end
    end





    // decode


    wire [WORD_WIDTH-1:0] ImmI_w = {{21{Insn_q[31]}}, Insn_q[30:20]};

    //                                        31..10| 9| 8| 7| 6| 5| 4| 3|2|1|0
    // ImmCIW for C.ADDI4SPN   (opcode 000:00)   -  |10| 9| 8| 7|12|11| 5|6|-|-
    // ImmCI for C.ADDI        (opcode 000:01)  12  |12|12|12|12|12| 6| 5|4|3|2
    // ImmCI for C.LI          (opcode 010:01)  12  |12|12|12|12|12| 6| 5|4|3|2
    //           C.ADDI16SP    (opcode 011:01)  12  |12| 4| 3| 5| 2| 6| -|-|-|-
    // ImmCI for arith         (opcode 100:01)  12  |12|12|12|12|12| 6| 5|4|3|2
    //       for C.SLLI        (opcode 000:10)   ?  | ?| ?| ?| ?| ?| 6| 5|4|3|2
    wire [WORD_WIDTH-1:0] ImmCI_w = {
        Insn_q[0] ? {22{Insn_q[12]}} : 22'b0,
        Insn_q[0] ? Insn_q[12] : Insn_q[10],
        ~Insn_q[0] ? Insn_q[9:7] : Insn_q[13] ? {Insn_q[4:3], Insn_q[5]} : {3{Insn_q[12]}},
        Insn_q[13] ? Insn_q[2] : Insn_q[12],
        (Insn_q[1] | Insn_q[0]) ? Insn_q[6] : Insn_q[11],
        Insn_q[13] ? 1'b0 : Insn_q[5],
        (~Insn_q[1] & ~Insn_q[0]) ? Insn_q[6] : Insn_q[13] ? 1'b0 : Insn_q[4],
        ((~Insn_q[1] & ~Insn_q[0]) | Insn_q[13]) ? 2'b00 : Insn_q[3:2]};

    wire [WORD_WIDTH-1:0] ImmALU_w = (Insn_q[1] & Insn_q[0]) ? ImmI_w : ImmCI_w;
        // Immediate for ALU input B


    //                                      31..17|16..12|11..5|4..0
    //        CSR     (opcode xxx:11100:11) 31..17|16..12|  ?  | ?    only 19..15
    // ImmU   LUI     (opcode xxx:01101:11) 31..17|16..12|  -  | -
    // ImmCIW C.LUI   (opcode 011:xxxxx:01)   12  | 6..2 |  -  | -
    // ImmCI  C.LI    (opcode 010:xxxxx:01)   12  |  12  | 12  |6..2
    // ImmU+PC AUIPC  (opcode xxx:00101:11)
    wire [WORD_WIDTH-1:0] ImmUpper2_w = {
        Insn_q[1] ? Insn_q[31:17] : {15{Insn_q[12]}},
        Insn_q[1] ? Insn_q[16:13] : Insn_q[13] ? Insn_q[6:3] : {4{Insn_q[12]}},
        (~Insn_q[1] &  Insn_q[13]) ? Insn_q[2] : Insn_q[12],
        (~Insn_q[1] & ~Insn_q[13]) ? {{7{Insn_q[12]}}, Insn_q[6:2]} : 12'b0
    };
    wire [WORD_WIDTH-1:0] ImmUpper_w =
        (Insn_q[1] & ~Insn_q[5]) ? (PC_q + {Insn_q[31:12], 12'b0}) : ImmUpper2_w;



    //                                       11..8| 7| 6| 5| 4| 3| 2| 1| 0
    // ImmI for JALR  (opcode xxx:11011:11) 31..28|27|26|25|24|23|22|21|20
    // ImmI for load  (opcode xxx:00000:11) 31..28|27|26|25|24|23|22|21|20
    // ImmS for store (opcode xxx:01000:11) 31..28|27|26|25|11|10| 9| 8| 7
    // C.LW and C.SW  (opcode x10:xxxxx:00)    -  | -| 5|12|11|10| 6| -| -
    // C.LWSP         (opcode 010:xxxxx:10)    -  | 3| 2|12| 6| 5| 4| -| -
    // C.SWSP         (opcode 110:xxxxx:10)    -  | 8| 7|12|11|10| 9| -| -
    // C.JR/JALR      (opcode 100:00000:10)    -  | -| -| -| -| -| -| -| -
    wire [11:0] Imm12PlusReg2_w = {
        (Insn_q[1] & Insn_q[0]) ? Insn_q[31:28] : 4'b0000,
        ~Insn_q[1] ?      1'b0 : Insn_q[0] ? Insn_q[27] : Insn_q[15] ? Insn_q[8] : Insn_q[3],
        ~Insn_q[1] ? Insn_q[5] : Insn_q[0] ? Insn_q[26] : Insn_q[15] ? Insn_q[7] : Insn_q[2],
        (Insn_q[1] & Insn_q[0]) ? Insn_q[25] : Insn_q[12],
        Insn_q[0] ? ((Insn_q[5] & ~Insn_q[6])  ? Insn_q[11:10] : Insn_q[24:23])
                  : ((Insn_q[1] & ~Insn_q[15]) ? Insn_q[6:5]   : Insn_q[11:10]),
        Insn_q[0] ? ((Insn_q[5] & ~Insn_q[6])  ? Insn_q[9] : Insn_q[22])
                  : (~Insn_q[1] ? Insn_q[6] : Insn_q[15] ? Insn_q[9] : Insn_q[4]),
        Insn_q[0] ? ((Insn_q[5] & ~Insn_q[6])  ? Insn_q[8:7] : Insn_q[21:20])
                  : 2'b00
    };

    wire [11:0] Imm12PlusReg_w = (~Insn_q[0] & Insn_q[1] & ~Insn_q[14])
        ? 0 // C.JR or C.JALR
        : Imm12PlusReg2_w;




/*
    //                          31..20|19..12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1|0
    // ImmJ JAL    (xxx:11011:11) 31  |19..12|20|30|29|28|27|26|25|24|23|22|21|-
    // ImmB branch (xxx:11000:11) 31  |  31  | 7|30|29|28|27|26|25|11|10| 9| 8|-
    // 4  FENCE.I  (xxx:00011:11)  -  |     -| -| -| -| -| -| -| -| -| -| X| -|-
    // C.J/C.JAL   (x01:xxxxx:10) 12  |  12  |12| 8|10| 9| 6| 7| 2|11| 5| 4| 3|-
    // C.BEQZ/BNEZ (11x:xxxxx:10) 12  |  12  |12|12|12|12| 6| 5| 2|11|10| 4| 3|-
    wire [20:0] Imm21PCRel_w = {
        ~Insn_q[0] ? Insn_q[12] : (Insn_q[5] & Insn_q[31]), // 20
        ~Insn_q[0] ? {8{Insn_q[12]}} : ~Insn_q[5] ? 8'b0
            : Insn_q[2] ? Insn_q[19:12] : {8{Insn_q[31]}}, // 19..12
        ~Insn_q[0] ? Insn_q[12] : ~Insn_q[5] ? 1'b0
            : Insn_q[2] ? Insn_q[20] : Insn_q[7], // 11
        ~Insn_q[0] ? (Insn_q[14] ? {3{Insn_q[12]}} : {Insn_q[8], Insn_q[10], Insn_q[9]})
                   : (Insn_q[5] ? Insn_q[30:28] : 3'b0), // 10..8
        ~Insn_q[0] ? Insn_q[6]
                   : (Insn_q[5] & Insn_q[27]), // 7
        ~Insn_q[0] ? (Insn_q[14] ? Insn_q[12] : Insn_q[8])
                   : (Insn_q[5] & Insn_q[30]), // 6
        ~Insn_q[0] ? Insn_q[2]
                   : (Insn_q[5] & Insn_q[25]), // 5
*/


    //                          31..20|19..12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1|0
    // JAL         (xxx:11011:11) 31  |19..12|20|30|29|28|27|26|25|24|23|22|21|-
    // branch      (xxx:11000:11) 31  |  31  | 7|30|29|28|27|26|25|11|10| 9| 8|-
    // 4  FENCE.I  (xxx:00011:11)  -  |     -| -| -| -| -| -| -| -| -| -| X| -|-
    // C.BEQZ/BNEZ (11x:xxxxx:01) 12  |  12  |12|12|12|12| 6| 5| 2|11|10| 4| 3|-
    // C.J/C.JAL   (x01:xxxxx:01) 12  |  12  |12| 8|10| 9| 6| 7| 2|11| 5| 4| 3|-
    wire [20:0] ImmJ_w = {Insn_q[31], Insn_q[19:12], Insn_q[20], Insn_q[30:21], 1'b0};
    wire [20:0] ImmB_w = {{9{Insn_q[31]}}, Insn_q[7], Insn_q[30:25], Insn_q[11:8], 1'b0};
    wire [20:0] ImmCB_w = {{13{Insn_q[12]}}, Insn_q[6], Insn_q[5], Insn_q[2],
        Insn_q[11], Insn_q[10], Insn_q[4:3], 1'b0};
    wire [20:0] ImmCJ_w = {{10{Insn_q[12]}}, Insn_q[8], Insn_q[10], Insn_q[9],
        Insn_q[6], Insn_q[7], Insn_q[2], Insn_q[11], Insn_q[5:3], 1'b0};

    wire [20:0] Imm21PCRel_w =
        Insn_q[1] ? (Insn_q[5]  ? (Insn_q[2] ? ImmJ_w  // JAL
                                             : ImmB_w) // branch
                                : 21'h4)               // FENCE.I
                  : (Insn_q[14] ? ImmCB_w              // C.branch
                                : ImmCJ_w);            // C.jump

    wire [WORD_WIDTH-1:0] PCImm = PC_q + {{11{Imm21PCRel_w[20]}}, Imm21PCRel_w};







    wire ShiftRight_d   = (Insn_q[1] & Insn_q[0]) ? Insn_q[14] : Insn_q[15];
    wire ShiftArith_d   = (Insn_q[1] & Insn_q[0]) ? Insn_q[30] : Insn_q[10];
    wire InvertBranch_d = (Insn_q[1] & Insn_q[0])
        ? (Insn_q[6] & Insn_q[12]) 
            // set for BNE or BGE or BGEU, but not for SLT..SLTIU
        : Insn_q[13];
            // set for C.BNEZ, but not for C.BEQZ

    wire LTU = Insn_q[6] ? Insn_q[13] : Insn_q[12];
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
        // for all other opcodes, LTU and InvertBranch don't mind





    wire IrqResponse_d = f_MModeIntEnable &
        (SoftwareIrq_q | TimerIrq_q | ExternalIrq_q) &
        ~NoIrq_d;    // no interrup the 2nd cycle after a jump or branch


    localparam [7:0] mcNop = 0;
    localparam [7:0] mcCsr = 1;
    localparam [7:0] mcMem = 2;
    localparam [7:0] mcWFI = 3;
    localparam [7:0] mcJumpReg = 4;
    localparam [7:0] mcException = 5;
    localparam [7:0] mcExcWrCause            = 10;
    localparam [7:0] mcMulDiv = 6;
    localparam [7:0] mcMulDivWriteback = 7;
    localparam [7:0] mcMulDivLast = 8;
    localparam [7:0] mcUnalignedJump = 9;
    localparam [7:0] mcMem2 = 11;
    localparam [7:0] mcLoad = 12;
    localparam [7:0] mcStore = 13;


    // Fetching
    reg [31:0] Insn_d;
    reg [5:0] RdNo1;
    reg [5:0] RdNo2;
    reg [31:0] DelayedInsn_d;

    reg [WORD_WIDTH-1:0] FetchAddr_d;
    reg [WORD_WIDTH-1:0] PC_d;
    reg [31:0] PartialInsn_d;
    reg OddPC_d;
    reg RealignedPC_d;
    reg RVCInsn_w;
    reg FetchAgainD_d;

    // Decoding
    reg ExcInvalidInsn;
    reg RetiredD_d;
    reg DecodeWrEn;
    reg [5:0] DecodeWrNo;

    reg InsnCSR;
    reg CsrFromExtD_d;
    reg [1:0] CsrOp;
    reg BubbleD_d;

    reg InsnBEQ;
    reg InsnBLTorBLTU;
    reg InsnJALR;
    reg BranchUncondPCRel;
    reg ReturnPC;
    reg NoIrq_d;

    reg NegB;
    reg SelSum;
    reg SetCond;
    reg EnShift;
    reg vSelImm;
    reg [1:0] SelLogic;
    reg ReturnUI;

    reg AddrFromSum;
    reg MemStore;
    reg [1:0] MemWidth;

    reg [5:0] ModRdNo1;
    reg MC_d;
    reg [7:0] MCState_d;
    reg [5:0] MCAux_d;
    reg [4:0] MCRegNo_d;

    reg [WORD_WIDTH-1:0] ImmUpper_d;
    reg [11:0] Imm12PlusReg_d;
    reg Overwrite_d;              // overwrite result in M-stage (used by exceptions)
    reg [WORD_WIDTH-1:0] OverwriteVal_d;
    reg OverwriteSelD_d;
    reg InsnMRET_d;
    reg TrapEnter_d;

    reg StartMulDiv_d;
    reg SelRemOrDiv_d;
    reg SelDivOrMul_d;
    reg SelMulLowOrHigh_d;

    always @* begin


        ////////////
        // decode
        ////////////


        ExcInvalidInsn = 1;
        RetiredD_d = 0;
        DecodeWrEn = 0;
        DecodeWrNo = 0; // don't care

        FetchAddr_d = FetchAddr_q;
        PC_d = PC_q;
        OddPC_d = OddPC_q;
        RealignedPC_d = 0;
        RdNo1 = 0;
        RVCInsn_w = 0;
        FetchAgainD_d = 0;

        InsnCSR = 0;
        CsrFromExtD_d = 0;
        CsrOp = 0;
        BubbleD_d = 0;

        InsnBEQ         = 0;
        InsnBLTorBLTU   = 0;
        InsnJALR        = 0;
        BranchUncondPCRel    = 0;
        ReturnPC        = 0;
        NoIrq_d           = 0;
            // don't allow interrupt in 2nd cycle, because PC_q is
            // not set correctly, which will result in a wrong MEPC

        NegB            = 0;
        SelSum          = 0;
        SetCond         = 0;
        EnShift         = 0;
        vSelImm         = 0;
        SelLogic        = 2'b01;
        ReturnUI        = 0;

        AddrFromSum     = 0;
        MemStore        = 0;
        MemWidth        = 2'b11; // no memory access

        ModRdNo1        = 0; // don't change the register read in the next cycle
        MC_d            = 0;
        MCState_d       = 0;
        MCAux_d         = MCAux_q - 1;
        MCRegNo_d       = MCRegNo_q;

        ImmUpper_d       = ImmUpper_w; // used for AUIPC, LUI, C.LUI, C.LI and CSR
        Imm12PlusReg_d  = Imm12PlusReg_w; // used for memory accesses and JALR

        Overwrite_d     = 0;
        OverwriteVal_d  = PC_q;
        OverwriteSelD_d = 0;
        TrapEnter_d     = 0;
        InsnMRET_d      = 0;

        StartMulDiv_d   = 0;
        SelRemOrDiv_d   = SelRemOrDiv_q;
        SelDivOrMul_d   = SelDivOrMul_q;
        SelMulLowOrHigh_d = SelMulLowOrHigh_q;


        if (m_Kill) begin
            FetchAddr_d = FetchAddr_q + 4; // continue fetching
            PC_d = FetchAddr_q;

            OddPC_d = FetchAddr_q[1];
            if (FetchAddr_q[1]) begin
                FetchAddr_d[1] = 0;
                MC_d = 1;
                MCState_d = mcUnalignedJump;
            end
            ExcInvalidInsn = 0;

        end else if (ExcInvalidInsn_q) begin
            ExcInvalidInsn = 0;
            Overwrite_d = 1;
            OverwriteVal_d = PrevPC_q;
            DecodeWrNo = REG_CSR_MEPC;
            RdNo1 = REG_CSR_MTVEC;
            MC_d = 1;
            MCState_d = mcException;
            MCAux_d = 6'h02 + 1;
            TrapEnter_d = 1;

        // microcoded cycles of multi-cycle instructions
        end else if (MC_q) begin
            ExcInvalidInsn = 0;
            case (MCState_q)
                mcNop: begin
                end
                mcCsr: begin // 2nd cycle of CSR access
                    DecodeWrEn     = CsrFromRegE_q;
                    DecodeWrNo     = {1'b1, CsrTranslateE_q};
                end
                mcMem: begin // 2nd cycle of memory access
                    FetchAgainD_d = 1;
                    BubbleD_d = 1;
                    MC_d = 1;
                    MCState_d = mcMem2;

                end
                mcWFI: begin // WFI
                    if (~IrqResponse_q) begin
                        MC_d = 1;
                        MCState_d = mcWFI;
                    end
                end
                mcJumpReg: begin // MRET
                    // jump to adddress in first register (MTVEC)
                    NoIrq_d       = 1;
                    InsnJALR    = 1;
                    AddrFromSum = 1;
                    Imm12PlusReg_d = 0;
                end
                mcException: begin 
                    // jump to adddress in first register (MEPC) and write MTVAL
                    NoIrq_d = 1;
                    InsnJALR = 1;
                    AddrFromSum = 1;
                    Imm12PlusReg_d = 0;
                    Overwrite_d = 1;
                    if (MCAux_q==(6'h03 + 1)) begin
                        // EBREAK exception: MTVAL=PC
                        OverwriteVal_d = OverwriteValE_q;
                    end else if (MCAux_q==(6'h02 + 1)) begin
                        // Illegal instruction exception: MTVAL=insn
                        OverwriteVal_d = InsnM_q;
                    end else begin
                        OverwriteVal_d = 0;
                    end
                    DecodeWrNo = REG_CSR_MTVAL;
                    MC_d = 1;
                    MCState_d = mcExcWrCause;
                end
                mcExcWrCause: begin 
                    // write MCAUSE
                    NoIrq_d = 1;
                    Overwrite_d = 1;
                    OverwriteVal_d = {MCAux_q[4], {(WORD_WIDTH-5){1'b0}}, MCAux_q[3:0]};
                    DecodeWrNo = REG_CSR_MCAUSE;
                end
                mcMulDiv: begin
                    MC_d = 1;
                    if (MCAux_q != 0) begin
                        MCState_d = mcMulDiv;
                    end else begin
                        MCState_d = mcMulDivWriteback;
                    end
                end
                mcMulDivWriteback: begin
                    DecodeWrEn = (MCRegNo_q!=0); // OPTIMISE
                        // required for forwarding
                    DecodeWrNo = {1'b0, MCRegNo_q};
                    Overwrite_d = (MCRegNo_q!=0); // OPTIMISE
                    OverwriteSelD_d = 1;
                    MC_d = 1;
                    MCState_d = mcMulDivLast;
                end
                mcMulDivLast: begin
                    // just do nothing because result is not yet available
                    FetchAddr_d = FetchAddr_q + 4;
                end
                mcUnalignedJump: begin
                    // wait one cycle because of unaligned jump target
                    FetchAddr_d = FetchAddr_q + 4;
                end

                mcLoad: begin
                    if (MemMisaligned_d) begin
                        BubbleD_d = 1;
                        AddrFromSum = 1;
                        MemWidth = e_MemWidth;
                        DecodeWrEn = e_WrEn;
                        DecodeWrNo = e_WrNo;
                        MC_d = 1;
                        MCState_d = mcMem;
                    end
                end
                mcStore: begin
                    if (MemMisaligned_d) begin
                        BubbleD_d = 1;
                        AddrFromSum = 1;
                        MemStore = 1;
                        MemWidth = e_MemWidth;
                        DecodeWrEn = 0;
                        MC_d = 1;
                        MCState_d = mcMem;
                    end
                end


                default: begin
                end
            endcase


        end else if (IrqResponse_q) begin
            ExcInvalidInsn = 0;
            if (ExternalIrq_q) begin
                MCAux_d = 6'h1b + 1;
            end else if (SoftwareIrq_q) begin
                MCAux_d = 6'h13 + 1;
            end else begin
                MCAux_d = 6'h17 + 1;
            end
            Overwrite_d = 1;
            OverwriteVal_d = PC_q;
            DecodeWrNo = REG_CSR_MEPC;
            RdNo1 = REG_CSR_MTVEC;
            MC_d = 1;
            MCState_d = mcException;
            TrapEnter_d = 1;



        // 32 bit instruction
        end else if (Insn_q[1:0]==2'b11) begin
            FetchAddr_d = FetchAddr_q + 4;
            PC_d = PC_q + 4;

            DecodeWrEn = (Insn_q[11:7]!=0); // do not write to x0
            DecodeWrNo = {CsrFromRegE_q, Insn_q[11:7]};

            case (Insn_q[6:2])
                5'b01101: begin // LUI
                    ExcInvalidInsn = 0;
                    ReturnUI       = 1;
                end
                5'b00101: begin // AUIPC
                    ExcInvalidInsn = 0;
                    ReturnUI       = 1;
                end
                5'b11011: begin // JAL
                    ExcInvalidInsn = 0;
                    BranchUncondPCRel = 1;
                    ReturnPC       = 1;
                    NoIrq_d          = 1;
                end
                5'b11001: begin // JALR
                    ExcInvalidInsn = (Insn_q[14:12]!=3'b000);
                    ReturnPC       = 1;
                    NoIrq_d          = 1;
                    InsnJALR    = 1;
                    AddrFromSum = 1;
                end
                5'b11000: begin // branch
                    ExcInvalidInsn = (Insn_q[14:13]==2'b01);
                    DecodeWrEn     = 0;
                    InsnBEQ        = (Insn_q[14:13]==2'b00);
                    InsnBLTorBLTU  = Insn_q[14];
                    NoIrq_d          = 1;
                    NegB           = 1;
                    SelLogic       = InsnBEQ ? 2'b00 : 2'b01;
                end
                5'b00000: begin // load
                    ExcInvalidInsn = (Insn_q[13] & (Insn_q[14] | Insn_q[12]));
                    BubbleD_d         = 1;
                    AddrFromSum    = 1;
                    MemWidth       = Insn_q[13:12];
                    MC_d           = 1;
                    MCState_d      = mcLoad;
                end
                5'b01000: begin // store
                    ExcInvalidInsn = (Insn_q[14] | (Insn_q[13] & Insn_q[12]));
                    DecodeWrEn     = 0;
                    BubbleD_d         = 1;
                    AddrFromSum    = 1;
                    MemStore       = 1;
                    MemWidth       = Insn_q[13:12];
                    MC_d           = 1;
                    MCState_d      = mcStore;
                end
                5'b00100: begin // immediate
                    ExcInvalidInsn = ~Insn_q[13] & Insn_q[12] &
                        (Insn_q[31] | (Insn_q[29:25]!=0) |
                        (Insn_q[30] & ~Insn_q[14]));
                    NegB = ~Insn_q[14] & (Insn_q[13] | Insn_q[12]);
                        // SLLI, SLTI, SLTIU
                    SelSum   = ~Insn_q[14] & ~Insn_q[13] & ~Insn_q[12]; // ADDI
                    SetCond  = ~Insn_q[14] &  Insn_q[13]; // SLTI, SLTIU
                    EnShift  = ~Insn_q[13] &  Insn_q[12]; // SLLI, SRAI, SRLI
                    vSelImm  = 1; // only for forwarding
                    SelLogic =  Insn_q[14] ? Insn_q[13:12] : 2'b01;
                        // ANDI, ORI, XORI
                end
                5'b01100: begin
                    if (Insn_q[25]) begin // RVM
                        ExcInvalidInsn = (Insn_q[31:26]!=0);

                        FetchAddr_d = FetchAddr_q;
                            // increment not before last cycle

                        MC_d = 1;
                        MCState_d = mcMulDiv;
                        MCAux_d = 31;
                        MCRegNo_d = Insn_q[11:7];

                        StartMulDiv_d = 1;
                        SelRemOrDiv_d = Insn_q[13] | InsnMULH_d;
                        SelDivOrMul_d = Insn_q[14];
                        SelMulLowOrHigh_d = (Insn_q[13:12]==2'b00);

                    end else begin // arith
                        ExcInvalidInsn = Insn_q[31] | (Insn_q[29:26]!=0) |
                            (Insn_q[30] & (Insn_q[13] | (Insn_q[14]!=Insn_q[12])));
                        NegB     = ~Insn_q[14] & // SUB, SLL, SLT, SLTU
                                   (Insn_q[13] | Insn_q[12] | Insn_q[30]);
                        SelSum   = ~Insn_q[14] & ~Insn_q[13] & ~Insn_q[12]; // ADD, SUB
                        SetCond  = ~Insn_q[14] &  Insn_q[13]; // SLT, SLTU
                        EnShift  = ~Insn_q[13] &  Insn_q[12]; // SLL, SRA, SRL
                        SelLogic =  Insn_q[14] ? Insn_q[13:12] : 2'b01; // AND, OR, XOR
                    end
                end
                5'b00011: begin // fence
                    ExcInvalidInsn = Insn_q[14] | Insn_q[13];
                    DecodeWrEn = 0;
                    BranchUncondPCRel = Insn_q[12];
                end
                5'b11100: begin // system
                    BubbleD_d = 1;
                    MC_d = 1;
                    MCState_d = mcNop;
                    if (Insn_q[13] | Insn_q[12]) begin // RVZicsr
                        ExcInvalidInsn = 1;
                        if (~Insn_q[31] | ~Insn_q[30] // rw-CSR or
                            | (Insn_q[13] & (Insn_q[19:15]==0))) // ro-CSR and CSRRS/C and rs1=0
                        begin
                            ExcInvalidInsn = ~CsrFromRegD_d & ~CsrValid_w;
                            CsrFromExtD_d  = DecodeWrEn & CsrValid_w;
                            DecodeWrEn     = DecodeWrEn & CsrFromRegD_d;
                            InsnCSR        = 1;
                            CsrOp          = Insn_q[13:12];
                            RdNo1          = {1'b1, CsrTranslateD_d};
                            MC_d           = 1;
                            MCState_d      = mcCsr;
                        end
                    end else begin
                        if (Insn_q[19:7]==0) begin
                            ExcInvalidInsn = 0;
                            case (Insn_q[31:20])
                                12'h000: begin // ECALL
                                    Overwrite_d = 1;
                                    OverwriteVal_d = PC_q;
                                    DecodeWrNo = REG_CSR_MEPC;
                                    RdNo1 = REG_CSR_MTVEC;
                                    MC_d = 1;
                                    MCState_d = mcException;
                                    MCAux_d = 6'h0b + 1;
                                    TrapEnter_d = 1;
                                end
                                12'h001: begin // EBREAK
                                    Overwrite_d = 1;
                                    OverwriteVal_d = PC_q;
                                    DecodeWrNo = REG_CSR_MEPC;
                                    RdNo1 = REG_CSR_MTVEC;
                                    MC_d = 1;
                                    MCState_d = mcException;
                                    MCAux_d = 6'h03 + 1;
                                    TrapEnter_d = 1;
                                end
                                12'h002: begin // URET
                                    RdNo1 = REG_CSR_MEPC;
                                    MC_d = 1;
                                    MCState_d = mcJumpReg;
                                    InsnMRET_d = 1;
                                end
                                12'h102: begin // SRET
                                    RdNo1 = REG_CSR_MEPC;
                                    MC_d = 1;
                                    MCState_d = mcJumpReg;
                                    InsnMRET_d = 1;
                                end
                                12'h302: begin // MRET
                                    RdNo1 = REG_CSR_MEPC;
                                    MC_d = 1;
                                    MCState_d = mcJumpReg;
                                    InsnMRET_d = 1;
                                end
                                12'h105: begin // WFI
                                    MC_d = 1;
                                    MCState_d = mcWFI;
                                end
                                default: begin
                                    ExcInvalidInsn = 1;
                                end
                            endcase
                        end
                    end
                end
                default: begin
                end
            endcase
            RetiredD_d = ~ExcInvalidInsn;

        // compressed instruction
        end else begin
            if (OddPC_q) begin
                OddPC_d = 0;
                RealignedPC_d = 1;
            end else begin
                OddPC_d = 1;
                RealignedPC_d = 0;
                FetchAddr_d = FetchAddr_q + 4;
            end
            PC_d = PC_q + 2;
            RVCInsn_w = 1;

            DecodeWrEn = (Insn_q[11:7]!=0);
            DecodeWrNo = {1'b0, Insn_q[11:7]};

            case ({Insn_q[15:14], Insn_q[1:0]})
                4'b0000: begin // C.ADDI4SPN
                    ExcInvalidInsn = Insn_q[13] | (Insn_q[12:2]==0);
                    SelSum = 1;
                    vSelImm = 1;
                    DecodeWrEn = 1;
                    DecodeWrNo = {3'b001, Insn_q[4:2]};
                end
                4'b0100: begin // C.LW
                    ExcInvalidInsn = Insn_q[13];
                    BubbleD_d = 1;
                    AddrFromSum = 1;
                    MemWidth = 2'b10;
                    DecodeWrEn = 1;
                    DecodeWrNo = {3'b001, Insn_q[4:2]};
                    MC_d = 1;
                    MCState_d = mcLoad;
                end
                4'b1100: begin // C.SW
                    ExcInvalidInsn = Insn_q[13];
                    DecodeWrEn = 0;
                    BubbleD_d = 1;
                    AddrFromSum = 1;
                    MemStore = 1;
                    MemWidth = 2'b10;
                    MC_d = 1;
                    MCState_d = mcStore;
                end
                4'b0001: begin
                    ExcInvalidInsn = 0;
                    if (~Insn_q[13]) begin // C.ADDI
                        SelSum = 1;
                        vSelImm = 1;
                    end else begin // C.JAL
                        BranchUncondPCRel = 1;
                        ReturnPC = 1;
                        NoIrq_d = 1;
                        DecodeWrEn = 1;
                        DecodeWrNo = 1;
                    end
                end
                4'b0101: begin
                    ExcInvalidInsn = 0;
                    if (Insn_q[13] & Insn_q[11:7]==2) begin // C.ADDI16SP
                        SelSum = 1;
                        vSelImm = 1;
                    end else begin // C.LI or C.LUI
                        ReturnUI = 1;
                    end
                end
                4'b1001: begin
                    if (~Insn_q[13]) begin
                        ExcInvalidInsn = Insn_q[12] & (Insn_q[11:10]!=2'b10);
                        DecodeWrEn = 1;
                        DecodeWrNo = {3'b001, Insn_q[9:7]};
                        if (Insn_q[11]) begin
                            if (Insn_q[10]) begin
                                case (Insn_q[6:5])
                                    2'b00: begin // C.SUB
                                        NegB = 1;
                                        SelSum = 1;
                                    end
                                    2'b01: begin // C.XOR
                                        SelLogic = 2'b00;
                                    end
                                    2'b10: begin // C.OR
                                        SelLogic = 2'b10;
                                    end
                                    2'b11: begin // C.AND
                                        SelLogic = 2'b11;
                                    end
                                endcase
                            end else begin // C.ANDI
                                SelLogic = 2'b11;
                                vSelImm = 1;
                            end
                        end else begin // C.SRLI and C.SRAI
                            EnShift = 1;
                                // ShiftRight_q and ShiftArith_q are set independently
                            vSelImm = 1;
                        end
                    end else begin // C.J
                        ExcInvalidInsn = 0;
                        DecodeWrEn = 0;
                        BranchUncondPCRel = 1;
                        NoIrq_d = 1;
                    end
                end
                4'b1101: begin // C.BEQZ ot C.BNEZ
                    ExcInvalidInsn = 0;
                    DecodeWrEn = 0;
                    InsnBEQ        = 1;
                        // InvertBranch_q is set independently
                    NoIrq_d        = 1;
                    NegB           = 1;
                    SelLogic       = 2'b00; // xor
                end
                4'b0010: begin // C.SLLI
                    ExcInvalidInsn = Insn_q[13] | Insn_q[12];
                    NegB = 1;
                    EnShift = 1;
                        // ShiftRight_q and ShiftArith_q are set independently
                    vSelImm = 1;
                end
                4'b0110: begin // C.LWSP
                    ExcInvalidInsn = Insn_q[13] | (Insn_q[11:7]==0);
                    BubbleD_d = 1;
                    AddrFromSum = 1;
                    MemWidth = 2'b10;
                    MC_d = 1;
                    MCState_d = mcLoad;
                end
                4'b1010: begin
                    ExcInvalidInsn = Insn_q[13];
                    if (Insn_q[6:2]==0) begin
                        if (Insn_q[12]) begin
                            if (Insn_q[11:7]==0) begin // C.EBREAK
                                Overwrite_d = 1;
                                OverwriteVal_d = PC_q;
                                DecodeWrEn = 1;
                                DecodeWrNo = REG_CSR_MEPC;
                                RdNo1 = REG_CSR_MTVEC;
                                MC_d = 1;
                                MCState_d = mcException;
                                MCAux_d = 6'h03 + 1;
                                TrapEnter_d = 1;
                            end else begin // C.JALR
                                ReturnPC = 1;
                                NoIrq_d = 1;
                                DecodeWrEn = 1;
                                DecodeWrNo = 1;
                                InsnJALR = 1;
                                AddrFromSum = 1;
                            end
                        end else begin // C.JR
                            ExcInvalidInsn = (Insn_q[11:7]==0);
                            DecodeWrEn = 0;
                            NoIrq_d = 1;
                            InsnJALR    = 1;
                            AddrFromSum = 1;
                        end
                    end else begin // C.MV or C.ADD
                        SelSum = 1;
                            // implicit rs1=x0 for C.MV is set in predecode
                    end
                end
                4'b1110: begin // C.SWSP
                    ExcInvalidInsn = Insn_q[13];
                    DecodeWrEn = 0;
                    BubbleD_d = 1;
                    AddrFromSum = 1;
                    MemStore = 1;
                    MemWidth = 2'b10;
                    MC_d = 1;
                    MCState_d = mcStore;
                end
                default: begin
                    ExcInvalidInsn = 1;
                end
            endcase
            RetiredD_d = ~ExcInvalidInsn;
        end



        ////////////
        // fetch
        ////////////

        if (e_InsnJALR & ~m_Kill) begin
            FetchAddr_d = {AddrSum_w[WORD_WIDTH-1:1], 1'b0};
        end
        if (Branch_w) begin
            FetchAddr_d = e_PCImm;
        end

        // 101 MEM-To-MEM
        if (BubbleD_d & BubbleM_q & ~m_Kill) begin
//        if (BubbleD_d & BubbleM_q & ~m_Kill & ~FetchAgainM_q) begin

if (FetchAgainM_q) begin
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {mem_rdata[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = mem_rdata;
            end

        if (RealignedPC_d) begin //4
            DelayedInsn_d = PartialInsn_q;
        end else begin
            DelayedInsn_d       = mem_rdata;
        end

//            if (RealignedPC_d) begin
                PartialInsn_d   = PartialInsn_q;
//            end else begin
//                PartialInsn_d   = mem_rdata;
//            end

end else begin
            if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            if (RealignedPC_d) begin
                DelayedInsn_d       = PartialInsn_q;
            end else begin
                DelayedInsn_d       = DelayedInsn_q;
            end
//3            PartialInsn_d       = DelayedInsn_q;
            PartialInsn_d       = PartialInsn_q;
end


        // 010 POP and PUSH (also 110)
        end else if (BubbleE_q & ~m_Kill) begin
            if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end

if (BubbleD_d) begin
            DelayedInsn_d       = DelayedInsn_q;
//            if (RealignedPC_d) begin
                PartialInsn_d       = PartialInsn_q;
//2            end else begin
//2                PartialInsn_d       = DelayedInsn_q;
//2            end
end else begin
            DelayedInsn_d       = mem_rdata;
            if (RealignedPC_d) begin
                PartialInsn_d       = PartialInsn_q;
            end else begin
                PartialInsn_d       = DelayedInsn_q;
            end
end

        // 000 MC
        end else if (MC_q & (MCState_q!=mcUnalignedJump) & ~m_Kill) begin
            if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            DelayedInsn_d       = DelayedInsn_q;
            if (RealignedPC_d) begin
                PartialInsn_d   = PartialInsn_q;
            end else begin
                PartialInsn_d   = DelayedInsn_q;
            end

        // 100 PUSH
        end else if ((BubbleD_d & ~BubbleM_q) | StartMulDiv_d) begin
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {mem_rdata[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = mem_rdata;
            end
            if (RealignedPC_d) begin
                DelayedInsn_d       = PartialInsn_q;
                PartialInsn_d       = mem_rdata;
            end else if (OddPC_d) begin
                DelayedInsn_d       = mem_rdata;
                PartialInsn_d       = PartialInsn_q;
            end else begin
                DelayedInsn_d       = mem_rdata;
                PartialInsn_d       = mem_rdata;
            end

        // 001 POP
//        end else if ((~BubbleD_d & BubbleM_q) & ~m_Kill) begin
        end else if ((~BubbleD_d & BubbleM_q) & ~m_Kill & ~FetchAgainM_q) begin
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            DelayedInsn_d       = DelayedInsn_q;
            if (RealignedPC_d) begin
                PartialInsn_d       = PartialInsn_q;
            end else begin
                PartialInsn_d       = DelayedInsn_q;
            end


        // 000 NOT DELAYED
        end else begin
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {mem_rdata[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = mem_rdata;
            end
            DelayedInsn_d       = DelayedInsn_q;
            if (RealignedPC_d) begin
                PartialInsn_d   = PartialInsn_q;
            end else begin
                PartialInsn_d   = mem_rdata;
            end
        end






        // register numbers for next insn

        if (~RdNo1[5]) begin

            if (~Insn_d[1]) begin
                if (~Insn_d[0]) begin // 00
                    if (Insn_d[15:13]==3'b000)  RdNo1 = 2;
                    else                        RdNo1 = {3'b001, Insn_d[9:7]};  // RVC rs1'
                end else begin // 01
                    if (~Insn_d[15])            RdNo1 = {1'b0, Insn_d[11:7]};   // RVC rs1
                    else                        RdNo1 = {3'b001, Insn_d[9:7]};  // RVC rs1'
                end
            end else begin
                if (~Insn_d[0]) begin // 10
                    if (Insn_d[14:13]==2'b00) begin
                        // implicit x0 for C.MV
                        if (Insn_d[15] & ~Insn_d[12] & (Insn_d[6:2]!=0))
                                                RdNo1 = 0;
                        else                    RdNo1 = {1'b0, Insn_d[11:7]};   // RVC rs1
                    end else                    RdNo1 = 2;
                end else begin // 11
                                                RdNo1 = {1'b0, Insn_d[19:15]};  // RVI rs1
                end
            end
        end

        if (~Insn_d[1]) begin
            if (Insn_d[0] & Insn_d[14]) begin
                RdNo2 = 0; // implicit x0 for C.BEQZ/BNEZ
            end else begin
                RdNo2 = {3'b001, Insn_d[4:2]};  // RVC rs2'
            end
        end else if (Insn_d[0]) begin
            RdNo2 = {1'b0, Insn_d[24:20]};  // rs2
        end else begin
            RdNo2 = {1'b0, Insn_d[6:2]};  // RVC rs2
        end
    end







    wire ExecuteWrEn   = ~m_Kill & (CsrFromExtE_q | e_WrEn);
    wire [5:0] ExecuteWrNo = e_WrNo;






    // forwarding

    wire FwdAE = e_WrEn & (d_RdNo1 == e_WrNo);
    wire FwdAM = m_WrEn & (d_RdNo1 == m_WrNo);
    wire FwdAW = w_WrEn & (d_RdNo1 == w_WrNo);
    wire [WORD_WIDTH-1:0] ForwardAR = (FwdAE | FwdAM | FwdAW) ? 0 : regset_rd1;
    wire [WORD_WIDTH-1:0] ForwardAM = FwdAM ? MemResult : (FwdAW ? w_WrData : 0);
    wire [WORD_WIDTH-1:0] ForwardAE = FwdAE ? ALUResult : (ForwardAR | ForwardAM);

    wire FwdBE = e_WrEn & (d_RdNo2 == e_WrNo) & ~vSelImm;
    wire FwdBM = m_WrEn & (d_RdNo2 == m_WrNo) & ~vSelImm;
    wire FwdBW = w_WrEn & (d_RdNo2 == w_WrNo);
    wire [WORD_WIDTH-1:0] ForwardImm = vSelImm ? ImmALU_w : 0;
    wire [WORD_WIDTH-1:0] ForwardBR = vSelImm ?    0 : (FwdBW ? w_WrData : regset_rd2);
    wire [WORD_WIDTH-1:0] ForwardBM =  FwdBM ? MemResult : (ForwardBR | ForwardImm);
    wire [WORD_WIDTH-1:0] ForwardBE = (FwdBE ? ALUResult : ForwardBM) ^ {WORD_WIDTH{NegB}};






    // ALU




    wire [WORD_WIDTH-1:0] vLogicResult = ~e_SelLogic[1]
        ? (~e_SelLogic[0] ? (e_A ^ e_B) : 32'h0)
        : (~e_SelLogic[0] ? (e_A | e_B) : (e_A & e_B));
    wire [WORD_WIDTH-1:0] vPCResult =
          (e_ReturnPC ? PC_q : 0);
    wire [WORD_WIDTH-1:0] vUIResult =
        e_ReturnUI ? ImmUpper_q : 0;

    // OPTIMIZE? vFastResult has one input left
    wire [WORD_WIDTH-1:0] vFastResult = 
        vLogicResult | vUIResult | vPCResult;
    wire [WORD_WIDTH-1:0] Sum = e_A + e_B + {{(WORD_WIDTH-1){1'b0}}, e_NegB};
    wire [WORD_WIDTH-1:0] vShiftAlternative = {
        e_SelSum ? Sum[WORD_WIDTH-1:1] :  vFastResult[WORD_WIDTH-1:1],
        e_SelSum ? Sum[0]              : (vFastResult[0] | vCondResultBit)};

    //                         62|61..32|31|30..0
    // SLL (funct3 001)        31|30..1 | 0|  -
    // SRL (funct3 101, i30 0)  -|   -  |31|30..0
    // SRA (funct3 101, i30 1) 31|  31  |31|30..0
    wire [62:0] vShift0 = {
        (ShiftRight_q & ~ShiftArith_q) ? 1'b0 : e_A[31],
        ~ShiftRight_q ? e_A[30:1] : (ShiftArith_q ? {30{e_A[31]}} :  30'b0),
        ~ShiftRight_q ? e_A[0] : e_A[31],
        ~ShiftRight_q ? 31'b0 : e_A[30:0]};

    wire [46:0] vShift1 = e_B[4] ? vShift0[62:16] : vShift0[46:0];
    wire [38:0] vShift2 = e_B[3] ? vShift1[46:8]  : vShift1[38:0];
    wire [34:0] vShift3 = e_B[2] ? vShift2[38:4]  : vShift2[34:0];
    wire [32:0] vShift4 = e_EnShift ? (e_B[1] ? vShift3[34:2] : vShift3[32:0]) : 0;
    wire [WORD_WIDTH-1:0] ALUResult = (e_B[0] ? vShift4[32:1] : vShift4[31:0]) | vShiftAlternative;








    // branch unit

    wire vEqual = (vLogicResult == ~0);
    wire vLessXor = InvertBranch_q ^ ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));
    wire vLess = (Sum[31] & (e_A[31] ^ e_B[31])) ^ vLessXor;
    wire vBEQ = e_InsnBEQ & (InvertBranch_q ^ vEqual) & ~m_Kill;
    wire vNotBEQ = ((e_InsnBLTorBLTU & vLess) | e_BranchUncondPCRel) & ~m_Kill;
    wire vCondResultBit = e_SetCond & vLess;

    wire Branch_w = vBEQ | vNotBEQ;
/*
    wire Equal_w = (vLogicResult == ~0);
    wire Less_w = (Sum[31] & (e_A[31] ^ e_B[31])) ^
                 ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));
    wire vCondResultBit = e_SetCond & Less_w;
    wire Branch_w = ~m_Kill & (
        (e_InsnBEQ       & (InvertBranch_q ^ Equal_w)) |
        (e_InsnBLTorBLTU & (InvertBranch_q ^  Less_w)) |
         e_BranchUncondPCRel
    );
*/


    wire Kill = Branch_w | (e_InsnJALR & ~m_Kill);
        // any jump or exception

    wire [WORD_WIDTH-1:0] AddrSum_w = e_A +
        {{(WORD_WIDTH-12){Imm12PlusReg_q[11]}}, Imm12PlusReg_q};
    wire [WORD_WIDTH-1:0] AddrSum2_w =
        MemMisaligned_q ? (AddrSum_q + 4) : AddrSum_w;


    wire [WORD_WIDTH-1:0] NextFetchAddr_w = FetchAddr_q + 4;

    wire [WORD_WIDTH-1:0] MemAddr =
        Branch_w // taken PC-relative branch
            ? {e_PCImm[WORD_WIDTH-1:2], 2'b00}
            : (e_AddrFromSum & ~m_Kill)
                ? {AddrSum2_w[WORD_WIDTH-1:1], 1'b0}
                : (RealignedPC_d | FetchAgainE_q)
                    ? FetchAddr_q
                    : NextFetchAddr_w;







    // mul/div unit

    // control signals for first cycle
    wire DivSigned_d = ~Insn_q[12] & Insn_q[14];
    wire MulASigned_d = (Insn_q[13:12] != 2'b11) & ~Insn_q[14];
    wire InsnMULH_d = (Insn_q[14:12]==3'b001);

    wire [WORD_WIDTH:0] DivDiff_w = DivRem_q - {1'b0, Long_q[WORD_WIDTH-1:0]};
    wire DivNegative_w = (Long_q[2*WORD_WIDTH-1:WORD_WIDTH]!=0) | DivDiff_w[WORD_WIDTH];
    wire [WORD_WIDTH+1:0] LongAdd_w =
        {Long_q[2*WORD_WIDTH], Long_q[2*WORD_WIDTH:WORD_WIDTH]}
            + (Long_q[0] ? {DivRem_q[WORD_WIDTH], DivRem_q}
                         : {(WORD_WIDTH+2){1'b0}});

    wire [WORD_WIDTH-1:0] AbsA_w = (DivSigned_q & e_A[WORD_WIDTH-1]) ? (-e_A) : e_A;
    wire [WORD_WIDTH-1:0] AbsB_w = (DivSigned_q & e_B[WORD_WIDTH-1]) ? (-e_B) : e_B;

    wire [WORD_WIDTH-1:0] DivQuot_d = {DivQuot_q[WORD_WIDTH-2:0], ~DivNegative_w};

    reg MulDivResNeg_d;
    reg MulDivResAdd_d;
    reg [WORD_WIDTH:0] DivRem_d;
    reg [2*WORD_WIDTH:0] Long_d;
    reg [WORD_WIDTH-1:0] MulDivResult_d;

    always @* begin

        // mul/div arithmetic step
        if (StartMulDiv_q) begin
            MulDivResNeg_d =
                (DivSigned_q &
                (e_A[WORD_WIDTH-1] ^ (~SelRemOrDiv_q & e_B[WORD_WIDTH-1])) &
                (e_B!=0 || SelRemOrDiv_q))
                |
                (InsnMULH_q & e_B[WORD_WIDTH-1]);
            MulDivResAdd_d = SelDivOrMul_q
                |
                (InsnMULH_q & e_B[WORD_WIDTH-1]);

            DivRem_d = {MulASigned_q & e_A[WORD_WIDTH-1], AbsA_w};
            if (SelDivOrMul_q) begin
                Long_d = {2'b0, AbsB_w, {(WORD_WIDTH-1){1'b0}}};
            end else begin
                Long_d = {{(WORD_WIDTH+1){1'b0}}, e_B};
            end
        end else begin
            MulDivResNeg_d = MulDivResNeg_q;
            MulDivResAdd_d = MulDivResAdd_q;

            DivRem_d = (DivNegative_w | ~SelDivOrMul_q)
                ? DivRem_q
                : {1'b0, DivDiff_w[WORD_WIDTH-1:0]};
            Long_d = {LongAdd_w[WORD_WIDTH+1:0], Long_q[WORD_WIDTH-1:1]};
        end

        // mul/div result computation
        MulDivResult_d =
            (SelDivOrMul_q ? {WORD_WIDTH{1'b0}}
                           : (SelMulLowOrHigh_q ? Long_q[WORD_WIDTH-1:0]
                                                : Long_q[2*WORD_WIDTH-1:WORD_WIDTH]))
            +
            (MulDivResAdd_q ? NotDivRem_w : {WORD_WIDTH{1'b0}})
            +
            {{(WORD_WIDTH-1){1'b0}}, MulDivResNeg_q};


    end

    wire [WORD_WIDTH-1:0] DivRem_w = SelRemOrDiv_q
        ? DivRem_q[WORD_WIDTH-1:0]
        : DivQuot_q;

    wire [WORD_WIDTH-1:0] NotDivRem_w = MulDivResNeg_q ? (~DivRem_w) : DivRem_w;




    wire MemWrEn        = m_WrEn | OverwriteM_q;
    wire [5:0] MemWrNo  = m_WrNo;





    // external CSR interface
    //
    // D stage: decode tree for CSR number (csr_addr)
    // E stage: read (csr_read) and write (csr_modify, csr_wdata) CSR
    // M stage: mux tree for csr_rdata

    reg  [2:0] e_CsrModify;
    reg [31:0] e_CsrWData;
    reg [11:0] e_CsrAddr;

    always @(posedge clk) begin
        e_CsrModify <= {Kill, (InsnCSR & ~m_Kill & (~Insn_q[13] | (Insn_q[19:15]!=0))) 
            ? Insn_q[13:12] : 2'b00};
        e_CsrWData  <= Insn_q[14] ? {{(WORD_WIDTH-5){1'b0}}, Insn_q[19:15]} : ForwardAE;

        // for internal CSRs
        e_CsrAddr   <= Insn_q[31:20];
    end

    assign retired    = RetiredM_q;
    assign csr_read   = CsrFromExtE_q;
    assign csr_modify = e_CsrModify;
    assign csr_wdata  = e_CsrWData;
    assign csr_addr   = Insn_q[31:20];


    // internal CSRs

    wire CsrValid_w = csr_valid | (Insn_q[31:20]==12'h300);

    reg [31:0] CsrRDataInternal_d;
    always @* case (e_CsrAddr)
        12'h300: begin // MSTATUS
            CsrRDataInternal_d = {28'b0, f_MModeIntEnable, 3'b0};
        end
        default: begin
            CsrRDataInternal_d = 0;
        end
    endcase


    // register-mapped CSRs
/*
    reg [5:0] CsrTranslateD_d;
    reg CsrFromRegD_d;
    always @* begin
        CsrFromRegD_d <= InsnCSR;
        case (Insn_q[31:20])
            12'h305: CsrTranslateD_d <= REG_CSR_MTVEC;
            12'h340: CsrTranslateD_d <= REG_CSR_MSCRATCH;
            12'h341: CsrTranslateD_d <= REG_CSR_MEPC;
            12'h342: CsrTranslateD_d <= REG_CSR_MCAUSE;
            12'h343: CsrTranslateD_d <= REG_CSR_MTVAL;
            default: begin
                CsrTranslateD_d <= 0; // cannot be written, always 0
                CsrFromRegD_d <= 0;
            end
        endcase
        CsrTranslateD_d <= {1'b1, Insn_q[26], Insn_q[23:20]};
    end
*/
    wire [4:0] CsrTranslateD_d = {Insn_q[26], Insn_q[23:20]};
    wire CsrFromRegD_d = (InsnCSR & ~m_Kill) && (Insn_q[31:27]==5'b00110) && 
        (Insn_q[25:23]==3'b0) &&
        (
         (Insn_q[22:20]==3'b100) || // mie, mip
         ((~Insn_q[26]) && (Insn_q[22:20]==3'b101)) || // mtvec
         ((Insn_q[26]) && (~Insn_q[22]))); // mscratch, mepc, mcause, mtval

    wire [WORD_WIDTH-1:0] CsrUpdate = e_CsrSelImm ? {27'b0, ImmUpper_q[19:15]} : e_A;

    wire [WORD_WIDTH-1:0] CsrModified = ~m_CsrOp[1]
        ? (~m_CsrOp[0] ? 32'h0 : m_CsrUpdate)
        : (~m_CsrOp[0] ? (e_A | m_CsrUpdate) : (e_A & ~m_CsrUpdate));
            // TRY: e_A instead of e_B would also be possible, if vCsrInsn is adjusted
            // e_A is a bypass from the execute stage of the next cycle

/*
    wire [WORD_WIDTH-1:0] CsrResult_w =
        (CsrFromRegM_q ? e_A           : 0) |
        (CsrFromRegW_q ? m_CsrModified : 0) |
        (CsrFromExtM_q ? (CsrRDataInternal_q | csr_rdata) : 0) |
        ((CsrFromRegM_q | CsrFromRegW_q | CsrFromExtM_q)
            ? 0 : m_WrData);

    wire [WORD_WIDTH-1:0] OverwrittenResult_w =
        OverwriteM_q ? OverwriteValM_q : CsrResult_w;
*/
    wire [WORD_WIDTH-1:0] OverwrittenResult_w =
        (CsrFromRegM_q ? e_A                              : 0) |
        (CsrFromRegW_q ? m_CsrModified                    : 0) |
        (CsrFromExtM_q ? (CsrRDataInternal_q | csr_rdata) : 0) |
        (OverwriteM_q  ? OverwriteValM_q                  : 0) |

        ((CsrFromRegM_q | CsrFromRegW_q | CsrFromExtM_q | OverwriteM_q)
                       ? 0 : m_WrData);





    wire [WORD_WIDTH-1:0] OverwriteValE_d = OverwriteSelE_q
        ? MulDivResult_d
        : OverwriteValE_q;





    // memory stage

    wire [1:0] AddrOfs = AddrSum_w[1:0];
    //wire [1:0] AddrOfs = {
    //    e_A[1] ^ Imm12PlusReg_q[1] ^ (e_A[0] & Imm12PlusReg_q[0]),
    //    e_A[0] ^ Imm12PlusReg_q[0]};

    reg MemMisaligned_d;
    always @* case ({e_MemWidth, AddrOfs})
        4'b0000: MemMisaligned_d = 0;
        4'b0001: MemMisaligned_d = 0;
        4'b0010: MemMisaligned_d = 0;
        4'b0011: MemMisaligned_d = 0;
        4'b0100: MemMisaligned_d = 0;
        4'b0101: MemMisaligned_d = 0;
        4'b0110: MemMisaligned_d = 0;
        4'b0111: MemMisaligned_d = 1; // lh +3
        4'b1000: MemMisaligned_d = 0;
        4'b1001: MemMisaligned_d = 1; // lw +1
        4'b1010: MemMisaligned_d = 1; // lw +2
        4'b1011: MemMisaligned_d = 1; // lw +3
        default: MemMisaligned_d = 0;
    endcase

    wire MemSigned2_w =         // LH                LB
        ((MemWidthM_q[0] ? (AddrOfsM_q==3) : (AddrOfsM_q==0)) & mem_rdata[7]) |
        ((MemWidthM_q[0] ? (AddrOfsM_q==0) : (AddrOfsM_q==1)) & mem_rdata[15]) |
        ((MemWidthM_q[0] ? (AddrOfsM_q==1) : (AddrOfsM_q==2)) & mem_rdata[23]) |
        ((MemWidthM_q[0] ? (AddrOfsM_q==2) : (AddrOfsM_q==3)) & mem_rdata[31]);
    wire MemSigned_w = MemSignedLoadM_q & MemSigned2_w;





/*
    reg [3:0] SelCurMemByte_d;
    always @* case ({MemWidthM_q, AddrOfsM_q})
        4'b0000: SelCurMemByte_d = 4'b0001;
        4'b0001: SelCurMemByte_d = 4'b0001;
        4'b0010: SelCurMemByte_d = 4'b0001;
        4'b0011: SelCurMemByte_d = 4'b0001;
        4'b0100: SelCurMemByte_d = 4'b0011;
        4'b0101: SelCurMemByte_d = 4'b0011;
        4'b0110: SelCurMemByte_d = 4'b0011;
        4'b0111: SelCurMemByte_d = 4'b0010; // lh +3
        4'b1000: SelCurMemByte_d = 4'b1111;
        4'b1001: SelCurMemByte_d = 4'b1000; // lw +1
        4'b1010: SelCurMemByte_d = 4'b1100; // lw +2
        4'b1011: SelCurMemByte_d = 4'b1110; // lw +3
        default: SelCurMemByte_d = 0;
    endcase

    wire [31:0] MemRShiftedBy1_w = AddrOfsM_q[1]
        ? {mem_rdata[15:0], mem_rdata[31:16]}
        :  mem_rdata;
    wire [31:0] MemRShifted_w = AddrOfsM_q[0]
        ? {MemRShiftedBy1_w[7:0], MemRShiftedBy1_w[31:8]}
        :  MemRShiftedBy1_w;

    reg [31:0] MemSignOrPrev_w;
    always @* begin
        MemSignOrPrev_w[31:24] = MemSigned_w
            ? 8'hFF
            : OverwrittenResult_w[31:24];
        MemSignOrPrev_w[23: 8] = (MemWidthM_q==2)
            ? MemRDataPrev_q[23:8]
            : (MemSigned_w ? 16'hFFFF
                           : OverwrittenResult_w[23:8]);
        MemSignOrPrev_w[ 7: 0] = (MemWidthM_q!=3)
            ? MemRDataPrev_q[7:0]
            : OverwrittenResult_w[7:0];
    end

    wire [31:0] MemResult = {
        SelCurMemByte_d[3] ? MemRShifted_w[31:24] : MemSignOrPrev_w[31:24],
        SelCurMemByte_d[2] ? MemRShifted_w[23:16] : MemSignOrPrev_w[23:16],
        SelCurMemByte_d[1] ? MemRShifted_w[15: 8] : MemSignOrPrev_w[15: 8],
        SelCurMemByte_d[0] ? MemRShifted_w[ 7: 0] : MemSignOrPrev_w[ 7: 0]};
*/


    reg [11:0] MemSel_w;
    always @* case ({MemWidthM_q, AddrOfsM_q})
        4'b0000: MemSel_w = 12'b00_000000_0001;
        4'b0001: MemSel_w = 12'b00_000000_0010;
        4'b0010: MemSel_w = 12'b00_000000_0100;
        4'b0011: MemSel_w = 12'b00_000000_1000;
        4'b0100: MemSel_w = 12'b00_000010_0001;
        4'b0101: MemSel_w = 12'b00_000100_0010;
        4'b0110: MemSel_w = 12'b00_001000_0100;
        4'b0111: MemSel_w = 12'b00_000001_0000;
        4'b1000: MemSel_w = 12'b01_000010_0001;
        4'b1001: MemSel_w = 12'b00_010000_0000;
        4'b1010: MemSel_w = 12'b00_100000_0000;
        4'b1011: MemSel_w = 12'b10_000001_0000;
        default: MemSel_w = 0;
    endcase

    wire  [7:0] LoByte0_w = (MemSel_w[0] ? mem_rdata[7:0] : 8'b0)
                          | (MemSel_w[1] ? mem_rdata[15:8] : 8'b0);
    wire  [7:0] LoByte1_w = (MemSel_w[2] ? mem_rdata[23:16] : 8'b0)
                          | (MemSel_w[3] ? mem_rdata[31:24] : 8'b0);
    wire  [7:0] LoByte2_w = OverwrittenResult_w[7:0]
                          | (MemSel_w[8] ? MemRData_q[15:8] : 8'b0);
    wire  [7:0] LoByte3_w = (MemSel_w[9] ? MemRData_q[23:16] : 8'b0)
                          | (MemSel_w[4] ? MemRData_q[31:24] : 8'b0);
    wire  [7:0] LoByte    = LoByte0_w | LoByte1_w | LoByte2_w | LoByte3_w;

    wire  [7:0] HiByte0_w = (MemSel_w[4] ? mem_rdata[7:0] : 8'b0)
                          | (MemSel_w[5] ? mem_rdata[15:8] : 8'b0);
    wire  [7:0] HiByte1_w = (MemSel_w[6] ? mem_rdata[23:16] : 8'b0)
                          | (MemSel_w[7] ? mem_rdata[31:24] : 8'b0);
    wire  [7:0] HiByte2_w = (MemSel_w[8] ? MemRData_q[23:16] : 8'b0)
                          | (MemSel_w[9] ? MemRData_q[31:24] : 8'b0);
    wire  [7:0] HiByte3_w = ((MemWidthM_q==0) & MemSigned_w)
                          ? 8'hFF : OverwrittenResult_w[15:8];
    wire  [7:0] HiByte    = HiByte0_w | HiByte1_w | HiByte2_w | HiByte3_w;

    wire  [15:0] HiHalf0_w = (MemSel_w[10] ? mem_rdata[31:16] : 16'b0)
                           | (MemSel_w[8] ? {mem_rdata[7:0], MemRData_q[31:24]} : 16'b0);
    wire  [15:0] HiHalf1_w = (MemSel_w[9] ? mem_rdata[15:0] : 16'b0)
                           | (MemSel_w[11] ? mem_rdata[23:8] : 16'b0);
    wire  [15:0] HiHalf3_w = (~MemWidthM_q[1] & MemSigned_w)
                           ? 16'hFFFF : OverwrittenResult_w[31:16];
    wire  [15:0] HiHalf    = HiHalf0_w | HiHalf1_w | HiHalf3_w;

    wire [31:0] MemResult = {HiHalf, HiByte, LoByte};








    reg [WORD_WIDTH-1:0] MemWData_d;
    always @* begin
        if (MemMisaligned_q) begin
            MemWData_d = MemWData_q;
        end else begin
            case (AddrOfs)
                2'b01: MemWData_d = {e_B[23:0], e_B[31:24]};
                2'b10: MemWData_d = {e_B[15:0], e_B[31:16]};
                2'b11: MemWData_d = {e_B[7:0], e_B[31:8]};
                default: MemWData_d = e_B;
            endcase
        end
    end

    reg [3:0] MemWMask_d;
    always @* begin
        if (MemMisaligned_q) begin
/*
            case ({MemWidthM_q[0], AddrOfsM_q})
                3'b111: MemWMask_d = 4'b0001;
                3'b001: MemWMask_d = 4'b0001;
                3'b010: MemWMask_d = 4'b0011;
                3'b011: MemWMask_d = 4'b0111;
                // other cases don't care, because there won't be
                // a second memory access cycle
                default: MemWMask_d = 4'b0001;
            endcase
*/
        MemWMask_d = {1'b0,
                      ~MemWidthM_q[0] & AddrOfsM_q[1] & AddrOfsM_q[0],
                      ~MemWidthM_q[0] & AddrOfsM_q[1],
                      1'b1};
        end else begin
            case ({e_MemWidth, AddrOfs})
                4'b0000: MemWMask_d = 4'b0001;
                4'b0001: MemWMask_d = 4'b0010;
                4'b0010: MemWMask_d = 4'b0100;
                4'b0011: MemWMask_d = 4'b1000;
                4'b0100: MemWMask_d = 4'b0011;
                4'b0101: MemWMask_d = 4'b0110;
                4'b0110: MemWMask_d = 4'b1100;
                4'b0111: MemWMask_d = 4'b1000;
                4'b1000: MemWMask_d = 4'b1111;
                4'b1001: MemWMask_d = 4'b1110;
                4'b1010: MemWMask_d = 4'b1100;
                4'b1011: MemWMask_d = 4'b1000;
                default: MemWMask_d = 0;
            endcase
        end
    end




    assign mem_valid = 1;
    assign mem_write = e_MemStore & ~m_Kill;
    assign mem_wmask = MemWMask_d;
    assign mem_wdata = MemWData_d;
    assign mem_wgrubby = 0;
    assign mem_addr  = MemAddr;





    // register set

    assign regset_we = MemWrEn;
    assign regset_wa = MemWrNo;
    assign regset_wd = MemResult;
    assign regset_wg = 0;
    assign regset_ra1 = RdNo1;
    assign regset_ra2 = RdNo2;






// ---------------------------------------------------------------------
// sequential logic
// ---------------------------------------------------------------------


    always @(posedge clk) begin

        // fetch
        Insn_q <= Insn_d;
        InsnE_q <= (Insn_q[1:0]==2'b11) ? Insn_q : {16'b0, Insn_q[15:0]};
            // only for illegal exception
        InsnM_q <= InsnE_q;

        d_RdNo1 <= RdNo1;
        d_RdNo2 <= RdNo2;
        CsrTranslateE_q <= CsrTranslateD_d;
        MC_q <= MC_d;
        MCState_q <= MCState_d;
        MCAux_q <= MCAux_d;
        MCRegNo_q <= MCRegNo_d;

        PartialInsn_q <= PartialInsn_d;
        OddPC_q <= OddPC_d;
        DelayedInsn_q <= DelayedInsn_d;
        BubbleM_q <= BubbleE_d;
        BubbleE_q <= BubbleD_d;

        FetchAddr_q <= FetchAddr_d;
        FetchAgainE_q <= FetchAgainD_d;
        FetchAgainM_q <= FetchAgainE_q;

        StartMulDiv_q <= StartMulDiv_d;
        DivSigned_q <= DivSigned_d;
        MulASigned_q <= MulASigned_d;
        InsnMULH_q <= InsnMULH_d;
        SelRemOrDiv_q <= SelRemOrDiv_d;
        SelDivOrMul_q <= SelDivOrMul_d;
        MulDivResNeg_q <= MulDivResNeg_d;
        MulDivResAdd_q <= MulDivResAdd_d;
        SelMulLowOrHigh_q <= SelMulLowOrHigh_d;
        DivQuot_q <= DivQuot_d;
        DivRem_q <= DivRem_d;
        Long_q <= Long_d;


        // decode
        PC_q <= PC_d;
        e_A <= ForwardAE;
        e_B <= ForwardBE;
        ImmUpper_q <= ImmUpper_d;
        Imm12PlusReg_q <= Imm12PlusReg_d;
        e_PCImm <= PCImm;

        e_WrEn <= DecodeWrEn;
        e_InsnJALR <= InsnJALR;
        e_InsnBEQ <= InsnBEQ;
        e_InsnBLTorBLTU <= InsnBLTorBLTU;
        e_BranchUncondPCRel <= BranchUncondPCRel;

        e_EnShift <= EnShift;
        ShiftRight_q <= ShiftRight_d;
        ShiftArith_q <= ShiftArith_d;
        e_ReturnPC <= ReturnPC;
        e_ReturnUI <= ReturnUI;

        e_SelSum <= SelSum;
        e_SetCond <= SetCond;
        e_LTU <= LTU;

        e_AddrFromSum <= AddrFromSum;
        e_MemStore <= MemStore;
        e_MemWidth <= MemWidth;

        e_SelLogic <= SelLogic;
        e_NegB <= NegB;
        e_WrNo <= DecodeWrNo;
        InvertBranch_q <= InvertBranch_d;
        e_InsnBit14 <= Insn_q[14];


        // execute
        m_WrEn <= ExecuteWrEn;
        m_WrNo <= ExecuteWrNo;
        m_WrData <= ALUResult;
        m_Kill <= Kill;

        MemSignedLoadM_q    <= MemMisaligned_q ? MemSignedLoadM_q : ~e_InsnBit14;
        MemWidthM_q         <= MemMisaligned_q ? MemWidthM_q :
                                (m_Kill ? 2'b11 : e_MemWidth);
        AddrOfsM_q          <= MemMisaligned_q ? AddrOfsM_q : AddrSum_w[1:0];
        AddrSum_q           <= AddrSum_w;
        MemRData_q          <= mem_rdata;
        MemWData_q          <= MemWData_d;
        MemMisaligned_q     <= MemMisaligned_d; // & ~m_Kill;
        //MemRDataPrev_q      <= MemRShifted_w;

        // mem stage
        w_WrEn <= MemWrEn;
        w_WrNo <= MemWrNo;
        w_WrData <= MemResult;

        RetiredM_q <= RetiredE_q & ~m_Kill;
        RetiredE_q <= RetiredD_d;

        // exception handling
        PrevPC_q            <= PC_q;
        OverwriteE_q        <= Overwrite_d;
        OverwriteValE_q     <= OverwriteVal_d;
        OverwriteM_q        <= OverwriteE_q;
        OverwriteValM_q     <= OverwriteValE_d;
        OverwriteSelE_q     <= OverwriteSelD_d;
        InsnMRET_E_q        <= InsnMRET_d;
        InsnMRET_M_q        <= InsnMRET_E_q;
        ExcInvalidInsn_q    <= ExcInvalidInsn;

        // csr
        m_CsrUpdate         <= CsrUpdate;
        e_CsrOp             <= CsrOp;
        m_CsrOp             <= e_CsrOp;
        m_CsrModified       <= CsrModified;
        CsrFromRegE_q       <= CsrFromRegD_d;
        CsrFromRegM_q       <= CsrFromRegE_q;
        CsrFromRegW_q       <= CsrFromRegM_q;
        CsrFromExtE_q       <= CsrFromExtD_d;
        CsrFromExtM_q       <= CsrFromExtE_q;
        CsrRDataInternal_q  <= CsrRDataInternal_d;

        // interrupt handling
        f_MModeIntEnable    <= MModeIntEnable;
        f_MModePriorIntEnable <= MModePriorIntEnable;
        IrqResponse_q       <= IrqResponse_d;
        NoIrq_q             <= NoIrq_d;
        SoftwareIrq_q       <= irq_software;
        TimerIrq_q          <= irq_timer;
        ExternalIrq_q       <= irq_external;




`ifdef DEBUG
        $display("F write=%b wmask=%b wdata=%h addr=%h rdata=%h",
            mem_write, mem_wmask, mem_wdata, mem_addr, mem_rdata);
        if (MC_q==0) begin
            if (Insn_q[1:0]==2'b11) begin
                $display("D pc=\033[1;33m%h\033[0m insn=%h next=%h \033[1;30mMC=no\033[0m",
                    PC_q, Insn_q, Insn_d);
            end else begin
                $display("D pc=\033[1;33m%h\033[0m insn=\033[1;30m%h\033[0m%h next=%h \033[1;30mMC=no\033[0m",
                    PC_q, Insn_q[31:16], Insn_q[15:0], Insn_d);
            end
        end else begin
            $display("D pc=\033[1;33m%h\033[1;30m insn=%h\033[0m next=%h \033[1;33mMC=%h:%h\033[0m",
                PC_q, Insn_q, Insn_d, MCState_q, MCAux_q);
        end

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

        $display("D read x%d=%h x%d=%h SelImm=%b %h",
            d_RdNo1, regset_rd1, d_RdNo2, regset_rd2, vSelImm, ImmALU_w);
        $display("D Bubble=%b%b%b FetchAddr_q=%h Delayed=%h Partial=%h",
            BubbleD_d, BubbleE_q, BubbleM_q, FetchAddr_q, DelayedInsn_q, PartialInsn_q);
/*
        $display("D P=%h align=%b%b RVC=%b",
            PartialInsn_d, RealignedPC_d, OddPC_d, RVCInsn_w);
        $display("D Imm21PCRel=%b",
            Imm21PCRel_w);
*/

        $display("E a=%h b=%h -> %h -> x%d wrenDE=%b%b",
            e_A, e_B, ALUResult, e_WrNo, DecodeWrEn, e_WrEn);
/*
        $display("E logic=%h pc=%h ui=%h e_SelSum=%b e_EnShift=%b",
            vLogicResult, vPCResult, vUIResult, e_SelSum, e_EnShift);
        $display("E rem=%h long=%h SelMulLowOrHigh_q=%b StartMulDiv_q=%b",
            DivRem_q, Long_q, SelMulLowOrHigh_q, StartMulDiv_q);
        $display("E SelDivOrMul_d=%b DivSigned_q=%b div=%h MulDivResult_d=%h",
            SelDivOrMul_d, DivSigned_q, DivQuot_q, MulDivResult_d);
*/

        if (Kill) $display("B \033[1;35mjump %h\033[0m", FetchAddr_d);
/*
        $display("B vBEQ=%b vNotBEQ=%b e_InsnJALR=%b KillEMW=%b%b",
            vBEQ, vNotBEQ, e_InsnJALR, Kill, m_Kill);
        $display("  e_InsnBLTorBLTU=%b vLess=%b e_BranchUncondPCRel=%b ReturnPC=%b",
            e_InsnBLTorBLTU, vLess, e_BranchUncondPCRel, ReturnPC);
        $display("  e_InsnBEQ=%b InvertBranch_q=%b vEqual=%b",
            e_InsnBEQ, InvertBranch_q, vEqual);
*/
/*
        $display("F AE=%b AM=%b AW=%b AR=%h AM=%h AE=%h",
            FwdAE, FwdAM, FwdAW, ForwardAR, ForwardAM, ForwardAE);
        $display("F BE=%b BM=%b BW=%b BR=%h BM=%h BE=%h vSelImm=%b",
            FwdBE, FwdBM, FwdBW, ForwardBR, ForwardBM, ForwardBE, vSelImm);
*/

        $display("C MTVEC=%h MSCRATCH=%h MEPC=%h MCAUSE=%h MTVAL=%h",
            RegSet.regs[REG_CSR_MTVEC],
            RegSet.regs[REG_CSR_MSCRATCH],
            RegSet.regs[REG_CSR_MEPC],
            RegSet.regs[REG_CSR_MCAUSE],
            RegSet.regs[REG_CSR_MTVAL]);

/*
        $display("C CsrTranslateD_d=%h CsrOp=%b e_%b m_%b",
            CsrTranslateD_d, CsrOp, e_CsrOp, m_CsrOp);
        $display("C CsrResult_w=%h OverwrittenResult_w=%h",
            CsrResult_w, OverwrittenResult_w);
        $display("C CsrFromRegD_d %b e_%b m_%b w_%b m_CsrModified=%h m_WrData=%h", 
            CsrFromRegD_d, CsrFromRegE_q, CsrFromRegM_q, CsrFromRegW_q,
            m_CsrModified, m_WrData);
        $display("C CsrFromExtM_q=%b CsrRDataInternal_q=%h",
            CsrFromExtM_q, CsrRDataInternal_q);
        $display("C CSR addr=%h rdata=%h valid=%b",
            csr_addr, csr_rdata, csr_valid);
*/


        $display("M e_MemWidth=%b AddrOfs=%b RealignedPC_d=%b MemAddr=%h FetchAgainE_q=%b",
            e_MemWidth, AddrOfs, RealignedPC_d, MemAddr, FetchAgainE_q);
        $display("M MemWidthM_q=%b AddrOfsM_q=%b MemMisaligned_q=%b",
            MemWidthM_q, AddrOfsM_q, MemMisaligned_q);
        $display("X OverwriteDEM=%b%b%b ValDEM=%h %h %h SelE=%b",
            Overwrite_d, OverwriteE_q, OverwriteM_q,
            OverwriteVal_d, OverwriteValE_q, OverwriteValM_q,
            OverwriteSelE_q);


//        $display("  DecodeWrNo=%b e_WrNo=%d ExecuteWrNo=%d m_WrNo=%d",
//            DecodeWrNo, e_WrNo, ExecuteWrNo, m_WrNo);
//        $display("  DestReg0Part=%b DisableWrite=%b EnableWrite2=%b WrEnEMW=%b%b%b",
//            DestReg0Part, DisableWrite, EnableWrite2, DecodeWrEn, ExecuteWrEn, MemWrEn);
        $display("X OverwrittenResult_w=%h",
            OverwrittenResult_w);

        $display("I MIE=%b MPIE=%b Software=%b Timer=%b External=%b IRdq=%b%b",
            f_MModeIntEnable, f_MModePriorIntEnable, 
            SoftwareIrq_q,
            TimerIrq_q,
            ExternalIrq_q,
            IrqResponse_d, IrqResponse_q);

        if (m_WrEn) $display("M x%d<-%h", m_WrNo, m_WrData);
        if (w_WrEn) $display("W x%d<-%h",w_WrNo, w_WrData);
`endif

        if (!rstn) begin
            e_WrEn <= 0;
            e_AddrFromSum <= 0;
            e_MemStore <= 0;
            e_MemWidth <= 2'b11;
            FetchAddr_q <= 32'hf0000000;

            Insn_q <= 32'h13;
            OddPC_q <= 0;
            MC_q <= 0;
            BubbleM_q <= 0;
            BubbleE_q <= 0;
            DelayedInsn_q <= 'h13;

            StartMulDiv_q <= 0;

            // fake a jump to address 0 on reset
            m_Kill <= 0;
            e_PCImm <= START_PC;
            e_BranchUncondPCRel  <= 1;

            f_MModeIntEnable <= 0;
            f_MModePriorIntEnable <= 0;
            IrqResponse_q <= 0;
            NoIrq_q <= 0;
        end
    end



`ifdef RISCV_FORMAL
    reg [31:0]           RvfiInsnM_q;
    reg                  RvfiTrapE_q;
    reg                  RvfiTrapM_q;
    reg                  RvfiHalt_q;
    reg                  RvfiIntr0_q;
    reg                  RvfiIntr1_q;
    reg                  RvfiIntrF_q;
    reg                  RvfiIntrD_q;
    reg                  RvfiIntrE_q;
    reg                  RvfiIntrM_q;
    reg [4:0]            RvfiRdNo1E_q;
    reg [4:0]            RvfiRdNo1M_q;
    reg [4:0]            RvfiRdNo2E_q;
    reg [4:0]            RvfiRdNo2M_q;
    reg [WORD_WIDTH-1:0] RvfiRdData1M_q;
    reg [WORD_WIDTH-1:0] RvfiRdData2M_q;
    reg [WORD_WIDTH-1:0] RvfiPcE_q;
    reg [WORD_WIDTH-1:0] RvfiPcM_q;
    reg [WORD_WIDTH-1:0] RvfiNextPcM_q;
    reg                  RvfiExcRetM_q;
    reg                  RvfiMemStoreM_q;
    reg [WORD_WIDTH-1:0] RvfiMemAddrM_q;
    reg [3:0]            RvfiMemRMaskM_q;
    reg [3:0]            RvfiMemWMaskM_q;
    reg [WORD_WIDTH-1:0] RvfiMemWDataM_q;
    reg                  RvfiAltResultM_q;
    reg [WORD_WIDTH-1:0] RvfiResultM_q;
    reg                  RvfiRetireMisLoadW_q;

    wire [3:0] RvfiMemMask =
        e_MemWidth[1] ? (e_MemWidth[0] ? 4'b0000 : 4'b1111)
                      : (e_MemWidth[0] ? 4'b0011 : 4'b0001);
    wire RvfiRetireMisLoadM_d = RetiredM_q
        & MC_q
        & (MCState_q==mcMem)
        & ~RvfiMemStoreM_q;
    wire RvfiRetire_w = RvfiTrapM_q 
        | (RetiredM_q & ~RvfiRetireMisLoadM_d)
        | RvfiRetireMisLoadW_q;

    always @(posedge clk) begin

        rvfi_valid      <= rstn & !rvfi_halt & RvfiRetire_w;
        rvfi_order      <= rstn ? rvfi_order + rvfi_valid : 0;
        rvfi_trap       <= RvfiTrapM_q;

        rvfi_halt       <= RvfiTrapM_q | rvfi_halt;
        rvfi_intr       <= RvfiIntrM_q;
        rvfi_mode       <= 3;
        rvfi_ixl        <= 1;

        // For the 2nd cycle of a misaligned load, just hold the values
        if (~RvfiRetireMisLoadW_q) begin
            rvfi_insn       <= RvfiInsnM_q;
            rvfi_rs1_addr   <= RvfiRdNo1M_q;
            rvfi_rs2_addr   <= RvfiRdNo2M_q;
            rvfi_rs1_rdata  <= RvfiRdData1M_q;
            rvfi_rs2_rdata  <= RvfiRdData2M_q;
            rvfi_pc_rdata   <= RvfiPcM_q;
            rvfi_pc_wdata   <= RvfiExcRetM_q ? FetchAddr_d : RvfiNextPcM_q;
            rvfi_mem_addr   <= RvfiMemAddrM_q;
            rvfi_mem_rmask  <= RvfiMemRMaskM_q;
            rvfi_mem_wmask  <= RvfiMemWMaskM_q;
            rvfi_mem_wdata  <= RvfiMemWDataM_q;
        end

        rvfi_rd_addr    <= (MemWrEn & ~MemWrNo[5]) ? MemWrNo : 0;
        rvfi_rd_wdata   <= RvfiAltResultM_q ? RvfiResultM_q :
                           (MemWrEn & ~MemWrNo[5]) ? MemResult : 0;
        rvfi_mem_rdata  <= MemResult;

        RvfiRetireMisLoadW_q <= RvfiRetireMisLoadM_d;
        RvfiInsnM_q     <= InsnE_q;
//        RvfiTrapM_q     <= (RvfiTrapE_q         // illegal instruction
        RvfiTrapM_q     <= (ExcInvalidInsn_q         // illegal instruction
                            | (MC_q && MCState_q==mcWFI)) & ~m_Kill;    // WFI insn
        RvfiTrapE_q     <= ExcInvalidInsn;
        RvfiIntrM_q     <= RvfiIntrE_q; // long delay (jump to MTVEC)
        RvfiIntrE_q     <= RvfiIntrD_q;
        RvfiIntrD_q     <= RvfiIntrF_q;
        RvfiIntrF_q     <= RvfiIntr1_q;
        RvfiIntr1_q     <= RvfiIntr0_q;
        RvfiIntr0_q     <= MC_q & (MCState_q==mcException);

        RvfiRdNo1M_q    <= RvfiRdNo1E_q;
        RvfiRdNo1E_q    <= d_RdNo1[5] ? 5'b0 : d_RdNo1; // no CSRs
        RvfiRdNo2M_q    <= RvfiRdNo2E_q;
        RvfiRdNo2E_q    <= (vSelImm | d_RdNo2[5]) ? 5'b0 : d_RdNo2; // no CSRs
        RvfiRdData1M_q  <= e_A;
        RvfiRdData2M_q  <= (RvfiRdNo2E_q==0) ? 5'b0 : (e_B ^ {WORD_WIDTH{e_NegB}});

        RvfiPcM_q       <= RvfiPcE_q;
        RvfiPcE_q       <= PC_q;
        RvfiNextPcM_q   <= Kill ? FetchAddr_d : PC_q;
        RvfiExcRetM_q   <= MC_q & ((MCState_q==mcJumpReg) | (MCState_q==mcException));

        RvfiMemStoreM_q <= e_MemStore;
        RvfiMemAddrM_q  <= AddrSum_w;
        RvfiMemRMaskM_q <= e_MemStore ? 0 : RvfiMemMask;
        RvfiMemWMaskM_q <= mem_write ? RvfiMemMask : 0;
        RvfiMemWDataM_q <= e_B & {{8{RvfiMemMask[3]}}, {8{RvfiMemMask[2]}},
                                  {8{RvfiMemMask[1]}}, {8{RvfiMemMask[0]}}};



`ifdef RISCV_FORMAL_ALTOPS
        RvfiAltResultM_q <= 0;
        RvfiResultM_q <= 0;
        if (StartMulDiv_q) begin
            RvfiAltResultM_q <= ExecuteWrEn;
            if (~SelDivOrMul_q) begin
                if (SelMulLowOrHigh_q) begin // MUL
                    RvfiResultM_q <= (e_A + e_B) ^ 32'h5876063e;
                end else if (InsnMULH_q) begin // MULH
                    RvfiResultM_q <= (e_A + e_B) ^ 32'hf6583fb7;
                end else if (MulASigned_q) begin // MULHSU
                    RvfiResultM_q <= (e_A - e_B) ^ 32'hecfbe137;
                end else begin // MULHU
                    RvfiResultM_q <= (e_A + e_B) ^ 32'h949ce5e8;
                end
            end else begin
                if (~SelRemOrDiv_q) begin
                    if (DivSigned_q) begin // DIV
                        RvfiResultM_q <= (e_A - e_B) ^ 32'h7f8529ec;
                    end else begin // DIVU
                        RvfiResultM_q <= (e_A - e_B) ^ 32'h10e8fd70;
                    end
                end else begin
                    if (DivSigned_q) begin // REM
                        RvfiResultM_q <= (e_A - e_B) ^ 32'h8da68fa5;
                    end else begin // REMU
                        RvfiResultM_q <= (e_A - e_B) ^ 32'h3138d0e1;
                    end
                end
            end
        end
`endif


        if (!rstn) begin
            rvfi_rd_addr    <= 0;
            rvfi_rd_wdata   <= 0;
            RvfiHalt_q      <= 0;
        end
    end
`endif



endmodule


// SPDX-License-Identifier: ISC
