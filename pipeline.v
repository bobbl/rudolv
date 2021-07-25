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
    output csr_write,
    output [31:0] csr_wdata,
    output [11:0] csr_addr,
    input [31:0] csr_rdata,
    input csr_valid,            // CSR addr is valid, not necessarily rdata

    output mem_valid,
    output mem_write,
    output [3:0] mem_wmask,
    output [31:0] mem_wdata,
    output [31:0] mem_waddr,    // early write address to select RAM block
    output [31:0] mem_addr,
    input [31:0] mem_rdata,

    output        regset_we,
    output  [5:0] regset_wa,
    output [31:0] regset_wd,
    output  [5:0] regset_ra1,
    output  [5:0] regset_ra2,
    input  [31:0] regset_rd1,
    input  [31:0] regset_rd2

);
    localparam integer WORD_WIDTH = 32;

    localparam [5:0] REG_CSR_MTVEC    = 6'b100101;
    localparam [5:0] REG_CSR_MSCRATCH = 6'b110000; // 48 30h
    localparam [5:0] REG_CSR_MEPC     = 6'b110001; // 49 31h
    localparam [5:0] REG_CSR_MCAUSE   = 6'b110010; // 50 32h
    localparam [5:0] REG_CSR_MTVAL    = 6'b110011; // 51 33h
    localparam [5:0] REG_CSR_TMP2     = 6'b111110; // 62 3Eh
    localparam [5:0] REG_CSR_TMP      = 6'b111111; // 63 3Fh


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
    reg MisalignedJumpE_q;

    // decode
    reg [31:0] Insn_q;
    reg [31:0] MCInsn_q;
    reg [5:0] d_RdNo1;
    reg [5:0] d_RdNo2;

    reg MC_q;
    reg MCModRdNo_q;
    reg [5:0] MCAux_q;
    reg NextInsnHold_q;
    reg [4:0] CsrTranslateE_q;

    reg [WORD_WIDTH-1:0] PC_q;
    reg [31:0] DelayedInsn_q;
    reg MemAccessEorM_q;
    reg MemAccessE_q;

    reg [31:0] PartialInsn_q;
    reg OddPC_q;
    reg MemMisalignedW_q;

    // mul/div
    reg MulDivResNeg_q;
    reg MulDivResAdd_q;
    reg [WORD_WIDTH-1:0] DivQuot_q;
    reg [WORD_WIDTH:0] DivRem_q;
    reg [2*WORD_WIDTH:0] Long_q;
    reg [WORD_WIDTH-1:0] MulDivResult_q;

    // execute
    reg e_InsnJALR;
    reg e_InsnBEQ;
    reg e_InsnBLTorBLTU;
    reg InvertBranch_q;
    reg [1:0] LogicOp_q;
    reg EnLogic_q;
    reg e_EnShift;
    reg ShiftRight_q;
    reg ShiftArith_q;
    reg ReturnPC_q;
    reg e_ReturnUI;
    reg e_InsnJAL;

    reg e_SetCond;
    reg e_LTU;
    reg e_SelSum;

    reg AddrFromSum_q;
    reg MemStore_q;
    reg [1:0] MemWidthE_q;

    reg [WORD_WIDTH-1:0] e_A;
    reg [WORD_WIDTH-1:0] e_B;
    reg [WORD_WIDTH-1:0] ImmUpper_q;
    reg [11:0] Imm12PlusReg_q;
    reg [WORD_WIDTH-1:0] e_PCImm;

    reg e_NegB;
    reg WrEnE_q;
    reg [5:0] WrNoE_q;

    // mem stage
    reg m_Kill; // to decode and execute stage
    reg WrEnM_q;
    reg [5:0] WrNoM_q;
    reg [WORD_WIDTH-1:0] WrDataM_q;
    reg [1:0] MemWidthM_q;
    reg [1:0] AddrOfsM_q;
    reg [WORD_WIDTH-1:0] MemRData_q;
    reg [WORD_WIDTH-1:0] MemWData_q;
    reg [WORD_WIDTH-1:0] MemRDataPrev_q;
    reg MemMisalignedM_q;
    reg [WORD_WIDTH-1:2] AddrSumAligned_q;

    // write back
    reg WrEnW_q;
    reg [5:0] WrNoW_q;
    reg [WORD_WIDTH-1:0] WrDataW_q;

    reg RetiredE_q;
    reg RetiredM_q;

    reg OverPcM_q;
    reg OverInsnM_q;
    reg OverPcE_q;
    reg OverInsnE_q;
    reg OverRs1_q;
    reg OverImm5_q;
    reg       CauseIntE_q;
    reg [3:0] CauseE_q;


    // exceptions
    reg [WORD_WIDTH-1:0] MCPrevPC_q;
    reg OverwriteE_q;
    reg OverwriteM_q;
    reg [WORD_WIDTH-1:0] OverwriteValM_q;
    reg MCOverSelDivE_q;
    reg MCOverSelDivM_q;
    reg TrapReturn_q;
    reg ExcIllegal_q;

    // CSRs for exceptions
    reg CsrFromExtE_q;
    reg CsrFromExtM_q;
    reg [WORD_WIDTH-1:0] CsrRDataInternal_q;

    reg f_MModeIntEnable;        // mstatus.mie
    reg f_MModePriorIntEnable;   // mstatus.mpie
    reg IrqResponse_q;
    reg SoftwareIrq_q;
    reg TimerIrq_q;      // external pin high
    reg ExternalIrq_q;

    reg [NumStates-1:0] MCState_q;
    reg InsnCEBREAK_q;
    reg TrapIllegal_q;


// ---------------------------------------------------------------------
// combinational circuits
// ---------------------------------------------------------------------







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

    wire [11:0] Imm12PlusReg_w = ((~Insn_q[0] & Insn_q[1] & ~Insn_q[14]) | MC_q)
        ? 0 // C.JR or C.JALR or enter/leave trap
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




    localparam mcNop            = 0;
    localparam mcLast           = 1;
    localparam mcSysInsn        = 2;
    localparam mcCsr1           = 3;
    localparam mcCsr2           = 4;
    localparam mcCsr3           = 5;
    localparam mcCsr4           = 6;

    localparam mcTrap0          = 7;
    localparam mcTrap1          = 8;
    localparam mcTrap2          = 9;
    localparam mcWaitForInt     = 10;
    localparam mcMem1           = 11;
    localparam mcMem2           = 12;
    localparam mcMulDiv1        = 13;
    localparam mcMulDiv2        = 14;

    localparam NumStates        = 16;





    // Fetching
    reg [31:0] Insn_d;
    reg [5:0] RdNo1;
    reg [5:0] RdNo2;
    reg [31:0] DelayedInsn_d;

    reg [WORD_WIDTH-1:0] FetchAddr_d;
    reg [WORD_WIDTH-1:0] PC_d;
    reg [31:0] PartialInsn_d;
    reg IncFetchAddr_v;

    // Decoding
    reg ExcIllegal_d;
    reg RetiredD_d;
    reg WrEnD_d;
    reg [5:0] WrNoD_d;

    reg CsrFromExtD_d;

    reg InsnBEQ;
    reg InsnBLTorBLTU;
    reg InsnJALR;
    reg InsnJAL;

    reg NegB;
    reg SelSum;
    reg SetCond;
    reg EnShift;
    reg SelImm_w;
    reg [1:0] LogicOp_d;
    reg EnLogic_d;
    reg ReturnUI;

    reg MemStore_d;
    reg [1:0] MemWidthD_d;
    reg MemAccessD_d;

    reg MC_d;
    reg MCModRdNo_d;
    reg NextInsnHold_d;

    // MC exclusive
    reg [5:0] MCAux_d;

    reg OverPcD_d;
    reg InsnCEBREAK_d;
    reg OverInsnD_d;
    reg TrapReturn_d;
    reg TrapIllegal_d;

    reg OverRs1_d;
    reg OverImm5_d;

    reg Overwrite_d;              // overwrite result in M-stage (used by exceptions)
    reg MCOverSelDivD_d;

    reg [NumStates-1:0] MCState_d;

    reg PartialHold_v;

    always @* begin


        ////////////
        // decode
        ////////////


        ExcIllegal_d = 0;
        RetiredD_d = 0;
        WrEnD_d = 0;
        WrNoD_d = 0; // don't care

        PC_d = PC_q;
        RdNo1 = 0;
        RdNo2 = 0;
        IncFetchAddr_v = 0;

        CsrFromExtD_d = 0;

        InsnBEQ         = 0;
        InsnBLTorBLTU   = 0;
        InsnJALR        = 0;
        InsnJAL         = 0;

        NegB            = 0;
        SelSum          = 0;
        SetCond         = 0;
        EnShift         = 0;
        SelImm_w        = 0;
        LogicOp_d       = 0; // don't care
        EnLogic_d       = 0;
        ReturnUI        = 0;

        MemStore_d      = 0;
        MemWidthD_d     = 2'b11; // no memory access
        MemAccessD_d     = 0;

        MC_d                = 0;
        MCModRdNo_d         = 0;
        NextInsnHold_d      = 0;
        MCAux_d             = MCAux_q;
        MCOverSelDivD_d     = 0;
        InsnCEBREAK_d  = 0;
        OverRs1_d       = 0;
        OverImm5_d       = 0;
        TrapIllegal_d     = 0;

        Overwrite_d         = 0;
        TrapReturn_d        = 0;

        MCState_d = 0;

        if (m_Kill) begin
            IncFetchAddr_v = 1; // continue fetching
            PC_d = {FetchAddr_q[WORD_WIDTH-1:2], MisalignedJumpE_q, 1'b0};
            MC_d = MisalignedJumpE_q;
            MCState_d = 1<<mcLast;
        end else if (ExcIllegal_q) begin
            TrapIllegal_d = 1;
            MC_d = 1;
            MCModRdNo_d = 1;
            MCState_d = 1<<mcTrap0;



        // microcoded cycles of multi-cycle instructions
        end else if (MC_q) begin
            MCState_d = 0;
            LogicOp_d = 2'b10;

            (* parallel_case *)
            case (1'b1)
                MCState_q[mcLast]: begin
                    IncFetchAddr_v = 1;
                end

                MCState_q[mcSysInsn]: begin
                    MCState_d = 1<<mcTrap1;
                    case (MCInsn_q[31:20])
                        12'h000: begin // ECALL
                            RdNo1 = REG_CSR_MTVEC;
                            Overwrite_d = 1;
                            WrNoD_d = REG_CSR_MCAUSE;
                        end
                        12'h001: begin // EBREAK
                            RdNo1 = REG_CSR_MTVEC;
                            Overwrite_d = 1;
                            WrNoD_d = REG_CSR_MCAUSE;
                        end
                        12'h002: begin // URET
                            RdNo1 = REG_CSR_MEPC;
                            TrapReturn_d = 1;
                        end
                        12'h102: begin // SRET
                            RdNo1 = REG_CSR_MEPC;
                            TrapReturn_d = 1;
                        end
                        12'h302: begin // MRET
                            RdNo1 = REG_CSR_MEPC;
                            TrapReturn_d = 1;
                        end
                        12'h105: begin // WFI
                            // RDNo1 don't care
                            MCState_d = 1<<mcWaitForInt;
                        end
                        default: begin
                            RdNo1 = REG_CSR_MTVEC;
                            Overwrite_d = 1;
                            WrNoD_d = REG_CSR_MCAUSE;
                        end
                    endcase
                    WrNoD_d = REG_CSR_MCAUSE;
                    MCModRdNo_d = 1; // for WFI
                    NextInsnHold_d = 1; // for WFI
                    MC_d  = 1;
                end

                // CSR access: decide if from regset or external interface
                MCState_q[mcCsr1]: begin
                    CsrFromExtD_d  = ~CsrFromReg_d;

                    WrEnD_d        = 1;
                    WrNoD_d        = REG_CSR_TMP2;
                    RdNo1          = {1'b1, CsrTranslateD_d};
                    RdNo2          = 0;

                    MC_d           = 1;
                    MCModRdNo_d    = 1;
                    NextInsnHold_d = 1;
                    MCState_d      = 1<<mcCsr2;
                end

                // 3rd cycle of CSR access
                // or tmp2, csr, x0
                MCState_q[mcCsr2]: begin
                    LogicOp_d = 2'b10; // or
                    EnLogic_d = 1;
                    TrapIllegal_d = CsrFromExtE_q & ~CsrValid_w;

                    WrEnD_d   = ~CsrFromExtE_q;
                    WrNoD_d   = REG_CSR_TMP2;
                    RdNo1 = MCInsn_q[13]
                        ? REG_CSR_TMP2 // CSRS, CSRC
                        : 0; // CSRW
                    RdNo2 = REG_CSR_TMP;

                    MC_d = 1;
                    MCModRdNo_d = 1;
                    NextInsnHold_d = 1;
                    MCState_d = TrapIllegal_d
                        ? (1<<mcTrap0) // illegal CSR -> exception
                        : (1<<mcCsr3);
                end

                // 4th cycle of CSR access
                // CSRW: or csr, x0, tmp
                // CSRS: or csr, tmp2, tmp
                // CSRC: and csr, tmp2, not tmp
                MCState_q[mcCsr3]: begin
                    LogicOp_d = {1'b1, MCInsn_q[13] & MCInsn_q[12]};
                        // and for CSRC, or for CSRS and CSRW
                    EnLogic_d = 1;
                    NegB = MCInsn_q[13] & MCInsn_q[12];
                        // not for CSRC

                    WrEnD_d = ~CsrFromExtM_q;
                    WrNoD_d = {1'b1, CsrTranslateE_q};
                    RdNo1 = REG_CSR_TMP2;
                    RdNo2 = 0;

                    MC_d = 1;
                    NextInsnHold_d = 1;
                    MCState_d = 1<<mcCsr4;
                end

                // or rd, tmp2, 0
                MCState_q[mcCsr4]: begin
                    LogicOp_d = 2'b10; // or
                    EnLogic_d = 1;
                    //CsrWrite_d   = (~MCInsn_q[13] | (MCInsn_q[19:15]!=0));

                    WrEnD_d = (MCInsn_q[11:7] != 0);
                    WrNoD_d = {1'b0, MCInsn_q[11:7]};

                    //MC_d = 0;
                    //IncFetchAddr_v = 1;
                    MC_d = 1;
                    NextInsnHold_d = 1;
                    MCState_d = 1<<mcLast;
                end

                // Innterrupt or illegal insn or C.BREAK
                MCState_q[mcTrap0]: begin
                    Overwrite_d = 1;
                    WrNoD_d = REG_CSR_MCAUSE;
                    RdNo1 = REG_CSR_MTVEC;

                    MC_d = 1;
                    MCModRdNo_d = 1;
                    MCState_d = 1<<mcTrap1;
                end


                // enter or return from trap
                // enter: jump to MTVEC and write MTVAL
                // return: jump to MEPV and don't write CSR
                MCState_q[mcTrap1]: begin
                    // EBREAK exception: MTVAL=PC
                    // Illegal instruction exception: MTVAL=insn
                    // otherwise: MTVAL=0
                    //OverPcE_d = OverPcE_q;
                    //OverInsnE_d = OverInsnE_q;

                    InsnJALR = 1;
                    // Imm12PlusReg_w is implicitly set to 0

                    Overwrite_d = ~TrapReturn_q; // don't write if returning
                    WrNoD_d = REG_CSR_MTVAL;

                    MC_d = 1;
                    MCModRdNo_d = 1;
                    MCState_d = 1<<mcTrap2;
                end

                // 2nd cycle of trap: write MEPC
                // FIXME: don't write if TrapReturn
                MCState_q[mcTrap2]: begin
                    //OverPcE_d = 1;

                    Overwrite_d = 1;
                    WrNoD_d = REG_CSR_MEPC;
                    MC_d = 0;

                    MC_d = 1;
                    MCState_d = 1<<mcLast;

                    // This way, the next insn is a MC nop
                    // Other solution: decode but kill next instruction
                    //MC_d = 0;
                end

                MCState_q[mcWaitForInt]: begin
                    MC_d = 1;
                    MCModRdNo_d = f_MModeIntEnable; // for mcTrap0
                    NextInsnHold_d = 1; // for mcLast
                    MCState_d = AnyIrq_w
                        ? (f_MModeIntEnable ? (1<<mcTrap0)
                                            : (1<<mcLast))
                        :                     (1<<mcWaitForInt);

/* simpler, but longer response time:
                    MC_d = 1;
                    NextInsnHold_d = 1;
                    MCState_d = AnyIrq_w
                        ? (1<<mcLast)
                        : (1<<mcWaitForInt);
*/


/* wfi as nop:
                    MC_d = 1;
                    NextInsnHold_d = 1;
                    MCState_d = (1<<mcLast);
*/
                end

                MCState_q[mcMem1]: begin
                    if (MemMisalignedE_d) begin
                        MemStore_d   = MemStore_q;
                        MemWidthD_d  = MemWidthE_q;
                        MemAccessD_d = 1;

                        WrEnD_d      = WrEnE_q;
                        WrNoD_d      = WrNoE_q;
                        MC_d         = 1;
                        NextInsnHold_d = 1;
                        MCState_d = 1<<mcMem2;
                    end
                end

                MCState_q[mcMem2]: begin
                    MC_d = 1;
                    MCState_d = 1<<mcNop;
                end

                MCState_q[mcMulDiv1]: begin
                    MCAux_d = 31;

                    WrNoD_d = WrNoE_q;
                    MC_d    = 1;
                    NextInsnHold_d = 1;
                    MCState_d = 1<<mcMulDiv2;
                end

                MCState_q[mcMulDiv2]: begin
                    MCAux_d = MCAux_q - 1;

                    WrNoD_d = WrNoE_q;
                    MC_d = 1;
                    NextInsnHold_d = 1;
                    MCState_d = 1<<mcMulDiv2;
                    if (MCAux_q == 0) begin
                        Overwrite_d = (WrNoE_q != 0);
                        WrEnD_d = Overwrite_d;
                            // required for forwarding
                        MCOverSelDivD_d = 1;
                        MCState_d = 1<<mcLast;
                    end
                end
            endcase

        end else if (IrqResponse_q) begin
            MC_d = 1;
            MCModRdNo_d = 1;
            MCState_d = 1<<mcTrap0;


        // 32 bit instruction
        end else if (Insn_q[1:0]==2'b11) begin
            ExcIllegal_d = 1;
            IncFetchAddr_v = 1;
            PC_d = PC_q + 4;

            WrEnD_d = (Insn_q[11:7]!=0); // do not write to x0
            WrNoD_d = {1'b0, Insn_q[11:7]};

            case ({Insn_q[6], Insn_q[4:2]})
                4'b0101: begin // LUI or AUIPC
                    ExcIllegal_d   = 0;
                    ReturnUI       = 1;
                end
                4'b1011: begin // JAL
                    ExcIllegal_d   = ~Insn_q[5];
                    InsnJAL        = ~ExcIllegal_d; // jump disables exception
                end
                4'b1001: begin // JALR
                    ExcIllegal_d   = ~Insn_q[5] | (Insn_q[14:12]!=3'b000);
                    InsnJALR       = ~ExcIllegal_d; // jump disables exception
                end
                4'b1000: begin // branch
                    ExcIllegal_d   = ~Insn_q[5] | (Insn_q[14:13]==2'b01);
                    WrEnD_d        = 0;
                    InsnBEQ        = Insn_q[5] & (Insn_q[14:13]==2'b00);
                    InsnBLTorBLTU  = Insn_q[5] & Insn_q[14];
                    NegB           = 1;
                    LogicOp_d      = 2'b00;
                    EnLogic_d      = InsnBEQ;
                end
                4'b0000: begin // load or store
                    //ExcIllegal_d = ~Insn_q[5]
                    //    ? (Insn_q[13] & (Insn_q[14] | Insn_q[12]))
                    //    : (Insn_q[14] | (Insn_q[13] & Insn_q[12]));
                    ExcIllegal_d = (Insn_q[13] & (Insn_q[14] | Insn_q[12]))
                                   | (Insn_q[5] & Insn_q[14]);
                    WrEnD_d        = WrEnD_d & ~Insn_q[5];
                    MemStore_d     = Insn_q[5];
                    MemWidthD_d    = Insn_q[13:12];
                    MemAccessD_d   = 1;
                    MCState_d      = 1<<mcMem1;
                end
                4'b0100: begin // arithmetic
                    if (~Insn_q[5]) begin // immediate
                        ExcIllegal_d = ~Insn_q[13] & Insn_q[12] &
                            (Insn_q[31] | (Insn_q[29:25]!=0) |
                            (Insn_q[30] & ~Insn_q[14]));
                        SelImm_w  = 1; // only for forwarding
                        NegB = ~Insn_q[14] & (Insn_q[13] | Insn_q[12]);
                            // SLLI, SLTI, SLTIU
                    end else begin
                        if (Insn_q[25]) begin // RVM
                            ExcIllegal_d = (Insn_q[31:26]!=0);
                            IncFetchAddr_v = 0;
                                // increment not before last cycle
                            NextInsnHold_d = 1;
                            MCState_d = 1<<mcMulDiv1;
                        end else begin // arith
                            ExcIllegal_d = Insn_q[31] | (Insn_q[29:26]!=0) |
                                (Insn_q[30] & (Insn_q[13] | (Insn_q[14]!=Insn_q[12])));
                            NegB     = ~Insn_q[14] & // SUB, SLL, SLT, SLTU
                                       (Insn_q[13] | Insn_q[12] | Insn_q[30]);
                        end
                    end
                    SelSum   = ~Insn_q[14] & ~Insn_q[13] & ~Insn_q[12]; // ADD, SUB
                    SetCond  = ~Insn_q[14] &  Insn_q[13]; // SLT, SLTU
                    EnShift  = ~Insn_q[13] &  Insn_q[12]; // SLL, SRA, SRL
                    LogicOp_d = Insn_q[13:12]; // AND, OR, XOR
                    EnLogic_d = Insn_q[14] & (Insn_q[13:12]!=2'b01);
                end
                4'b0011: begin // fence
                    ExcIllegal_d = Insn_q[14] | Insn_q[13];
                    WrEnD_d = 0;
                    InsnJAL = Insn_q[12] & ~ExcIllegal_d;
                end
                4'b1100: begin // system
                    ExcIllegal_d    = 1;
                    IncFetchAddr_v  = 0;
                    Overwrite_d     = 1;
                    WrEnD_d         = 1;
                    WrNoD_d         = REG_CSR_TMP;
                    OverRs1_d   = ~Insn_q[14];
                    OverImm5_d   = Insn_q[14];
                    NextInsnHold_d  = 1;
                    MCModRdNo_d     = 1;
                    if (Insn_q[5]) begin
                        if (Insn_q[13] | Insn_q[12]) begin // RVZicsr
                            if (~Insn_q[31] | ~Insn_q[30] // rw-CSR or
                                | (Insn_q[13] & (Insn_q[19:15]==0))) 
                                // ro-CSR and CSRRS/C and rs1=0
                            begin
                                ExcIllegal_d = 0;
                            end
                            MCState_d = 1<<mcCsr1;
                        end else begin
                            if (Insn_q[19:7]==0) begin
                                ExcIllegal_d = 0;
                            end
                            MCState_d = 1<<mcSysInsn;
                        end
                    end
                end
                default: begin
                end
            endcase
            MC_d = MemAccessD_d | NextInsnHold_d;
            RetiredD_d = ~ExcIllegal_d;

        // compressed instruction
        end else begin
            ExcIllegal_d = 1;
            if (~OddPC_q) IncFetchAddr_v = 1;
            PC_d = PC_q + 2;

            WrEnD_d = (Insn_q[11:7]!=0);
            WrNoD_d = {1'b0, Insn_q[11:7]};

            case ({Insn_q[14], Insn_q[1:0]})
                3'b000: begin // C.ADDI4SPN
                    ExcIllegal_d = Insn_q[15] | Insn_q[13] | (Insn_q[12:5]==0);
                    SelSum = 1;
                    SelImm_w = 1;
                    WrEnD_d = 1;
                    WrNoD_d = {3'b001, Insn_q[4:2]};
                end
                3'b001: begin
                    ExcIllegal_d = Insn_q[15] & ~Insn_q[13] & Insn_q[12] 
                        & (Insn_q[11:10]!=2'b10);
                    if (Insn_q[13]) begin // C.JAL or C.J
                        WrEnD_d = ~Insn_q[15]; // 1 for C.JAL
                        WrNoD_d = 1;
                        InsnJAL = ~ExcIllegal_d;
                    end else begin
                        if (~Insn_q[15]) begin // C.ADDI
                            ExcIllegal_d = 0;
                            SelSum = 1;
                            SelImm_w = 1;
                        end else begin
                            WrEnD_d = 1;
                            WrNoD_d = {3'b001, Insn_q[9:7]};
                            if (Insn_q[11]) begin
                                if (Insn_q[10]) begin
                                    case (Insn_q[6:5])
                                        2'b00: begin // C.SUB
                                            NegB = 1;
                                            SelSum = 1;
                                        end
                                        2'b01: begin // C.XOR
                                            LogicOp_d = 2'b00;
                                            EnLogic_d = 1;
                                        end
                                        2'b10: begin // C.OR
                                            LogicOp_d = 2'b10;
                                            EnLogic_d = 1;
                                        end
                                        2'b11: begin // C.AND
                                            LogicOp_d = 2'b11;
                                            EnLogic_d = 1;
                                        end
                                    endcase
                                end else begin // C.ANDI
                                    LogicOp_d = 2'b11;
                                    EnLogic_d = 1;
                                    SelImm_w = 1;
                                end
                            end else begin // C.SRLI and C.SRAI
                                EnShift = 1;
                                    // ShiftRight_q and ShiftArith_q are set independently
                                SelImm_w = 1;
                            end
                        end
                    end
                end
                3'b010: begin
                    if (~Insn_q[15]) begin  // C.SLLI
                        ExcIllegal_d = Insn_q[13] | Insn_q[12];
                        NegB = 1;
                        EnShift = 1;
                            // ShiftRight_q and ShiftArith_q are set independently
                        SelImm_w = 1;
                    end else begin
                        ExcIllegal_d = Insn_q[13];
                        if (Insn_q[6:2]==0) begin
                            if (Insn_q[12]) begin
                                if (Insn_q[11:7]==0) begin // C.EBREAK
                                    ExcIllegal_d = 0;
                                    InsnCEBREAK_d = 1;
                                    MCModRdNo_d = 1;
                                    MCState_d = 1<<mcTrap0;
                                end else begin // C.JALR
                                    WrEnD_d = 1;
                                    WrNoD_d = 1;
                                    InsnJALR = ~ExcIllegal_d;
                                end
                            end else begin // C.JR
                                ExcIllegal_d = (Insn_q[11:7]==0);
                                WrEnD_d = 0;
                                InsnJALR    = ~ExcIllegal_d;
                            end
                        end else begin
                            SelSum = Insn_q[12]; // C.ADD
                            EnLogic_d = ~Insn_q[12]; // C.MV
                            LogicOp_d = 2'b01; // copy rs2
                        end
                    end
                end
                3'b100: begin // C.LW or C.SW
                    ExcIllegal_d = Insn_q[13];
                    WrEnD_d        = ~Insn_q[15]; // 1 for C.LW
                    WrNoD_d        = {3'b001, Insn_q[4:2]};
                    MemStore_d     = Insn_q[15]; // 1 for C.SW
                    MemWidthD_d    = 2'b10;
                    MemAccessD_d   = 1;
                    MCState_d      = 1<<mcMem1;
                end
                3'b101: begin
                    ExcIllegal_d = 0;
                    LogicOp_d = 2'b00; // only cares for branches
                    if (~Insn_q[15]) begin
                        if (Insn_q[13] & Insn_q[11:7]==2) begin // C.ADDI16SP
                            SelSum = 1;
                            SelImm_w = 1;
                        end else begin // C.LI or C.LUI
                            ReturnUI = 1;
                        end
                    end else begin // C.BEQZ ot C.BNEZ
                        // InvertBranch_q is set independently
                        WrEnD_d   = 0;
                        InsnBEQ   = 1;
                        NegB      = 1;
                        EnLogic_d = 1;
                    end
                end
                3'b110: begin // C.LWSP
                    ExcIllegal_d = Insn_q[13] | (~Insn_q[15] & (Insn_q[11:7]==0));
                    MemWidthD_d    = 2'b10;
                    MemAccessD_d   = 1;
                    WrEnD_d        = ~Insn_q[15]; // 1 for C.LWSP
                    MemStore_d     =  Insn_q[15]; // 1 for C.SWSP
                    MCState_d      = 1<<mcMem1;
                end
                default: begin
                    ExcIllegal_d = 1;
                end
            endcase
            MC_d = MemAccessD_d | InsnCEBREAK_d;
            RetiredD_d = ~ExcIllegal_d;
        end






        ////////////
        // fetch
        ////////////



        // FetchAddr_d
        if (Branch_w) begin
            FetchAddr_d = {e_PCImm[WORD_WIDTH-1:2], 2'b00};
        end else if (e_InsnJALR & ~m_Kill) begin
            FetchAddr_d = {AddrSum_w[WORD_WIDTH-1:2], 2'b00};
        end else if (IncFetchAddr_v) begin
            FetchAddr_d = NextFetchAddr_w;
        end else begin
            FetchAddr_d = FetchAddr_q;
        end


        // Insn_d
        if (m_Kill | (~NextInsnHold_q & ~MemAccessEorM_q
                        & (~OddPC_q | ~CompressedInsn_w)))
        begin
            Insn_d[31:16] =
                (~m_Kill & (OddPC_q | CompressedInsn_w))
                    ? mem_rdata[15:0]
                    : mem_rdata[31:16];
        end else if (NextInsnHold_q) begin
            Insn_d[31:16] = Insn_q[31:16];
        end else case ({OddPC_q, CompressedInsn_w})
            2'b00:   Insn_d[31:16] = DelayedInsn_q[31:16];
            2'b11:   Insn_d[31:16] = PartialInsn_q[31:16];
            default: Insn_d[31:16] = DelayedInsn_q[15:0];
        endcase

        if (m_Kill | (~NextInsnHold_q & ~MemAccessEorM_q
                        & ~OddPC_q & ~CompressedInsn_w))
        begin
            Insn_d[15:0] = mem_rdata[15:0];
        end else if (NextInsnHold_q) begin
            Insn_d[15:0] = Insn_q[15:0];
        end else case ({OddPC_q, CompressedInsn_w})
            2'b00:   Insn_d[15:0] = DelayedInsn_q[15:0];
            2'b11:   Insn_d[15:0] = PartialInsn_q[15:0];
            default: Insn_d[15:0] = PartialInsn_q[31:16];
        endcase


        // DelayedInsn_d
        if (m_Kill | ~( (OddPC_q & CompressedInsn_w) |
                         NextInsnHold_q |
                        (MemAccessEorM_q & MemAccessD_d)))
        begin
            DelayedInsn_d = mem_rdata;
        end else if (OddPC_q & CompressedInsn_w) begin
            DelayedInsn_d = PartialInsn_q;
        end else begin
            DelayedInsn_d = DelayedInsn_q;
        end

        // PartialInsn_d
        if ((~m_Kill & OddPC_q & CompressedInsn_w) |
            NextInsnHold_q | MemAccessD_d)
        begin
            PartialInsn_d = PartialInsn_q;
        end else if (MemAccessEorM_q) begin
            PartialInsn_d = DelayedInsn_q;
        end else begin
            PartialInsn_d = mem_rdata;
        end


        // register numbers for next insn
        if (m_Kill | ~MCModRdNo_q) begin

/*
            if (~Insn_d[1]) begin
                if (~Insn_d[0]) begin // 00
                    if (Insn_d[14:13]==2'b00)   RdNo1 = 2; // C.ADDI4SPN
                    else                        RdNo1 = {3'b001, Insn_d[9:7]};  // RVC rs1'
                end else begin // 01
                    if (~Insn_d[15])            RdNo1 = {1'b0, Insn_d[11:7]};   // RVC rs1
                    else                        RdNo1 = {3'b001, Insn_d[9:7]};  // RVC rs1'
                end
            end else begin
                if (~Insn_d[0]) begin // 10
                    if (Insn_d[14:13]==2'b00) begin
                                                RdNo1 = {1'b0, Insn_d[11:7]};   // RVC rs1
                    end else                    RdNo1 = 2;
                end else begin // 11
                                                RdNo1 = {1'b0, Insn_d[19:15]};  // RVI rs1
                end
            end

            RdNo1 = ~Insn_d[0]
                ? (~Insn_d[1]
                    ? (~Insn_d[14]
                        ? 6'b000010             // x2
                        : {3'b001, Insn_d[9:7]} // RVC rs1'
                      )
                    : (~Insn_d[14]
                        ? {1'b0, Insn_d[11:7]}  // RVC rs1
                        : 6'b000010             // x2
                      )
                  )
                : (~Insn_d[1]
                    ? (~Insn_d[15]
                        ? {1'b0, Insn_d[11:7]}  // RVC rs1
                        : {3'b001, Insn_d[9:7]} // RVC rs1'
                      )
                    : {1'b0, Insn_d[19:15]}     // RVI rs1
                  )
                ;

        Optimised by hand (10% increase in clockrate with IceStorm):
*/
            RdNo1[5] = 1'b0;
            RdNo1[4] = ~Insn_d[0]
                ? (Insn_d[1] & ~Insn_d[14] & Insn_d[11])
                : (~Insn_d[1]
                    ? (~Insn_d[15] & Insn_d[11])
                    : Insn_d[19]
                  );
            RdNo1[3] = ~Insn_d[0]
                ? (~Insn_d[1]
                    ? Insn_d[14]
                    : (~Insn_d[14] & Insn_d[10])
                  )
                : (~Insn_d[1]
                    ? (Insn_d[15] | Insn_d[10])
                    : Insn_d[18]
                  );
            RdNo1[2] = ~Insn_d[0]
                ? ((Insn_d[1] ^ Insn_d[14]) & Insn_d[9])
                : (~Insn_d[1] ? Insn_d[9] : Insn_d[17]);
            RdNo1[1] = ~Insn_d[0]
                ? (Insn_d[8] | ~(Insn_d[1] ^ Insn_d[14]))
                : (~Insn_d[1] ? Insn_d[8] : Insn_d[16]);
            RdNo1[0] = ~Insn_d[0]
                ? ((Insn_d[1] ^ Insn_d[14]) & Insn_d[7])
                : (~Insn_d[1] ? Insn_d[7] : Insn_d[15]);


/*
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
*/

            RdNo2[5] = 0;
            RdNo2[4] = Insn_d[1] & (Insn_d[0] ? Insn_d[24] : Insn_d[6]);
            RdNo2[3] = Insn_d[1]
                ? (Insn_d[0] ? Insn_d[23] : Insn_d[5])
                : (~Insn_d[0] | ~Insn_d[14]);
            RdNo2[2] = (Insn_d[0] & Insn_d[1] & Insn_d[22])
                | ((~Insn_d[0] | (~Insn_d[1] & ~Insn_d[14])) & Insn_d[4]);
            RdNo2[1] = (Insn_d[0] & Insn_d[1] & Insn_d[21])
                | ((~Insn_d[0] | (~Insn_d[1] & ~Insn_d[14])) & Insn_d[3]);
            RdNo2[0] = (Insn_d[0] & Insn_d[1] & Insn_d[20])
                | ((~Insn_d[0] | (~Insn_d[1] & ~Insn_d[14])) & Insn_d[2]);




        end else begin

            RdNo1 = REG_CSR_MTVEC;
            RdNo2 = 0;

            if (MCState_q[mcSysInsn]) begin
                if (MCInsn_q[21]) RdNo1 = REG_CSR_MEPC; // xRET
                             else RdNo1 = REG_CSR_MTVEC; // ECALL/EBREAK/illegal
            end
            if (MCState_q[mcCsr1]) begin
                RdNo1         = {1'b1, CsrTranslateD_d};
                RdNo2         = 0;
            end
            if (MCState_q[mcCsr2]) begin
                RdNo1 = MCInsn_q[13]
                    ? REG_CSR_TMP2 // CSRS, CSRC
                    : 0; // CSRW
                RdNo2 = REG_CSR_TMP;
            end
            if (MCState_q[mcCsr3]) begin
                RdNo1 = REG_CSR_TMP2;
                RdNo2 = 0;
            end

            // already covered by default values:
            //
            //if (MCState_q[mcTrap0]) begin
            //    RdNo1 = REG_CSR_MTVEC;
            //end
            //if (MCState_q[mcWaitForInt]) begin
            //    RdNo1 = REG_CSR_MTVEC;
            //end

        end

    end





    wire MemAccessE_d = MemAccessE_q & ~m_Kill;

    wire MisalignedJumpD_d = e_InsnJALR ? AddrSum_w[1] : e_PCImm[1];
            // only valid when Kill is set

    wire AddrFromSum_d = InsnJALR | MemAccessD_d;
    wire ReturnPC_d    = InsnJALR | InsnJAL;

    wire AnyIrq_w = SoftwareIrq_q | TimerIrq_q | ExternalIrq_q;
    wire IrqResponse_d = 
        f_MModeIntEnable &
        ~CsrWrite_q & 
            // don't allow interrupt in the cycle after a CSR modification,
            // because the flags (MSTATUES.MIE, ...)  need an additional
            // cycle until their modified values are visible
        AnyIrq_w;
//        (SoftwareIrq_q | TimerIrq_q | ExternalIrq_q);

    wire CsrWrite_d = MCState_q[mcCsr4] & (~MCInsn_q[13] | (MCInsn_q[19:15]!=0));

    wire WrEnE_d = ~m_Kill & WrEnE_q;
    wire WrEnM_d = WrEnM_q | OverwriteM_q;

    wire CompressedInsn_w = ~(MC_q | (Insn_q[1] & Insn_q[0]));
    wire OddPC_d = m_Kill ? MisalignedJumpE_q : (OddPC_q ^ CompressedInsn_w);






    // forwarding

    wire FwdAE = WrEnE_q & (d_RdNo1 == WrNoE_q);
    wire FwdAM = WrEnM_q & (d_RdNo1 == WrNoM_q);
    wire FwdAW = WrEnW_q & (d_RdNo1 == WrNoW_q);
    wire [WORD_WIDTH-1:0] ForwardAR = (FwdAE | FwdAM | FwdAW) ? 0 : regset_rd1;
    wire [WORD_WIDTH-1:0] ForwardAM = FwdAM ? WrDataM_d : (FwdAW ? WrDataW_q : 0);
    wire [WORD_WIDTH-1:0] ForwardAE = FwdAE ? WrDataE_d : (ForwardAR | ForwardAM);

    wire FwdBE = WrEnE_q & (d_RdNo2 == WrNoE_q) & ~SelImm_w;
    wire FwdBM = WrEnM_q & (d_RdNo2 == WrNoM_q) & ~SelImm_w;
    wire FwdBW = WrEnW_q & (d_RdNo2 == WrNoW_q);
    wire [WORD_WIDTH-1:0] ForwardImm = SelImm_w ? ImmALU_w : 0;
    wire [WORD_WIDTH-1:0] ForwardBR = SelImm_w ?    0 : (FwdBW ? WrDataW_q : regset_rd2);
    wire [WORD_WIDTH-1:0] ForwardBM =  FwdBM ? WrDataM_d : (ForwardBR | ForwardImm);
    wire [WORD_WIDTH-1:0] ForwardBE = (FwdBE ? WrDataE_d : ForwardBM) ^ {WORD_WIDTH{NegB}};






    // ALU




    wire [WORD_WIDTH-1:0] LogicResult_w = ~LogicOp_q[1]
        ? (~LogicOp_q[0] ? (e_A ^ e_B) : e_B)
        : (~LogicOp_q[0] ? (e_A | e_B) : (e_A & e_B));
    wire [WORD_WIDTH-1:0] PCResult_w =
          (ReturnPC_q ? PC_q : 0);
    wire [WORD_WIDTH-1:0] UIResult_w =
          (e_ReturnUI ? ImmUpper_q : 0);

    // OPTIMIZE? vFastResult has one input left
    wire [WORD_WIDTH-1:0] vFastResult = EnLogic_q
        ? LogicResult_w
        : (UIResult_w | PCResult_w);
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
    wire [WORD_WIDTH-1:0] WrDataE_d = (e_B[0] ? vShift4[32:1] : vShift4[31:0]) | vShiftAlternative;








    // branch unit

    wire vEqual = (LogicResult_w == ~0);
    wire vLessXor = InvertBranch_q ^ ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));
    wire vLess = (Sum[31] & (e_A[31] ^ e_B[31])) ^ vLessXor;
    wire vBEQ = e_InsnBEQ & (InvertBranch_q ^ vEqual) & ~m_Kill;
    wire vNotBEQ = ((e_InsnBLTorBLTU & vLess) | e_InsnJAL) & ~m_Kill;
    wire vCondResultBit = e_SetCond & vLess;

    wire Branch_w = vBEQ | vNotBEQ;

/* Sum[31] critical:

    wire vEqual = (LogicResult_w == ~0);
    wire vLessXor = InvertBranch_q ^ ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));

    wire vLess = (Sum[31] & (e_A[31] ^ e_B[31])) ^ vLessXor;
    wire vCondResultBit = e_SetCond & vLess;

    wire BranchSel0_w = ~m_Kill & (~e_InsnBLTorBLTU | vLessXor);
    wire BranchSel1_w = ~m_Kill & e_InsnBLTorBLTU & (e_A[31] ^ e_B[31]);
    wire BranchC_w    = e_InsnJAL | e_InsnBLTorBLTU;
    wire BranchB_w    = e_InsnBEQ & (InvertBranch_q ^ vEqual) | BranchC_w;
    wire Branch_w     = BranchSel1_w ? (BranchSel0_w ^ Sum[31])
                                     : (BranchSel0_w & BranchB_w);
*/

/* clean code:

    wire Equal_w = (vLogicResult == ~0);
    wire Less_w = (Sum[31] & (e_A[31] ^ e_B[31])) ^
                 ((e_A[31] ^ e_LTU) & (e_B[31] ^ e_LTU));
    wire vCondResultBit = e_SetCond & Less_w;
    wire Branch_w = ~m_Kill & (
        (e_InsnBEQ       & (InvertBranch_q ^ Equal_w)) |
        (e_InsnBLTorBLTU & (InvertBranch_q ^  Less_w)) |
         e_InsnJAL
    );
*/


    wire Kill = Branch_w | (e_InsnJALR & ~m_Kill);
        // any jump or exception

    wire [WORD_WIDTH-1:0] AddrSum_w = e_A +
        {{(WORD_WIDTH-12){Imm12PlusReg_q[11]}}, Imm12PlusReg_q};
    wire [WORD_WIDTH-1:0] AddrSum2_w =
        MemMisalignedM_q ? {AddrSumAligned_q, 2'b00} + 4 : AddrSum_w;

    wire [WORD_WIDTH-1:0] NextFetchAddr_w = FetchAddr_q + 4;

    wire [WORD_WIDTH-1:0] MemAddr =
        Branch_w // taken PC-relative branch
            ? {e_PCImm[WORD_WIDTH-1:2], 2'b00}
            : (AddrFromSum_q & ~m_Kill)
                ? {AddrSum2_w[WORD_WIDTH-1:1], 1'b0}
                : ((~m_Kill & OddPC_q & CompressedInsn_w) | MemMisalignedW_q)
                    ? FetchAddr_q
                    : NextFetchAddr_w;







    // mul/div unit

    // control signals for first cycle
    wire DivSigned_w = ~MCInsn_q[12] & MCInsn_q[14];
    wire MulASigned_w = (MCInsn_q[13:12] != 2'b11) & ~MCInsn_q[14];
    wire InsnMULH_w = (MCInsn_q[14:12]==3'b001);

    // control signals for every cycle
    wire SelRemOrDiv_w = MCInsn_q[13] | (MCInsn_q[14:12]==3'b001);
    wire SelDivOrMul_w = MCInsn_q[14];


    // control signals for last cycle
    wire SelMulLowOrHigh_w = (MCInsn_q[13:12]==2'b00);

    wire [WORD_WIDTH:0] DivDiff_w = DivRem_q - {1'b0, Long_q[WORD_WIDTH-1:0]};
    wire DivNegative_w = (Long_q[2*WORD_WIDTH-1:WORD_WIDTH]!=0) | DivDiff_w[WORD_WIDTH];
    wire [WORD_WIDTH+1:0] LongAdd_w =
        {Long_q[2*WORD_WIDTH], Long_q[2*WORD_WIDTH:WORD_WIDTH]}
            + (Long_q[0] ? {DivRem_q[WORD_WIDTH], DivRem_q}
                         : {(WORD_WIDTH+2){1'b0}});

    wire [WORD_WIDTH-1:0] AbsA_w = (DivSigned_w & e_A[WORD_WIDTH-1]) ? (-e_A) : e_A;
    wire [WORD_WIDTH-1:0] AbsB_w = (DivSigned_w & e_B[WORD_WIDTH-1]) ? (-e_B) : e_B;

    wire [WORD_WIDTH-1:0] DivQuot_d = {DivQuot_q[WORD_WIDTH-2:0], ~DivNegative_w};

    reg MulDivResNeg_d;
    reg MulDivResAdd_d;
    reg [WORD_WIDTH:0] DivRem_d;
    reg [2*WORD_WIDTH:0] Long_d;
    reg [WORD_WIDTH-1:0] MulDivResult_d;

    always @* begin

        // mul/div arithmetic step
        if (MCState_q[mcMulDiv1]) begin
            MulDivResNeg_d =
                (DivSigned_w &
                (e_A[WORD_WIDTH-1] ^ (~SelRemOrDiv_w & e_B[WORD_WIDTH-1])) &
                (e_B!=0 || SelRemOrDiv_w))
                |
                (InsnMULH_w & e_B[WORD_WIDTH-1]);
            MulDivResAdd_d = SelDivOrMul_w
                |
                (InsnMULH_w & e_B[WORD_WIDTH-1]);

            DivRem_d = {MulASigned_w & e_A[WORD_WIDTH-1], AbsA_w};
            if (SelDivOrMul_w) begin
                Long_d = {2'b0, AbsB_w, {(WORD_WIDTH-1){1'b0}}};
            end else begin
                Long_d = {{(WORD_WIDTH+1){1'b0}}, e_B};
            end
        end else begin
            MulDivResNeg_d = MulDivResNeg_q;
            MulDivResAdd_d = MulDivResAdd_q;

            DivRem_d = (DivNegative_w | ~SelDivOrMul_w)
                ? DivRem_q
                : {1'b0, DivDiff_w[WORD_WIDTH-1:0]};
            Long_d = {LongAdd_w[WORD_WIDTH+1:0], Long_q[WORD_WIDTH-1:1]};
        end

        // mul/div result computation
        MulDivResult_d =
            (SelDivOrMul_w ? {WORD_WIDTH{1'b0}}
                           : (SelMulLowOrHigh_w ? Long_q[WORD_WIDTH-1:0]
                                                : Long_q[2*WORD_WIDTH-1:WORD_WIDTH]))
            +
            (MulDivResAdd_q ? NotDivRem_w : {WORD_WIDTH{1'b0}})
            +
            {{(WORD_WIDTH-1){1'b0}}, MulDivResNeg_q};


    end

    wire [WORD_WIDTH-1:0] DivRem_w = SelRemOrDiv_w
        ? DivRem_q[WORD_WIDTH-1:0]
        : DivQuot_q;

    wire [WORD_WIDTH-1:0] NotDivRem_w = MulDivResNeg_q ? (~DivRem_w) : DivRem_w;









    // register-mapped CSRs
/*
    reg [5:0] CsrTranslateD_d;
    reg CsrFromReg_d;
    always @* begin
        CsrFromReg_d <= 1;
        case (Insn_q[31:20])
            12'h305: CsrTranslateD_d <= REG_CSR_MTVEC;
            12'h340: CsrTranslateD_d <= REG_CSR_MSCRATCH;
            12'h341: CsrTranslateD_d <= REG_CSR_MEPC;
            12'h342: CsrTranslateD_d <= REG_CSR_MCAUSE;
            12'h343: CsrTranslateD_d <= REG_CSR_MTVAL;
            default: begin
                CsrTranslateD_d <= 0; // cannot be written, always 0
                CsrFromReg_d <= 0;
            end
        endcase
        CsrTranslateD_d <= {1'b1, Insn_q[26], Insn_q[23:20]};
    end
*/
    wire [4:0] CsrTranslateD_d = {MCInsn_q[26], MCInsn_q[23:20]};
    wire CsrFromReg_d = (MCInsn_q[31:27]==5'b00110) &&
        (MCInsn_q[25:23]==3'b0) &&
        (
         (MCInsn_q[22:20]==3'b100) || // mie, mip
         ((~MCInsn_q[26]) && (MCInsn_q[22:20]==3'b101)) || // mtvec
         ((MCInsn_q[26]) && (~MCInsn_q[22]))); // mscratch, mepc, mcause, mtval


    // external CSR interface

    reg         CsrWrite_q;
    wire [31:0] CsrWData_q = WrDataM_q; // for internal CSRs
    wire [11:0] CsrAddr_q = MCInsn_q[31:20]; // for internal CSRs

    assign retired    = RetiredM_q;
    assign csr_read   = CsrFromExtE_q;
    assign csr_write  = CsrWrite_q;
    assign csr_wdata  = WrDataM_q;
    assign csr_addr   = MCInsn_q[31:20];



    // internal CSRs

    wire CsrValid_w = csr_valid | (MCInsn_q[31:20]==12'h300);

    reg [31:0] CsrRDataInternal_d;
    always @* case (CsrAddr_q)
        12'h300: begin // MSTATUS
            CsrRDataInternal_d = {24'b0, f_MModePriorIntEnable, 3'b0,
                                         f_MModeIntEnable, 3'b0};
        end
        default: begin
            CsrRDataInternal_d = 0;
        end
    endcase


    // modify MSTATUS

    reg MModeIntEnable;
    reg MModePriorIntEnable;
    always @* begin
        MModeIntEnable = f_MModeIntEnable;
        MModePriorIntEnable = f_MModePriorIntEnable;
        if (CsrWrite_q & (CsrAddr_q==12'h300)) begin  // mstatus
            MModeIntEnable      = CsrWData_q[3];
            MModePriorIntEnable = CsrWData_q[7];
        end

        if (MCState_q[mcTrap1]) begin
            if (TrapReturn_q) begin 
                // return from trap (MRET)
                MModeIntEnable = f_MModePriorIntEnable;
                MModePriorIntEnable = 1;
            end else begin 
                // enter trap
                MModeIntEnable = 0;
                MModePriorIntEnable = f_MModeIntEnable;
            end
        end
    end





    // feed microcode values in M-stage into pipeline

    reg       CauseIntD_d;
    reg [3:0] CauseD_d;

    always @* begin
        CauseIntD_d = 0;
        CauseD_d = 0;
        OverPcD_d = 0;
        OverInsnD_d = 0;
        if (MCState_q[mcSysInsn]) begin
            case (MCInsn_q[31:20])
                12'h000: begin // ECALL
                    CauseD_d = 'h0B;
                end
                12'h001: begin // EBREAK
                    CauseD_d = 'h03;
                    OverPcD_d = 1;
                end
/*
                        12'h002: begin // URET
                        end
                        12'h102: begin // SRET
                        end
                        12'h302: begin // MRET
                        end
*/
                12'h105: begin // WFI
                    OverInsnD_d = 0;
                end
                default: begin
                    CauseD_d = 'h02;
                    OverInsnD_d = 1;
                end
            endcase
        end
        if (MCState_q[mcTrap0]) begin
            if (InsnCEBREAK_q) begin
                CauseD_d = 'h03;
                OverPcD_d = 1;
            end else if (TrapIllegal_q) begin
                CauseD_d = 'h02;
                OverInsnD_d = 1;
            //end else if (IrqResponse_q) begin
            end else begin
                CauseIntD_d = 1;
                //CauseD_d = ExternalIrq_q ? 'h0B :
                //           SoftwareIrq_q ? 'h03 : 'h07;
                CauseD_d = {ExternalIrq_q, ~ExternalIrq_q & ~SoftwareIrq_q, 2'b11};
            end
        end
    end

    wire OverPcE_d = OverPcE_q | MCState_q[mcTrap2];
    wire OverInsnE_d = OverInsnE_q;

    wire [WORD_WIDTH-1:0] OverwriteValE_d = 0
            | (MCState_q[mcTrap1] ? {CauseIntE_q, 27'b0, CauseE_q}   : 0)
            | (OverPcM_q   ? MCPrevPC_q                       : 0)
            | (OverInsnM_q ? MCInsn_q                         : 0)
            | (OverRs1_q   ? e_A                              : 0)
            | (OverImm5_q  ? {27'b0, MCInsn_q[19:15]}         : 0)
            //| (CsrFromExtE_q  ? (CsrRDataInternal_q | csr_rdata) : 0)
          ;

    wire [WORD_WIDTH-1:0] OverwrittenResult_w =
        (MCOverSelDivM_q ? MulDivResult_q                   : 0) |
        (CsrFromExtM_q   ? (CsrRDataInternal_q | csr_rdata) : 0) |
        (OverwriteM_q    ? OverwriteValM_q                  : 0) |

        ((MCOverSelDivM_q | CsrFromExtM_q | OverwriteM_q) ? 0 : WrDataM_q);






    // memory stage

    wire [1:0] AddrOfs = AddrSum_w[1:0];
    //wire [1:0] AddrOfs = {
    //    e_A[1] ^ Imm12PlusReg_q[1] ^ (e_A[0] & Imm12PlusReg_q[0]),
    //    e_A[0] ^ Imm12PlusReg_q[0]};

    reg MemMisalignedE_d;
    always @* case ({MemWidthE_q, AddrOfs})
        4'b0000: MemMisalignedE_d = 0;
        4'b0001: MemMisalignedE_d = 0;
        4'b0010: MemMisalignedE_d = 0;
        4'b0011: MemMisalignedE_d = 0;
        4'b0100: MemMisalignedE_d = 0;
        4'b0101: MemMisalignedE_d = 0;
        4'b0110: MemMisalignedE_d = 0;
        4'b0111: MemMisalignedE_d = 1; // lh +3
        4'b1000: MemMisalignedE_d = 0;
        4'b1001: MemMisalignedE_d = 1; // lw +1
        4'b1010: MemMisalignedE_d = 1; // lw +2
        4'b1011: MemMisalignedE_d = 1; // lw +3
        default: MemMisalignedE_d = 0;
    endcase

    wire MemSigned2_w =         // LH                LB
        ((MemWidthM_q[0] ? (AddrOfsM_q==3) : (AddrOfsM_q==0)) & mem_rdata[7]) |
        ((MemWidthM_q[0] ? (AddrOfsM_q==0) : (AddrOfsM_q==1)) & mem_rdata[15]) |
        ((MemWidthM_q[0] ? (AddrOfsM_q==1) : (AddrOfsM_q==2)) & mem_rdata[23]) |
        ((MemWidthM_q[0] ? (AddrOfsM_q==2) : (AddrOfsM_q==3)) & mem_rdata[31]);
    wire MemSigned_w = ~MCInsn_q[14] & MemSigned2_w;
        // signed or   ^^^^^^^^^^^^^ unsigned load









    wire [31:0] MemRShiftedBy16_w = AddrOfsM_q[1]
        ? {mem_rdata[15:0], mem_rdata[31:16]}
        :  mem_rdata;
    wire [31:0] MemRShifted_w = AddrOfsM_q[0]
        ? {MemRShiftedBy16_w[7:0], MemRShiftedBy16_w[31:8]}
        :  MemRShiftedBy16_w;

    reg [31:0] MemSignOrPrev_w;
    always @* begin
        MemSignOrPrev_w[31:24] = ((MemWidthM_q!=3) & MemSigned_w)
            ? 8'hFF
            : OverwrittenResult_w[31:24];
        MemSignOrPrev_w[23: 8] = MemWidthM_q[1]
            ? (MemWidthM_q[0]
                ? OverwrittenResult_w[23:8]
                : MemRDataPrev_q[23:8])
            : (MemSigned_w
                ? 16'hFFFF
                : 16'h0000);
        MemSignOrPrev_w[ 7: 0] = (MemWidthM_q==3)
            ? OverwrittenResult_w[7:0]
            : MemRDataPrev_q[7:0];
    end

    reg [3:0] SelCurMemByte_w;
    always @* case ({MemWidthM_q, AddrOfsM_q})
        4'b0000: SelCurMemByte_w = 4'b0001;
        4'b0001: SelCurMemByte_w = 4'b0001;
        4'b0010: SelCurMemByte_w = 4'b0001;
        4'b0011: SelCurMemByte_w = 4'b0001;
        4'b0100: SelCurMemByte_w = 4'b0011;
        4'b0101: SelCurMemByte_w = 4'b0011;
        4'b0110: SelCurMemByte_w = 4'b0011;
        4'b0111: SelCurMemByte_w = 4'b0010; // lh +3
        4'b1000: SelCurMemByte_w = 4'b1111;
        4'b1001: SelCurMemByte_w = 4'b1000; // lw +1
        4'b1010: SelCurMemByte_w = 4'b1100; // lw +2
        4'b1011: SelCurMemByte_w = 4'b1110; // lw +3
        default: SelCurMemByte_w = 0;
    endcase
    wire [31:0] WrDataM_d = {
        SelCurMemByte_w[3] ? MemRShifted_w[31:24] : MemSignOrPrev_w[31:24],
        SelCurMemByte_w[2] ? MemRShifted_w[23:16] : MemSignOrPrev_w[23:16],
        SelCurMemByte_w[1] ? MemRShifted_w[15: 8] : MemSignOrPrev_w[15: 8],
        SelCurMemByte_w[0] ? MemRShifted_w[ 7: 0] : MemSignOrPrev_w[ 7: 0]};



/*
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

    wire [31:0] WrDataM_d = {HiHalf, HiByte, LoByte};

wire [31:0] MemRShifted_w = 0;
*/











    reg [WORD_WIDTH-1:0] MemWData_d;
    always @* begin
        if (MemMisalignedM_q) begin
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
        if (MemMisalignedM_q) begin
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
            case ({MemWidthE_q, AddrOfs})
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
    assign mem_write = MemStore_q & ~m_Kill;
    assign mem_wmask = MemWMask_d;
    assign mem_wdata = MemWData_d;
    assign mem_waddr = AddrSum2_w;
    assign mem_addr  = MemAddr;



    // register set

    assign regset_we = WrEnM_d;
    assign regset_wa = WrNoM_q;
    assign regset_wd = WrDataM_d;
    assign regset_ra1 = RdNo1;
    assign regset_ra2 = RdNo2;






// ---------------------------------------------------------------------
// sequential logic
// ---------------------------------------------------------------------


    always @(posedge clk) begin

        // fetch
        Insn_q <= Insn_d;
        MCInsn_q <= (MC_q | ExcIllegal_q)
            ? MCInsn_q // keep insn during micorocode
            : (Insn_q[1:0]==2'b11) ? Insn_q 
                                   : {16'b0, Insn_q[15:0]};
                                     // clear upper half for 16 bit insn

        d_RdNo1 <= RdNo1;
        d_RdNo2 <= RdNo2;
        CsrTranslateE_q <= CsrTranslateD_d;
        MC_q <= MC_d;
        MCModRdNo_q <= MCModRdNo_d;
        MCAux_q <= MCAux_d;
        NextInsnHold_q <= NextInsnHold_d;

        OverPcM_q <= OverPcE_d;
        OverPcE_q <= OverPcD_d;
        OverInsnM_q <= OverInsnE_d;
        OverInsnE_q <= OverInsnD_d;
        OverRs1_q <= OverRs1_d;
        OverImm5_q <= OverImm5_d;
        TrapReturn_q <= TrapReturn_d;


        PartialInsn_q <= PartialInsn_d;
        OddPC_q <= OddPC_d;
        DelayedInsn_q <= DelayedInsn_d;
        MemAccessEorM_q <= MemAccessE_d | MemAccessD_d;
        MemAccessE_q <= MemAccessD_d;

        FetchAddr_q <= FetchAddr_d;
        MisalignedJumpE_q <= MisalignedJumpD_d;

        MulDivResNeg_q <= MulDivResNeg_d;
        MulDivResAdd_q <= MulDivResAdd_d;
        DivQuot_q <= DivQuot_d;
        DivRem_q <= DivRem_d;
        Long_q <= Long_d;
        MulDivResult_q <= MulDivResult_d;


        // decode
        PC_q <= PC_d;
        e_A <= ForwardAE;
        e_B <= ForwardBE;
        ImmUpper_q <= ImmUpper_w;
        Imm12PlusReg_q <= Imm12PlusReg_w;
        e_PCImm <= PCImm;

        WrEnE_q <= WrEnD_d;
        e_InsnJALR <= InsnJALR;
        e_InsnBEQ <= InsnBEQ;
        e_InsnBLTorBLTU <= InsnBLTorBLTU;
        e_InsnJAL <= InsnJAL;

        e_EnShift <= EnShift;
        ShiftRight_q <= ShiftRight_d;
        ShiftArith_q <= ShiftArith_d;
        ReturnPC_q <= ReturnPC_d;
        e_ReturnUI <= ReturnUI;

        e_SelSum <= SelSum;
        e_SetCond <= SetCond;
        e_LTU <= LTU;

        AddrFromSum_q <= AddrFromSum_d;
        MemStore_q <= MemStore_d;
        MemWidthE_q <= MemWidthD_d;

        LogicOp_q <= LogicOp_d;
        EnLogic_q <= EnLogic_d;
        e_NegB <= NegB;
        WrNoE_q <= WrNoD_d;
        InvertBranch_q <= InvertBranch_d;


        // execute
        WrEnM_q <= WrEnE_d;
        WrNoM_q <= WrNoE_q;
        WrDataM_q <= WrDataE_d;
        m_Kill <= Kill;

        MemWidthM_q         <= MemMisalignedM_q ? MemWidthM_q :
                                (m_Kill ? 2'b11 : MemWidthE_q);
        AddrOfsM_q          <= MemMisalignedM_q ? AddrOfsM_q : AddrSum_w[1:0];
        AddrSumAligned_q    <= AddrSum_w[WORD_WIDTH-1:2];
        MemRData_q          <= mem_rdata;
        MemWData_q          <= MemWData_d;
        MemMisalignedM_q     <= MemMisalignedE_d & ~m_Kill & ~MemMisalignedM_q;
        MemMisalignedW_q <= MemMisalignedM_q;
        MemRDataPrev_q      <= MemRShifted_w;

        // mem stage
        WrEnW_q <= WrEnM_d;
        WrNoW_q <= WrNoM_q;
        WrDataW_q <= WrDataM_d;

        RetiredM_q <= RetiredE_q & ~m_Kill;
        RetiredE_q <= RetiredD_d;

        // exception handling
        MCPrevPC_q          <= ((MC_q & ~MCState_q[mcWaitForInt]) | ExcIllegal_q)
            ? MCPrevPC_q // keep PC during micorocode
            : PC_q;
        OverwriteE_q        <= Overwrite_d;
        OverwriteM_q        <= OverwriteE_q;
        OverwriteValM_q     <= OverwriteValE_d;
        MCOverSelDivE_q     <= MCOverSelDivD_d;
        MCOverSelDivM_q     <= MCOverSelDivE_q;
        ExcIllegal_q    <= ExcIllegal_d;

        // csr
        CsrFromExtE_q       <= CsrFromExtD_d;
        CsrFromExtM_q       <= CsrFromExtE_q;
        CsrRDataInternal_q  <= CsrRDataInternal_d;
        CsrWrite_q          <= CsrWrite_d;

        // interrupt handling
        f_MModeIntEnable    <= MModeIntEnable;
        f_MModePriorIntEnable <= MModePriorIntEnable;
        IrqResponse_q       <= IrqResponse_d;
        SoftwareIrq_q       <= irq_software;
        TimerIrq_q          <= irq_timer;
        ExternalIrq_q       <= irq_external;

        MCState_q <= MCState_d;
        InsnCEBREAK_q <= InsnCEBREAK_d;
        CauseIntE_q <= CauseIntD_d;
        CauseE_q <= CauseD_d;
        TrapIllegal_q <= TrapIllegal_d;


`ifdef DEBUG
        $display("F write=%b wmask=%b wdata=%h addr=%h rdata=%h",
            mem_write, mem_wmask, mem_wdata, mem_addr, mem_rdata);
        if (MC_q==0) begin
            if (Insn_q[1:0]==2'b11) begin
                $display("D \033[1;33mpc=%h\033[0m insn=%h next=%h \033[1;30mMC=no\033[0m",
                    PC_q, Insn_q, Insn_d);
            end else begin
                $display("D \033[1;33mpc=%h\033[0m insn=\033[1;30m%h\033[0m%h next=%h \033[1;30mMC=no\033[0m",
                    PC_q, Insn_q[31:16], Insn_q[15:0], Insn_d);
            end
        end else begin
//            $display("D \033[1;33mpc=%h\033[1;30m insn=%h\033[0m next=%h \033[1;33mMCAux=%h\033[0m",
            $display("D \033[1;33mpc=%h\033[1;30m insn=%h\033[0m next=%h \033[1;33mMCState=%h\033[0m",
                PC_q, Insn_q, Insn_d, MCState_q);
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
            d_RdNo1, regset_rd1, d_RdNo2, regset_rd2, SelImm_w, ImmALU_w);
        $display("D MemAccessDEM=%b%b%b FetchAddr_q=%h Delayed=%h Partial=%h",
            MemAccessD_d, MemAccessE_q, MemAccessEorM_q, FetchAddr_q, DelayedInsn_q, PartialInsn_q);

        $display("E a=%h b=%h -> %h -> x%d wrenDEM=%b%b%b",
            e_A, e_B, WrDataE_d, WrNoE_q, WrEnD_d, WrEnE_q, WrEnM_q);
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
        $display("  e_InsnBLTorBLTU=%b vLess=%b e_InsnJAL=%b ReturnPC=%b",
            e_InsnBLTorBLTU, vLess, e_InsnJAL, ReturnPC);
        $display("  e_InsnBEQ=%b InvertBranch_q=%b vEqual=%b",
            e_InsnBEQ, InvertBranch_q, vEqual);
*/
/*
        $display("F AE=%b AM=%b AW=%b AR=%h AM=%h AE=%h",
            FwdAE, FwdAM, FwdAW, ForwardAR, ForwardAM, ForwardAE);
        $display("F BE=%b BM=%b BW=%b BR=%h BM=%h BE=%h SelImm_w=%b",
            FwdBE, FwdBM, FwdBW, ForwardBR, ForwardBM, ForwardBE, SelImm_w);
*/

        $display("C MTVEC=%h MSCRATCH=%h MEPC=%h MCAUSE=%h MTVAL=%h",
            RegSet.regs[REG_CSR_MTVEC],
            RegSet.regs[REG_CSR_MSCRATCH],
            RegSet.regs[REG_CSR_MEPC],
            RegSet.regs[REG_CSR_MCAUSE],
            RegSet.regs[REG_CSR_MTVAL]);

//        $display("C TMP=%h TMP2=%h",
//            RegSet.regs[REG_CSR_TMP],
//            RegSet.regs[REG_CSR_TMP2]);


        $display("C CsrTranslateD_d=%h",
            CsrTranslateD_d);
        $display("C CsrWrite_q=%b CsrAddr_q=%h CsrWData_q=%h",
            CsrWrite_q, CsrAddr_q, CsrWData_q);
/*
        $display("C CsrResult_w=%h OverwrittenResult_w=%h",
            CsrResult_w, OverwrittenResult_w);
*/
        $display("C CsrFromExtM_q=%b CsrRDataInternal_q=%h ExcInvalidInsn_d=%b",
            CsrFromExtM_q, CsrRDataInternal_q, ExcIllegal_d);
        $display("C CSR addr=%h rdata=%h valid=%b",
            csr_addr, csr_rdata, csr_valid);




        $display("M MemWidthE_q=%b AddrOfs=%b MemAddr=%h MemMisalignedW_q=%b",
            MemWidthE_q, AddrOfs, MemAddr, MemMisalignedW_q);
        $display("M MemWidthM_q=%b AddrOfsM_q=%b MemMisalignedM_q=%b",
            MemWidthM_q, AddrOfsM_q, MemMisalignedM_q);
        $display("M SelCurMemByte_w=%b, MemRShifted_w=%h, MemSignOrPrev_w=%h MemSigned_w=%b",
            SelCurMemByte_w, MemRShifted_w, MemSignOrPrev_w, MemSigned_w);

/*
        $display("  DestReg0Part=%b DisableWrite=%b EnableWrite2=%b WrEnEMW=%b%b%b",
            DestReg0Part, DisableWrite, EnableWrite2, WrEnD_d, WrEnE_d, WrEnM_d);
            OverInsnM_q,
            TrapEnterIrq_q,
            OverPcE_q,
            TrapEnterECALL_q,
            OverInsnE_q,
            OverRs1_q,
            OverImm5_q);
*/
        $display("C OverwriteValM_q=%h OverwrittenResult_w=%h OverwriteDEM=%b%b%b",
            OverwriteValM_q, OverwrittenResult_w,
            Overwrite_d, OverwriteE_q, OverwriteM_q);
        $display("C OverPcM_q=%b OverInsnDEM_q=%b%b%b CauseE_q=%b:%h",
            OverPcM_q, OverInsnD_d, OverInsnE_d, OverInsnM_q,
            CauseIntE_q, CauseE_q);
        $display("T InsnCEBREAK_q=%b TrapIllegal_q=%b",
            InsnCEBREAK_q, TrapIllegal_q);



        $display("I MIE=%b MPIE=%b Software=%b Timer=%b External=%b IRdq=%b%b",
            f_MModeIntEnable, f_MModePriorIntEnable, 
            SoftwareIrq_q,
            TimerIrq_q,
            ExternalIrq_q,
            IrqResponse_d, IrqResponse_q);

        if (WrEnM_q) $display("M x%d<-%h", WrNoM_q, WrDataM_q);
        if (WrEnW_q) $display("W x%d<-%h",WrNoW_q, WrDataW_q);
`endif

        if (!rstn) begin
            WrEnE_q <= 0;
            AddrFromSum_q <= 0;
            MemStore_q <= 0;
            MemWidthE_q <= 2'b11;
            FetchAddr_q <= 32'hf0000000;

            Insn_q <= 32'h13;
            OddPC_q <= 0;
            MC_q <= 0;
            MCState_q <= 0;

            MCAux_q <= 0;
            MemAccessEorM_q <= 0;
            MemAccessE_q <= 0;
            DelayedInsn_q <= 'h13;

            // fake a jump to address 0 on reset
            m_Kill <= 0;
            e_PCImm <= START_PC;
            e_InsnJAL  <= 1;

            f_MModeIntEnable <= 0;
            f_MModePriorIntEnable <= 0;
            IrqResponse_q <= 0;
        end
    end



`ifdef RISCV_FORMAL
    reg [31:0]           RvfiInsnM_q;
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
    reg [WORD_WIDTH-1:0] RvfiRdData1E_q;
    reg [WORD_WIDTH-1:0] RvfiRdData1M_q;
    reg [WORD_WIDTH-1:0] RvfiRdData2E_q;
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
    reg                  RvfiValidCsrD_q;
    reg                  RvfiValidCsrE_q;
    reg                  RvfiIllegal_q;

    wire [3:0] RvfiMemMask =
        MemWidthE_q[1] ? (MemWidthE_q[0] ? 4'b0000 : 4'b1111)
                       : (MemWidthE_q[0] ? 4'b0011 : 4'b0001);

    wire RvfiRetire_w = Rvfi_Trap_w
                      | RetiredM_q
                      | RvfiValidCsrE_q
                      | MemMisalignedW_q;

    wire Rvfi_Trap_w = MCState_q[mcWaitForInt] | MCState_q[mcTrap1] 
        | RvfiIllegal_q;
            // illegal trap must be one cycle eralier than other traps
            // to be synchronous with the illegal instruction


    always @(posedge clk) begin
        rvfi_valid      <= rstn & !rvfi_halt & (RvfiRetire_w | Rvfi_Trap_w);
        rvfi_order      <= rstn ? rvfi_order + rvfi_valid : 0;
        rvfi_trap       <= Rvfi_Trap_w;
        rvfi_halt       <= Rvfi_Trap_w | rvfi_halt;

        rvfi_intr       <= RvfiIntrM_q;
        rvfi_mode       <= 3;
        rvfi_ixl        <= 1;

        rvfi_insn       <= RvfiInsnM_q;
        rvfi_rs1_addr   <= Rvfi_Trap_w ? 5'b0 : RvfiRdNo1M_q;
        rvfi_rs2_addr   <= Rvfi_Trap_w ? 5'b0 : RvfiRdNo2M_q;
        rvfi_rs1_rdata  <= Rvfi_Trap_w ? 0 : RvfiRdData1M_q;
        rvfi_rs2_rdata  <= Rvfi_Trap_w ? 0 : RvfiRdData2M_q;
        rvfi_pc_rdata   <= RvfiPcM_q;
        rvfi_pc_wdata   <= RvfiExcRetM_q ? (ForwardAE&~1) : RvfiNextPcM_q;

        rvfi_mem_addr   <= RvfiMemAddrM_q;
        rvfi_mem_rmask  <= RvfiMemRMaskM_q;
        rvfi_mem_wmask  <= RvfiMemWMaskM_q;
        rvfi_mem_wdata  <= RvfiMemWDataM_q;

        rvfi_rd_addr    <= (WrEnM_d & ~WrNoM_q[5]) ? WrNoM_q : 0;
        rvfi_rd_wdata   <= RvfiAltResultM_q         ? RvfiResultM_q :
                           (WrEnM_d & ~WrNoM_q[5])  ? WrDataM_d : 0;
        rvfi_mem_rdata  <= WrDataM_d;

        RvfiInsnM_q     <= MCInsn_q;
        RvfiIntrM_q     <= RvfiIntrE_q; // long delay (jump to MTVEC)
        RvfiIntrE_q     <= RvfiIntrD_q;
        RvfiIntrD_q     <= RvfiIntrF_q;
        RvfiIntrF_q     <= RvfiIntr1_q;
        RvfiIntr1_q     <= RvfiIntr0_q;
        RvfiIntr0_q     <= MCState_q[mcTrap1] & ~TrapReturn_q;

        RvfiRdNo1M_q    <= RvfiRdNo1E_q;
        RvfiRdNo1E_q    <= d_RdNo1[5] ? 5'b0 : d_RdNo1; // no CSRs
        RvfiRdNo2M_q    <= RvfiRdNo2E_q;
        RvfiRdNo2E_q    <= (SelImm_w | d_RdNo2[5]) ? 5'b0 : d_RdNo2; // no CSRs
        RvfiRdData1M_q  <= e_A;
        RvfiRdData2M_q  <= (RvfiRdNo2E_q==0) ? 5'b0 : (e_B ^ {WORD_WIDTH{e_NegB}});

        RvfiPcM_q       <= RvfiPcE_q;
        RvfiPcE_q       <= MC_q ? MCPrevPC_q /*RvfiPcE_q*/ : PC_q;
        RvfiNextPcM_q   <= Kill
            ? {FetchAddr_d[WORD_WIDTH-1:2], MisalignedJumpD_d, 1'b0}
            : PC_q;
        RvfiExcRetM_q   <= MCState_q[mcTrap1];

        RvfiMemStoreM_q <= MemStore_q;
        RvfiMemAddrM_q  <= AddrSum_w;
        RvfiMemRMaskM_q <= MemStore_q ? 0 : RvfiMemMask;
        RvfiMemWMaskM_q <= mem_write ? RvfiMemMask : 0;
        RvfiMemWDataM_q <= e_B & {{8{RvfiMemMask[3]}}, {8{RvfiMemMask[2]}},
                                  {8{RvfiMemMask[1]}}, {8{RvfiMemMask[0]}}};

        RvfiValidCsrD_q <= MCState_q[mcCsr4];
        RvfiValidCsrE_q <= RvfiValidCsrD_q;
        RvfiIllegal_q <= ExcIllegal_q & ~m_Kill;

        // hold values for next cycle
        if (MemMisalignedM_q /*| MCState_q[mcCsr2]*/) begin
            rvfi_valid <= !rvfi_halt & Rvfi_Trap_w; // avoid loosing a trap

            RvfiInsnM_q <= RvfiInsnM_q;
            RvfiRdNo1M_q <= RvfiRdNo1M_q;
            RvfiRdNo2M_q <= RvfiRdNo2M_q;
            RvfiRdData1M_q <= RvfiRdData1M_q;
            RvfiRdData2M_q <= RvfiRdData2M_q;
            RvfiPcM_q      <= RvfiPcM_q;
            RvfiNextPcM_q  <= RvfiNextPcM_q;

            RvfiMemAddrM_q   <= RvfiMemAddrM_q;
            RvfiMemRMaskM_q  <= RvfiMemRMaskM_q;
            RvfiMemWMaskM_q  <= RvfiMemWMaskM_q;
            RvfiMemWDataM_q  <= RvfiMemWDataM_q;
        end
        if (MCState_q[mcCsr2]) begin
            // clear retire signal of 1st cycle
            rvfi_valid <= !rvfi_halt & Rvfi_Trap_w; // avoid loosing a trap
        end

        if (RvfiValidCsrE_q) begin
            rvfi_rs1_addr   <= 0;
            rvfi_rs2_addr   <= 0;
            rvfi_rs1_rdata  <= 0;
            rvfi_rs2_rdata  <= 0;
        end




`ifdef RISCV_FORMAL_ALTOPS
        RvfiAltResultM_q <= 0;
        RvfiResultM_q <= 0;
        if (MCState_q[mcMulDiv1]) begin
            RvfiAltResultM_q <= WrEnE_d;
            if (~SelDivOrMul_w) begin
                if (SelMulLowOrHigh_w) begin // MUL
                    RvfiResultM_q <= (e_A + e_B) ^ 32'h5876063e;
                end else if (InsnMULH_w) begin // MULH
                    RvfiResultM_q <= (e_A + e_B) ^ 32'hf6583fb7;
                end else if (MulASigned_w) begin // MULHSU
                    RvfiResultM_q <= (e_A - e_B) ^ 32'hecfbe137;
                end else begin // MULHU
                    RvfiResultM_q <= (e_A + e_B) ^ 32'h949ce5e8;
                end
            end else begin
                if (~SelRemOrDiv_w) begin
                    if (DivSigned_w) begin // DIV
                        RvfiResultM_q <= (e_A - e_B) ^ 32'h7f8529ec;
                    end else begin // DIVU
                        RvfiResultM_q <= (e_A - e_B) ^ 32'h10e8fd70;
                    end
                end else begin
                    if (DivSigned_w) begin // REM
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
