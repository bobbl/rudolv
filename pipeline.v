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
    reg [5:0] d_RdNo1;
    reg [5:0] d_RdNo2;

    reg MC_q;
    reg [7:0] MCState_q;
    reg [5:0] MCAux_q;
    reg [4:0] MCRegNo_q;
    reg [4:0] d_CsrTranslate;

    reg [WORD_WIDTH-1:0] PC_q;
    reg [31:0] DelayedInsn_q;
    reg DelayedInsnGrubby_q;
    reg BubbleM_q;
    reg BubbleE_q;

    reg [31:0] PartialInsn_q;
    reg OddPC_q;
    reg RealignedPC_q;

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
    reg e_AGrubby;
    reg e_BGrubby;

    reg e_NegB;
    reg e_WrEn;
    reg [5:0] e_WrNo;

    reg  e_InsnBit14;
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

    reg RetiredE_q;
    reg RetiredM_q;

    // exceptions
    reg [WORD_WIDTH-1:0] e_ExcWrData2;
    reg [WORD_WIDTH-1:0] m_ExcWrData;
    reg e_ExcGrubbyInsn;
    reg m_ExcGrubbyInsn;
    reg f_ExcGrubbyJump;
    reg d_ExcGrubbyJump;
    reg e_ExcGrubbyJump;
    reg m_ExcMem;
    reg w_ExcMem;
    reg m_MemStore;
    reg m_WriteMCAUSE;
    reg m_WriteMTVAL;
    reg [4:0] m_Cause;

    reg OverwriteE_q;
    reg [WORD_WIDTH-1:0] OverwriteValE_q;
    reg OverwriteM_q;
    reg [WORD_WIDTH-1:0] OverwriteValM_q;
    reg OverwriteSelE_q;
    reg InsnMRET_E_q;
    reg InsnMRET_M_q;

    // CSRs for exceptions
    reg [WORD_WIDTH-1:0] m_CsrUpdate;
    reg [1:0] e_CsrOp;
    reg [1:0] m_CsrOp;

    reg e_CsrFromReg;
    reg m_CsrFromReg;

    reg w_CsrFromReg;
    reg [WORD_WIDTH-1:0] m_CsrModified;



    reg        e_CsrFromExt;
    reg        e_CsrRead;
    reg        m_CsrRead;
    reg        m_CsrValid;
    reg [WORD_WIDTH-1:0] m_CsrRdData;



    reg f_MModeIntEnable;        // mstatus.mie
    reg f_MModePriorIntEnable;   // mstatus.mpie
    reg IrqResponse_q;
    reg NoIrq_q;
    reg SoftwareIrq_q;
    reg TimerIrq_q;      // external pin high
    reg ExternalIrq_q;

    reg d_GrubbyInsn;


// deprecated
    wire [WORD_WIDTH-1:0] f_PC = FetchAddr_q;
    wire [WORD_WIDTH-1:0] d_PC = PC_q;


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
                    MModeIntEnable      = f_MModeIntEnable      & ~e_CsrWData[3];
                    MModePriorIntEnable = f_MModePriorIntEnable & ~e_CsrWData[7];
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
    assign csr_read   = e_CsrRead;
    assign csr_modify = e_CsrModify;
    assign csr_wdata  = e_CsrWData;
    assign csr_addr   = Insn_q[31:20];


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


    // Fetching
    reg [31:0] Insn_d;
    reg DecodeGrubbyInsn;
    reg [5:0] RdNo1;
    reg [5:0] RdNo2;
    reg [31:0] DelayedInsn_d;
    reg DelayedInsnGrubby_d;

    reg [WORD_WIDTH-1:0] FetchAddr_d;
    reg [WORD_WIDTH-1:0] PC_d;
    reg [31:0] PartialInsn_d;
    reg OddPC_d;
    reg RealignedPC_d;
    reg RVCInsn_w;

    // Decoding
    reg ExcInvalidInsn;
    reg RetiredD_d;
    reg DecodeWrEn;
    reg [5:0] DecodeWrNo;

    reg InsnCSR;
    reg CsrRead;
    reg CsrFromExt; // only to avoid write enable in second cycle of CSR Insn
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
    reg Overwrite_d;              // overwrite result in M-stage (used by exceptions
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

        InsnCSR = 0;
        CsrRead = 0;
        CsrFromExt = 0;
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
            if (f_ExcGrubbyJump) begin
                RdNo1          = REG_CSR_MTVEC;
                MC_d           = 1;
                MCState_d      = mcJumpReg;
                TrapEnter_d    = 1;
            end

        end else if (d_GrubbyInsn) begin
            RdNo1          = REG_CSR_MTVEC;
            MC_d           = 1;
            MCState_d      = mcJumpReg;
            TrapEnter_d    = 1;

        // microcoded cycles of multi-cycle instructions
        end else if (MC_q) begin
            ExcInvalidInsn = 0;
            case (MCState_q)
                mcNop: begin
                end
                mcCsr: begin // 2nd cycle of CSR access
                    DecodeWrEn     = e_CsrFromReg;
                    DecodeWrNo     = {1'b1, d_CsrTranslate};
                end
                mcMem: begin // 2nd cycle of memory access
                    if (MemMisaligned) begin
                        InsnJALR = 1;
                        AddrFromSum = 1;
                        Imm12PlusReg_d = 0;
                    end
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
                        OverwriteVal_d = InsnE_q;
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
            DecodeWrNo = {e_CsrFromReg, Insn_q[11:7]};

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

                    // disable JALR, if it is the memory bubble and there is no memory exception
                    // TODO: get rid of it
                    if ((e_MemWidth==2'b11) | MemMisaligned) begin // no mem access or misaligned
                        InsnJALR    = 1;
                        AddrFromSum = 1;
                    end
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
                    RdNo1       = REG_CSR_MTVEC;
                        // read MTVEC in 2nd cycle, so that its value is
                        // available if Exc is raised
                    MC_d           = 1;
                    MCState_d      = mcMem;
                end
                5'b01000: begin // store
                    ExcInvalidInsn = (Insn_q[14] | (Insn_q[13] & Insn_q[12]));
                    DecodeWrEn     = 0;
                    BubbleD_d         = 1;
                    AddrFromSum    = 1;
                    MemStore       = 1;
                    MemWidth       = Insn_q[13:12];
                    RdNo1       = REG_CSR_MTVEC;
                    MC_d           = 1;
                    MCState_d      = mcMem;
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
                    if (Insn_q[13] | Insn_q[12]) begin
                        // RVZicsr
                        ExcInvalidInsn =
                            Insn_q[31] & Insn_q[30] & (~Insn_q[13] | (Insn_q[19:15]!=0));
                                // readonly CSR && (CSRRW || rs1!=0)
                        CsrRead    = DecodeWrEn & ~CsrFromReg;
                        DecodeWrEn = DecodeWrEn & CsrFromReg;
                        InsnCSR    = 1;
                        CsrFromExt = ~CsrFromReg;
                        CsrOp      = Insn_q[13:12];
                        RdNo1   = {1'b1, vCsrTranslate};
                        MC_d       = 1;
                        MCState_d  = mcCsr;
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
                    RdNo1 = REG_CSR_MTVEC;
                        // read MTVEC in 2nd cycle, so that its value is
                        // available if Exc is raised
                    DecodeWrEn = 1;
                    DecodeWrNo = {3'b001, Insn_q[4:2]};
                    MC_d = 1;
                    MCState_d = mcMem;
                end
                4'b1100: begin // C.SW
                    ExcInvalidInsn = Insn_q[13];
                    DecodeWrEn = 0;
                    BubbleD_d = 1;
                    AddrFromSum = 1;
                    MemStore = 1;
                    MemWidth = 2'b10;
                    RdNo1 = REG_CSR_MTVEC;
                    MC_d = 1;
                    MCState_d = mcMem;
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
                    RdNo1 = REG_CSR_MTVEC;
                        // read MTVEC in 2nd cycle, so that its value is
                        // available if Exc is raised
                    MC_d = 1;
                    MCState_d = mcMem;
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

                                // disable JALR, if it is the memory bubble and there is no memory exception
                                // TODO: get rid of it
                                if ((e_MemWidth==2'b11) | MemMisaligned) begin // no mem access or misaligned
                                    InsnJALR = 1;
                                    AddrFromSum = 1;
                                end
                            end
                        end else begin // C.JR
                            ExcInvalidInsn = (Insn_q[11:7]==0);
                            DecodeWrEn = 0;
                            NoIrq_d = 1;

                            // disable JALR, if it is the memory bubble and there is no memory exception
                            // TODO: get rid of it
                            if ((e_MemWidth==2'b11) | MemMisaligned) begin // no mem access or misaligned
                                InsnJALR    = 1;
                                AddrFromSum = 1;
                            end
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
                    RdNo1 = REG_CSR_MTVEC;
                    MC_d = 1;
                    MCState_d = mcMem;
                end
                default: begin
                    ExcInvalidInsn = 1;
                end
            endcase
            RetiredD_d = ~ExcInvalidInsn;
        end

        // OPTIMISE
        if (ExcInvalidInsn) begin
//        (m_CsrRead & ~m_Kill & ~w_Kill & ~csr_valid)) begin
            Overwrite_d = 1;
            OverwriteVal_d = PC_q;
            DecodeWrNo = REG_CSR_MEPC;
            RdNo1 = REG_CSR_MTVEC;
            MC_d = 1;
            MCState_d = mcException;
            MCAux_d = 6'h02 + 1;
            TrapEnter_d = 1;
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
            DecodeGrubbyInsn = DelayedInsnGrubby_q;
            if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            DelayedInsnGrubby_d = DelayedInsnGrubby_q;
            if (RealignedPC_d) begin
                DelayedInsn_d       = PartialInsn_q;
            end else begin
                DelayedInsn_d       = DelayedInsn_q;
            end
            PartialInsn_d       = DelayedInsn_q;

        // 010 POP and PUSH
        end else if (BubbleE_q & ~m_Kill) begin
            DecodeGrubbyInsn = DelayedInsnGrubby_q;
            if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            DelayedInsnGrubby_d = mem_rgrubby;
            DelayedInsn_d       = mem_rdata;
            if (RealignedPC_d) begin
                PartialInsn_d       = PartialInsn_q;
            end else begin
                PartialInsn_d       = DelayedInsn_q;
            end

        // 000 MC
        end else if (MC_q & (MCState_q!=mcUnalignedJump) & ~m_Kill) begin
            DecodeGrubbyInsn = DelayedInsnGrubby_q;
            if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            DelayedInsnGrubby_d = DelayedInsnGrubby_q;
            DelayedInsn_d       = DelayedInsn_q;
            if (RealignedPC_d) begin
                PartialInsn_d   = PartialInsn_q;
            end else begin
                PartialInsn_d   = DelayedInsn_q;
            end

        // 100 PUSH
        end else if ((BubbleD_d & ~BubbleM_q) | StartMulDiv_d) begin
            DecodeGrubbyInsn = mem_rgrubby;
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {mem_rdata[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = mem_rdata;
            end
            DelayedInsnGrubby_d = mem_rgrubby;
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
        end else if ((~BubbleD_d & BubbleM_q) & ~m_Kill) begin
            DecodeGrubbyInsn = DelayedInsnGrubby_q;
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {DelayedInsn_q[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = DelayedInsn_q;
            end
            DelayedInsnGrubby_d = DelayedInsnGrubby_q;
            DelayedInsn_d       = DelayedInsn_q;
            if (RealignedPC_d) begin
                PartialInsn_d       = PartialInsn_q;
            end else begin
                PartialInsn_d       = DelayedInsn_q;
            end


        // 000 NOT DELAYED
        end else begin
            DecodeGrubbyInsn = mem_rgrubby;
            if (RealignedPC_d) begin
                Insn_d = PartialInsn_q;
            end else if (OddPC_d) begin
                Insn_d = {mem_rdata[15:0], PartialInsn_q[31:16]};
            end else begin
                Insn_d = mem_rdata;
            end
            DelayedInsnGrubby_d = DelayedInsnGrubby_q;
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







    wire ExecuteWrEn   = ~m_Kill & (e_CsrRead | e_WrEn);
    wire [5:0] ExecuteWrNo = e_WrNo;






    // forwarding

    // 6*(4 + 32) LC = 216 LC
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

    wire ALUResultGrubby = 0;
    wire ForwardAWGrubby =  FwdAW ? w_WrGrubby : regset_rg1;
    wire ForwardAMGrubby =  FwdAM ? MemResultGrubby : ForwardAWGrubby;
    wire ForwardAEGrubby = (FwdAE ? ALUResultGrubby : ForwardAMGrubby)
        & ~d_RdNo1[5];
    wire ForwardBRGrubby = ~vSelImm & (FwdBW ? w_WrGrubby : regset_rg2);
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

    wire [WORD_WIDTH-1:0] NextFetchAddr_w = FetchAddr_q + 4;

    wire [WORD_WIDTH-1:0] MemAddr =
        Branch_w // taken PC-relative branch
            ? {e_PCImm[WORD_WIDTH-1:2], 2'b00}
            : (e_AddrFromSum & ~m_Kill)
                ? {AddrSum_w[WORD_WIDTH-1:1], 1'b0}
                : RealignedPC_d
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




    // exception handling

    wire ExcGrubbyInsn  = d_GrubbyInsn;
    wire ExcGrubbyJump  = e_InsnJALR & e_AGrubby & ~m_Kill;

    wire vWriteMEPC     = f_ExcGrubbyJump | m_ExcGrubbyInsn | m_ExcMem;
    wire WriteMCAUSE    = f_ExcGrubbyJump | m_ExcGrubbyInsn | w_ExcMem;
    wire WriteMTVAL     = d_ExcGrubbyJump | m_ExcMem;

    wire vExcOverwrite  = vWriteMEPC | m_WriteMTVAL | m_WriteMCAUSE;
    wire MemWrEn        = m_WrEn | vExcOverwrite | OverwriteM_q;

    wire [5:0] MemWrNo  =
        vWriteMEPC      ? REG_CSR_MEPC :
        (m_WriteMTVAL   ? REG_CSR_MTVAL :
        (m_WriteMCAUSE  ? REG_CSR_MCAUSE :
        m_WrNo));


    wire [4:0] Cause = m_ExcMem        ? (m_MemStore ? 5'h06 : 5'h04) :
                      (e_ExcGrubbyInsn ? 5'h0e :
                      (ExcGrubbyJump   ? 5'h0a :
                      /* e_ExcJump */    5'h00 )); // -> m_Cause


    wire [WORD_WIDTH-1:0] ExcWrData2 =
        MemMisaligned   ? AddrSum_w     // MTVAL for mem access
                        : PC_q;         // MEPC 


    wire [WORD_WIDTH-1:0] ExcWrData =
        WriteMCAUSE     ? {m_Cause[4], {(WORD_WIDTH-5){1'b0}}, m_Cause[3:0]}  // MCAUSE
                        : e_ExcWrData2;


    wire [WORD_WIDTH-1:0] OverwriteValE_d = OverwriteSelE_q ? MulDivResult_d : OverwriteValE_q;

    wire [WORD_WIDTH-1:0] CsrResult =
        OverwriteM_q ? OverwriteValM_q
                     : (vExcOverwrite ? (e_ExcGrubbyJump ? e_ExcWrData2 // MTVAL for jump
                                                         : m_ExcWrData)
                                      : vCsrOrALU);

    // CSRs
/*
    reg [5:0] vCsrTranslate;
    reg CsrFromReg;
    always @* begin
        CsrFromReg <= InsnCSR;
        case (Insn_q[31:20])
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
        vCsrTranslate <= {1'b1, Insn_q[26], Insn_q[23:20]};
    end
*/
    wire [4:0] vCsrTranslate = {Insn_q[26], Insn_q[23:20]};
    wire CsrFromReg = (InsnCSR & ~m_Kill) && (Insn_q[31:27]==5'b00110) && 
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

    wire [WORD_WIDTH-1:0] vCsrOrALU =
        (m_CsrFromReg             ? e_A           : 0) |
        (w_CsrFromReg             ? m_CsrModified : 0) |
        ((m_CsrRead & m_CsrValid) ? m_CsrRdData   : 0) |
        ((m_CsrRead & csr_valid)  ? csr_rdata     : 0) |
        ((m_CsrFromReg | w_CsrFromReg | m_CsrRead)
            ? 0 : m_WrData);







    // memory signals, generated in execute stage

    wire [1:0] AddrOfs = AddrSum_w[1:0];
    //wire [1:0] AddrOfs = {
    //    e_A[1] ^ Imm12PlusReg_q[1] ^ (e_A[0] & Imm12PlusReg_q[0]),
    //    e_A[0] ^ Imm12PlusReg_q[0]};

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
    wire [4:0] MemByte = m_Kill ? 5'b0 : MemSignals[11:7];
    wire [2:0] MemSign = (m_Kill | e_MemUnsignedLoad) ? 3'b0 : MemSignals[6:4];



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

    assign mem_valid = 1;
    assign mem_write = e_MemStore & ~m_Kill;
    assign mem_wmask = MemSignals[3:0];
    assign mem_wdata = MemWriteData;
    assign mem_wgrubby = (e_MemWidth!=2'b10) | e_AGrubby | e_BGrubby;
    assign mem_addr  = MemAddr;





    // register set

    assign regset_we = MemWrEn;
    assign regset_wa = MemWrNo;
    assign regset_wd = MemResult;
    assign regset_wg = MemResultGrubby;
    assign regset_ra1 = RdNo1;
    assign regset_ra2 = RdNo2;






// ---------------------------------------------------------------------
// sequential logic
// ---------------------------------------------------------------------


    always @(posedge clk) begin

        // fetch
        Insn_q <= Insn_d;
        InsnE_q <= Insn_q; // only for illegal exception
        d_RdNo1 <= RdNo1;
        d_RdNo2 <= RdNo2;
        d_CsrTranslate <= vCsrTranslate;
        MC_q <= MC_d;
        MCState_q <= MCState_d;
        MCAux_q <= MCAux_d;
        MCRegNo_q <= MCRegNo_d;

        PartialInsn_q <= PartialInsn_d;
        OddPC_q <= OddPC_d;
        RealignedPC_q <= RealignedPC_d;
        DelayedInsn_q <= DelayedInsn_d;
        DelayedInsnGrubby_q <= DelayedInsnGrubby_d;
        BubbleM_q <= BubbleE_d;
        BubbleE_q <= BubbleD_d;

        FetchAddr_q <= FetchAddr_d;




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
        e_AGrubby <= ForwardAEGrubby;
        e_BGrubby <= ForwardBEGrubby;
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
        w_Kill <= m_Kill; 

        e_InsnBit14 <= Insn_q[14];

        // execute
        m_WrEn <= ExecuteWrEn;
        m_WrNo <= ExecuteWrNo;
        m_WrData <= ALUResult;
        m_Kill <= Kill;
        m_MemSign <= MemSign;
        m_MemByte <= MemByte;

        // mem stage
        w_WrEn <= MemWrEn;
        w_WrNo <= MemWrNo;
        w_WrData <= MemResult;
        w_WrGrubby <= MemResultGrubby;

        RetiredM_q <= RetiredE_q & ~m_Kill;
        RetiredE_q <= RetiredD_d;

        // exception handling
        e_ExcWrData2        <= ExcWrData2;
        m_ExcWrData         <= ExcWrData; 
        e_ExcGrubbyInsn     <= ExcGrubbyInsn;
        m_ExcGrubbyInsn     <= (e_ExcGrubbyInsn & ~m_Kill);
        f_ExcGrubbyJump     <= ExcGrubbyJump;
        d_ExcGrubbyJump     <= f_ExcGrubbyJump;
        e_ExcGrubbyJump     <= d_ExcGrubbyJump;
        m_ExcMem            <= MemMisaligned & ~m_Kill;
        w_ExcMem            <= m_ExcMem & ~m_Kill;
        m_MemStore          <= e_MemStore;
        m_WriteMCAUSE       <= WriteMCAUSE;
        m_WriteMTVAL        <= WriteMTVAL;
        m_Cause             <= Cause;

        OverwriteE_q        <= Overwrite_d;
        OverwriteValE_q     <= OverwriteVal_d;
        OverwriteM_q        <= OverwriteE_q;
        OverwriteValM_q     <= OverwriteValE_d;
        OverwriteSelE_q     <= OverwriteSelD_d;
        InsnMRET_E_q        <= InsnMRET_d;
        InsnMRET_M_q        <= InsnMRET_E_q;

        // csr
        m_CsrUpdate         <= CsrUpdate;
        e_CsrOp             <= CsrOp;
        m_CsrOp             <= e_CsrOp;
        e_CsrFromReg        <= CsrFromReg;
        m_CsrFromReg        <= e_CsrFromReg;

        w_CsrFromReg        <= m_CsrFromReg;
        m_CsrModified       <= CsrModified;

        // interrupt handling
        f_MModeIntEnable    <= MModeIntEnable;
        f_MModePriorIntEnable <= MModePriorIntEnable;
        IrqResponse_q       <= IrqResponse_d;
        NoIrq_q             <= NoIrq_d;
        SoftwareIrq_q       <= irq_software;
        TimerIrq_q          <= irq_timer;
        ExternalIrq_q       <= irq_external;

        // csr
        e_CsrFromExt        <= CsrFromExt;
        e_CsrRead           <= CsrRead;
        m_CsrRead           <= e_CsrRead;
        m_CsrValid          <= CsrValidInternal;
        m_CsrRdData         <= CsrRDataInternal;

        d_GrubbyInsn <= DecodeGrubbyInsn;





`ifdef DEBUG
        $display("F write=%b wmask=%b wdata=%h wgrubby=%b addr=%h rdata=%h rgrubby=%b",
            mem_write, mem_wmask, mem_wdata, mem_wgrubby, mem_addr, mem_rdata, mem_rgrubby);
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

/*
        $display("R grubby %b%b%b%b %b%b%b%b %b%b%b%b %b%b%b%b",
            RegSet.grubby[0], RegSet.grubby[1], RegSet.grubby[2], RegSet.grubby[3],
            RegSet.grubby[4], RegSet.grubby[5], RegSet.grubby[6], RegSet.grubby[7],
            RegSet.grubby[8], RegSet.grubby[9], RegSet.grubby[10], RegSet.grubby[11],
            RegSet.grubby[12], RegSet.grubby[13], RegSet.grubby[14], RegSet.grubby[15]);
*/

        $display("R grubby %b%b%b%b %b%b%b%b %b%b%b%b %b%b%b%b",
            RegSet.regs[0][32], RegSet.regs[1][32], RegSet.regs[2][32], RegSet.regs[3][32],
            RegSet.regs[4][32], RegSet.regs[5][32], RegSet.regs[6][32], RegSet.regs[7][32],
            RegSet.regs[8][32], RegSet.regs[9][32], RegSet.regs[10][32], RegSet.regs[11][32],
            RegSet.regs[12][32], RegSet.regs[13][32], RegSet.regs[14][32], RegSet.regs[15][32]);

        $display("D read x%d=%h x%d=%h SelImm=%b %h",
            d_RdNo1, regset_rd1, d_RdNo2, regset_rd2, vSelImm, ImmALU_w);
        $display("D Bubble=%b%b%b FetchAddr_q=%h Delayed=%h Partial=%h",
            BubbleD_d, BubbleE_q, BubbleM_q, FetchAddr_q, DelayedInsn_q, PartialInsn_q);
        $display("D P=%h align=%b%b RVC=%b",
            PartialInsn_d, RealignedPC_d, OddPC_d, RVCInsn_w);
        $display("D Imm21PCRel=%b",
            Imm21PCRel_w);


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

        $display("X Exc GInsnDEM %b%b%b GJumpFDE %b%b%b MemEMW %b%b%b vWriteMEPC=%b",
            ExcGrubbyInsn,
            e_ExcGrubbyInsn,
            m_ExcGrubbyInsn,
            f_ExcGrubbyJump,
            d_ExcGrubbyJump,
            e_ExcGrubbyJump,
            MemMisaligned,
            m_ExcMem,
            w_ExcMem,
            vWriteMEPC
            );

        if (Kill) $display("B \033[1;35mjump %h\033[0m", FetchAddr_d);

/*
        $display("B vBEQ=%b vNotBEQ=%b e_InsnJALR=%b KillEMW=%b%b%b",
            vBEQ, vNotBEQ, e_InsnJALR, Kill, m_Kill, w_Kill);
        $display("  e_InsnBLTorBLTU=%b vLess=%b e_BranchUncondPCRel=%b ReturnPC=%b",
            e_InsnBLTorBLTU, vLess, e_BranchUncondPCRel, ReturnPC);
        $display("  e_InsnBEQ=%b InvertBranch_q=%b vEqual=%b",
            e_InsnBEQ, InvertBranch_q, vEqual);
*/


/*
        $display("G MemResultGrubby=%b e_AGrubby=%b ExcGrubbyJump",
            MemResultGrubby, e_AGrubby, ExcGrubbyJump);
        $display("G FwdAEGrubby=%b d_RdNo1[5]=%b w_WrGrubby=%b regset_rg1=%b",
            ForwardAEGrubby, d_RdNo1[5], w_WrGrubby, regset_rg1);
*/

/*
    wire ForwardAWGrubby =  FwdAW ? w_WrGrubby : regset_rg1;
    wire ForwardAMGrubby =  FwdAM ? MemResultGrubby : ForwardAWGrubby;
    wire ForwardAEGrubby = (FwdAE ? ALUResultGrubby : ForwardAMGrubby)
        & ~d_RdNo1[5];
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

//        $display("M MemResult=%h m_MemSign=%b m_MemByte=%b",
//            MemResult, m_MemSign, m_MemByte);
        $display("X OverwriteDEM=%b%b%b ValDEM=%h %h %h SelE=%b",
            Overwrite_d, OverwriteE_q, OverwriteM_q,
            OverwriteVal_d, OverwriteValE_q, OverwriteValM_q,
            OverwriteSelE_q);


//        $display("  DecodeWrNo=%b e_WrNo=%d ExecuteWrNo=%d m_WrNo=%d",
//            DecodeWrNo, e_WrNo, ExecuteWrNo, m_WrNo);
//        $display("  DestReg0Part=%b DisableWrite=%b EnableWrite2=%b WrEnEMW=%b%b%b",
//            DestReg0Part, DisableWrite, EnableWrite2, DecodeWrEn, ExecuteWrEn, MemWrEn);
        $display("X vWriteMEPC=%b m_WriteMTVAL=%b m_WriteMCAUSE=%b m_Cause=%h",
            vWriteMEPC, m_WriteMTVAL, m_WriteMCAUSE, m_Cause);
        $display("X ExcWrData=%h e_ExcWrData2=%h m_ExcWrData=%h CsrResult=%h",
            ExcWrData, e_ExcWrData2, m_ExcWrData, CsrResult);
//        $display("  MemWidth=%b e_MemWidth=%b m_MemByte=%b m_MemSign=%b",
//            MemWidth, e_MemWidth,  m_MemByte, m_MemSign);


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
            DelayedInsnGrubby_q <= 0;

            StartMulDiv_q <= 0;

            // fake a jump to address 0 on reset
            m_Kill <= 0;
            w_Kill <= 0;
            e_PCImm <= START_PC;
            e_BranchUncondPCRel  <= 1;

            f_MModeIntEnable <= 0;
            f_MModePriorIntEnable <= 0;
            IrqResponse_q <= 0;
            NoIrq_q <= 0;
        end
    end


`ifdef RISCV_FORMAL
    wire [WORD_WIDTH-1:0] RvfiForwardB =
        (e_WrEn & (d_RdNo2 == e_WrNo)) ? ALUResult :
        (m_WrEn & (d_RdNo2 == m_WrNo)) ? MemResult :
        FwdBW ? w_WrData  : regset_rd2;
        // ForwardBE, FwdBM and FwdBE cannot be used, because they also
        // depend on vImm

    wire RvfiTrap_w = RvfiTrapM_q // illegal instruction
                            | (m_CsrRead & ~m_Kill & ~w_Kill & ~csr_valid); // illegal CSR

    reg [31:0]           RvfiCycle_q;

    reg                  RvfiResetN1_q = 0;
    reg                  RvfiResetN2_q = 0;
    reg [31:0]           RvfiInsnE_q;
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
    reg [WORD_WIDTH-1:0] RvfiRdData2E_q;
    reg [WORD_WIDTH-1:0] RvfiRdData2M_q;
    reg [WORD_WIDTH-1:0] RvfiPcE_q;
    reg [WORD_WIDTH-1:0] RvfiPcM_q;
    reg [WORD_WIDTH-1:0] RvfiNextPcM_q;
    reg                  RvfiExcRetM_q;
    reg [WORD_WIDTH-1:0] RvfiMemAddrM_q;
    reg [3:0]            RvfiMemRMaskM_q;
    reg [3:0]            RvfiMemWMaskM_q;
    reg [WORD_WIDTH-1:0] RvfiMemWDataM_q;
    reg                  RvfiAltResultM_q;
    reg [WORD_WIDTH-1:0] RvfiResultM_q;

    always @(posedge clk) begin
        RvfiCycle_q      <= rstn ? RvfiCycle_q + 1 : 0;

        rvfi_valid      <= rstn & !rvfi_halt & !RvfiHalt_q & (RetiredM_q | RvfiTrapM_q);
        rvfi_order      <= RvfiResetN2_q ? rvfi_order + rvfi_valid : 0;
        rvfi_insn       <= (RvfiInsnM_q[1:0]==3) ? RvfiInsnM_q : RvfiInsnM_q[15:0];
//        rvfi_trap       <= RvfiTrapM_q // illegal instruction
//                            | (m_CsrRead & ~csr_valid); // illegal CSR
        rvfi_trap       <= RvfiTrap_w;

        rvfi_halt       <= RvfiTrap_w | RvfiHalt_q;
        rvfi_intr       <= RvfiIntrM_q;
        rvfi_mode       <= 3;
        rvfi_ixl        <= 1;

        rvfi_rs1_addr   <= RvfiRdNo1M_q;
        rvfi_rs2_addr   <= RvfiRdNo2M_q;
        rvfi_rs1_rdata  <= RvfiRdData1M_q;
        rvfi_rs2_rdata  <= RvfiRdData2M_q;
        rvfi_rd_addr    <= (MemWrEn & ~MemWrNo[5]) ? MemWrNo : 0;
        rvfi_rd_wdata   <= RvfiAltResultM_q ? RvfiResultM_q :
                           (MemWrEn & ~MemWrNo[5]) ? MemResult : 0;

        rvfi_pc_rdata   <= RvfiPcM_q;
        rvfi_pc_wdata   <= RvfiExcRetM_q ? FetchAddr_d : RvfiNextPcM_q;

        rvfi_mem_addr   <= RvfiMemAddrM_q;
        rvfi_mem_rmask  <= RvfiMemRMaskM_q;
        rvfi_mem_wmask  <= RvfiMemWMaskM_q;
        rvfi_mem_rdata  <= mem_rdata;
        rvfi_mem_wdata  <= RvfiMemWDataM_q;

        if (!rstn | !RvfiResetN1_q | !RvfiResetN2_q) begin
            rvfi_rd_addr    <= 0;
            rvfi_rd_wdata   <= 0;
            RvfiHalt_q      <= 0;
        end

        RvfiResetN2_q   <= RvfiResetN1_q;
        RvfiResetN1_q   <= rstn; // delay reset to hide first 2 cycles
        RvfiInsnM_q     <= RvfiInsnE_q;
        RvfiInsnE_q     <= Insn_q;
//        RvfiTrapM_q     <= (RvfiTrapE_q | MemMisaligned) & ~m_Kill;
        RvfiTrapM_q     <= (RvfiTrapE_q | MemMisaligned | (MC_q && MCState_q==mcWFI)) & ~m_Kill;
            // trap when WFI
        RvfiTrapE_q     <= ExcInvalidInsn;
        RvfiHalt_q      <= RvfiHalt_q | rvfi_trap; // halt after first trap
        RvfiIntrM_q     <= RvfiIntrE_q; // long delay (jump to MTVEC)
        RvfiIntrE_q     <= RvfiIntrD_q;
        RvfiIntrD_q     <= RvfiIntrF_q;
        RvfiIntrF_q     <= RvfiIntr1_q;
        RvfiIntr1_q     <= RvfiIntr0_q;
        RvfiIntr0_q     <= MC_q & (MCState_q==mcException);

        RvfiRdNo1M_q    <= RvfiRdNo1E_q;
        RvfiRdNo1E_q    <= d_RdNo1[5] ? 5'b0 : d_RdNo1; // no CSRs
        RvfiRdNo2M_q    <= RvfiRdNo2E_q;
        RvfiRdNo2E_q    <= d_RdNo2[5] ? 5'b0 : d_RdNo2; // no CSRs
        RvfiRdData1M_q  <= e_A;
        RvfiRdData2M_q  <= RvfiRdData2E_q;
        RvfiRdData2E_q  <= RvfiForwardB;

        RvfiPcM_q       <= RvfiPcE_q;
        RvfiPcE_q       <= PC_q;
        RvfiNextPcM_q   <= Kill ? FetchAddr_d : PC_q;
        RvfiExcRetM_q   <= MC_q & ((MCState_q==mcJumpReg) | (MCState_q==mcException));


        RvfiMemAddrM_q  <= {mem_addr[31:2], 2'b00}; // MemAddr;
        RvfiMemRMaskM_q <= ((e_MemWidth!=2'b11) & ~e_MemStore) ? 4'b1111 : 0;
        RvfiMemWMaskM_q <= mem_write ? mem_wmask : 0;
        RvfiMemWDataM_q <= mem_wdata & //MemWriteData;
                            {{8{mem_wmask[3]}}, {8{mem_wmask[2]}},
                             {8{mem_wmask[1]}}, {8{mem_wmask[0]}}};


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



    end
`endif



endmodule


// SPDX-License-Identifier: ISC
