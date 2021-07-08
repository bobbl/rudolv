// BRAM with preinit for x0, parity bit for grubby
// Xilinx, Icarus
module RegSet32(
    input             clk,
    input             we,
    input       [5:0] wa,
    input      [31:0] wd,
    input             wg,
    input       [5:0] ra1,
    input       [5:0] ra2,
    output reg [31:0] rd1,
    output reg        rg1,
    output reg [31:0] rd2,
    output reg        rg2
);
    reg [31:0] regs [0:63];

    integer i;
    initial begin
        for (i=0; i<64; i++) regs[0] = 0;
            // riscv-formal requires a defined state after reset
    end

    always @(posedge clk) begin
        if (we && wa!=0) regs[wa] <= wd;
        rd1 <= regs[ra1];
        rd2 <= regs[ra2];
        rg1 <= 0;
        rg2 <= 0;
    end
endmodule

module rvfi_wrapper (
    input clock,
    input reset,
    `RVFI_OUTPUTS
);
    (* keep *) wire irq_software = 0;
    (* keep *) wire irq_timer = 0;
    (* keep *) wire irq_external = 0;
    (* keep *) wire retired;

    (* keep *) wire        csr_read;
    (* keep *) wire        csr_write;
    (* keep *) wire [31:0] csr_wdata;
    (* keep *) wire [11:0] csr_addr;
    wire [31:0] csr_rdata = 0;
    wire        csr_valid = 0;

    (* keep *) wire        mem_valid;
    (* keep *) wire        mem_write;
    (* keep *) wire  [3:0] mem_wmask;
    (* keep *) wire [31:0] mem_wdata;
    (* keep *) wire        mem_wgrubby;
    (* keep *) wire [31:0] mem_addr;
    (* keep *) `rvformal_rand_reg [31:0] mem_rdata;
    (* keep *) wire        mem_rgrubby = 0;

    wire        regset_we;
    wire  [5:0] regset_wa;
    wire [31:0] regset_wd;
    wire        regset_wg;
    wire  [5:0] regset_ra1;
    wire  [5:0] regset_ra2;
    wire [31:0] regset_rd1;
    wire        regset_rg1;
    wire [31:0] regset_rd2;
    wire        regset_rg2;

    Pipeline uut (
        .clk            (clock),
        .rstn           (!reset),

        .irq_software   (irq_software),
        .irq_timer      (irq_timer),
        .irq_external   (irq_external),
        .retired        (retired),

        .csr_read       (csr_read),
        .csr_write      (csr_write),
        .csr_wdata      (csr_wdata),
        .csr_addr       (csr_addr),
        .csr_rdata      (csr_rdata),
        .csr_valid      (csr_valid),

        .mem_valid      (mem_valid),
        .mem_write      (mem_write),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata),

        .regset_we      (regset_we),
        .regset_wa      (regset_wa),
        .regset_wd      (regset_wd),
        .regset_ra1     (regset_ra1),
        .regset_ra2     (regset_ra2),
        .regset_rd1     (regset_rd1),
        .regset_rd2     (regset_rd2),

        `RVFI_CONN
    );

    RegSet32 RegSet (
        .clk    (clock),
        .we     (regset_we),
        .wa     (regset_wa),
        .wd     (regset_wd),
        .ra1    (regset_ra1),
        .ra2    (regset_ra2),
        .rd1    (regset_rd1),
        .rd2    (regset_rd2)
    );

endmodule


// SPDX-License-Identifier: ISC
