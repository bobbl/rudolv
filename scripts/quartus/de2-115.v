/* wrapper for Altera DE2-115 board

Memory map
0000'0000h main memory (BRAM, 64 KiByte)
0000'FE00h start address of boot loader

CSR
BC0h    UART
BC1h    LEDs
BC2h    Timer

press button KEY[3] to reset
switch SW[17] enables grubby security feature
*/


module top (
    input CLOCK_50,
    input [3:3] KEY,
    input [17:17] SW,
    input UART_RXD,
    output UART_TXD,
    output [17:0] LEDR
);
    localparam integer CLOCK_RATE = 50_000_000;
    localparam integer BAUD_RATE = 115200;

    wire clk = CLOCK_50;
    reg [5:0] reset_counter = 0;
    reg reset_button;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
        reset_button <= KEY[3];
    end
    wire rstn = &reset_counter & reset_button;

    wire        mem_valid;
    wire        mem_write;
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire        mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire        mem_rgrubby_from_mem;

    reg grubby_switch;
    always @(posedge clk) begin
        grubby_switch <= SW[17];
    end
`ifdef ENABLE_GRUBBY
    wire mem_rgrubby_to_pipe = mem_rgrubby_from_mem & grubby_switch;
`else
    wire mem_rgrubby_to_pipe = 0;
`endif

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

    wire        irq_software = 0;
    wire        irq_timer;
    wire        irq_external = 0;
    wire        retired;

    wire        csr_read;
    wire  [2:0] csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata;
    wire        csr_valid;

    CsrDefault #(
        .OUTPINS_COUNT(18),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE),
        .TIMER_WIDTH(32)
    ) uart (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (csr_rdata),
        .valid  (csr_valid),

        .retired(retired),
        .rx     (UART_RXD),
        .tx     (UART_TXD),
        .outpins(LEDR),
        .irq_timer(irq_timer),

        .AVOID_WARNING()
    );

    Pipeline #(
        .START_PC       (32'h_0000_FE00)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

        .irq_software   (irq_software),
        .irq_timer      (irq_timer),
        .irq_external   (irq_external),
        .retired        (retired),

        .csr_read       (csr_read),
        .csr_modify     (csr_modify),
        .csr_wdata      (csr_wdata),
        .csr_addr       (csr_addr),
        .csr_rdata      (csr_rdata),
        .csr_valid      (csr_valid),

        .mem_valid      (mem_valid),
        .mem_write      (mem_write),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_wgrubby    (mem_wgrubby),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata),
        .mem_rgrubby    (mem_rgrubby_to_pipe),

        .regset_we      (regset_we),
        .regset_wa      (regset_wa),
        .regset_wd      (regset_wd),
        .regset_wg      (regset_wg),
        .regset_ra1     (regset_ra1),
        .regset_ra2     (regset_ra2),
        .regset_rd1     (regset_rd1),
        .regset_rg1     (regset_rg1),
        .regset_rd2     (regset_rd2),
        .regset_rg2     (regset_rg2)
    );

    Memory4x9 #(
        .ADDR_WIDTH(14),
        .CONTENT_BYTE0("bootloader.byte0.hex"),
        .CONTENT_BYTE1("bootloader.byte1.hex"),
        .CONTENT_BYTE2("bootloader.byte2.hex"),
        .CONTENT_BYTE3("bootloader.byte3.hex")
    ) mem (
        .clk    (clk),
        .write  (mem_write),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .wgrubby(mem_wgrubby),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata),
        .rgrubby(mem_rgrubby_from_mem)
    );

    RegSet33 regset (
        .clk    (clk),
        .we     (regset_we),
        .wa     (regset_wa),
        .wd     (regset_wd),
        .wg     (regset_wg),
        .ra1    (regset_ra1),
        .ra2    (regset_ra2),
        .rd1    (regset_rd1),
        .rg1    (regset_rg1),
        .rd2    (regset_rd2),
        .rg2    (regset_rg2)
    );

endmodule


// SPDX-License-Identifier: ISC
