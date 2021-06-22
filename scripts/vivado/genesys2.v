/* wrapper for Digilent Genesys 2 board

Memory map
0000'0000h main memory (BRAM, 64 KiByte)
0000'FE00h start address of boot loader

CSR
BC0h    UART
BC1h    LEDs
BC2h    Timer

press button BTN1 to reset
*/


module top (
    input clk_p,
    input clk_n,
    input cpu_resetn,
    input uart_rx,
    output uart_tx,
    output [7:0] leds
);
    localparam integer CLOCK_RATE = 200_000_000;
    localparam integer BAUD_RATE = 115200;

    wire clk;
    IBUFGDS clk_inst (
        .O(clk),
        .I(clk_p),
        .IB(clk_n)
    );

    reg [5:0] reset_counter = 0;
    reg reset_button;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
        reset_button <= cpu_resetn;
    end
    wire rstn = &reset_counter & reset_button;

    wire        mem_valid;
    wire        mem_write;
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;

    wire        regset_we;
    wire  [5:0] regset_wa;
    wire [31:0] regset_wd;
    wire  [5:0] regset_ra1;
    wire  [5:0] regset_ra2;
    wire [31:0] regset_rd1;
    wire [31:0] regset_rd2;

    wire        irq_software = 0;
    wire        irq_timer;
    wire        irq_external = 0;
    wire        retired;

    wire        csr_read;
    wire        csr_write;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata;
    wire        csr_valid;

    CsrDefault #(
        .OUTPINS_COUNT(8),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE),
        .TIMER_WIDTH(32)
    ) uart (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .write  (csr_write),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (csr_rdata),
        .valid  (csr_valid),

        .retired(retired),
        .rx     (uart_rx),
        .tx     (uart_tx),
        .outpins(leds),
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
        .regset_rd2     (regset_rd2)
    );

    Memory32 #(
        .ADDR_WIDTH(14),
        .CONTENT("bootloader_3f80.hex")
    ) mem (
        .clk    (clk),
        .write  (mem_write),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata)
    );

    RegSet32 regset (
        .clk    (clk),
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
