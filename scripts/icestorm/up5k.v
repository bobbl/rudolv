/* wrapper for iCE40 UP5K MDP board

Memory map
0000'0000h 64KiB main memory (SPRAM)
0002'0000h  1KiB boot loader (BRAM)

CSR
BC0h    UART
BC2h    Timer
*/


module top (
    input uart_rx,
    output uart_tx
);
    localparam integer CLOCK_RATE = 24_000_000;
    //localparam integer CLOCK_RATE = 12_000_000;
    localparam integer BAUD_RATE = 115200;

    wire clk;

    SB_HFOSC OSCInst0(
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF(clk)   );
    defparam OSCInst0.CLKHF_DIV = "0b01"; // 48 MHz / 2
    //defparam OSCInst0.CLKHF_DIV = "0b10"; // 48 MHz / 4

    reg [5:0] reset_counter = 0;
    wire rstn = &reset_counter;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
    end

    wire        mem_valid;
    wire        mem_write;
    wire        mem_write_main = mem_write & ~mem_waddr[17];
    wire        mem_write_boot = mem_write & mem_waddr[17];
    wire  [3:0] mem_wmask;
//    wire  [3:0] mem_wmask_main = (mem_write & ~mem_addr[17]) ? mem_wmask : 0;
//    wire  [3:0] mem_wmask_boot = (mem_write &  mem_addr[17]) ? mem_wmask : 0;
    wire [31:0] mem_wdata;
    wire [31:0] mem_waddr;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_main;
    wire [31:0] mem_rdata_boot;
    wire [31:0] mem_rdata = q_SelBootMem ? mem_rdata_boot : mem_rdata_main;

    reg q_SelBootMem;
    always @(posedge clk) begin
        q_SelBootMem <= rstn ? mem_addr[17] : 0;
    end

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
    wire        leds;

    wire        csr_read;
    wire        csr_write;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata;
    wire        csr_valid;

    CsrDefault #(
        .OUTPINS_COUNT(1),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE),
        .TIMER_WIDTH(32)
    ) csrs (
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
        .START_PC       (32'h0002_0000)
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
        .mem_waddr      (mem_waddr),
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

    SPRAMMemory mainmem (
        .clk    (clk),
        .write  (mem_write_main),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_main)
    );

    Memory32 #(
        .ADDR_WIDTH(8), // 1024 bytes
        .CONTENT("../../sw/uart/build/bootloader_char.hex")
    ) bootmem (
        .clk    (clk),
        .write  (mem_write_boot),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[9:2]),
        .rdata  (mem_rdata_boot)
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



module SPRAMMemory (
    input clk,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [13:0] addr,
    output reg [31:0] rdata
);

SB_SPRAM256KA spram_lo(
    .DATAIN     (wdata[15:0]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[1], wmask[1], wmask[0], wmask[0]}),
    .WREN       (write),
    .CHIPSELECT (1'b1),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[15:0])
    );

SB_SPRAM256KA spram_hi(
    .DATAIN     (wdata[31:16]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[3], wmask[3], wmask[2], wmask[2]}),
    .WREN       (write),
    .CHIPSELECT (1'b1),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[31:16])
    );

endmodule


// SPDX-License-Identifier: ISC
