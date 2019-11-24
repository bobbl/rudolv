/* Connect pipeline with memory that can be infered by 
 * Microsemi IGLOO2 M2GL025 (Future Electronics Creative Board)

Memory map
0000'0000h 60KiB main memory (BRAM)
0002'0000h 128B  boot loader (LUTs)

CSR
BC0h       UART
*/

module top (
    input  FTDI_UART0_TXD,
    output FTDI_UART0_RXD,
    output LED1_GREEN,
    output LED1_RED,
    output LED2_GREEN,
    output LED2_RED
);
//    localparam integer CLOCK_RATE = 99_000_000;
    localparam integer CLOCK_RATE = 100_000_000;
    localparam integer BAUD_RATE = 115200;

    wire clk50;
    wire clk;
    wire rstn;

    OSC_C0 osc(
        .RCOSC_25_50MHZ_CCC (clk50) 
    );

    FCCC_C0 pll(
        .RCOSC_25_50MHZ (clk50),
        .GL0            (clk),
        .LOCK           (rstn)
    );

    wire [3:0] leds;
    assign LED1_GREEN = leds[0];
    assign LED1_RED   = leds[1];
    assign LED2_GREEN = leds[2];
    assign LED2_RED   = leds[3];


    reg q_SelBootMem;

    wire        mem_valid;
    wire        mem_write;
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire        MemWGrubby = ((mem_wmask != 4'b1111) | mem_wgrubby);
    wire [31:0] mem_addr;
    wire        mem_write_main = mem_write & ~mem_addr[17];
    wire        mem_write_boot = mem_write & mem_addr[17];
    wire [31:0] mem_rdata_main;
    wire [31:0] mem_rdata_boot;
    wire [31:0] mem_rdata   = q_SelBootMem ? mem_rdata_boot : mem_rdata_main[31:0];
    wire        mem_rgrubby_main;
    wire        mem_rgrubby = q_SelBootMem ? 1'b0 : mem_rgrubby_main;

    always @(posedge clk) begin
        q_SelBootMem <= rstn ? mem_addr[17] : 0;
    end

    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        UartValid;
    wire [31:0] UartRData;

    wire        retired;
    wire        irq_software = 0;
    wire        irq_timer = 0;
    wire        irq_external = 0;

    wire        csr_read;
    wire [2:0]  csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata = CounterRData | UartRData;
    wire        csr_valid = CounterValid | UartValid;

    CsrCounter counter (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (CounterRData),
        .valid  (CounterValid),

        .retired(retired)
    );

    CsrPinsOut #(
        .BASE_ADDR('hBC1),
        .COUNT(4),
        .RESET_VALUE('b1)
    ) pinout (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),

        .pins   (leds)
    );

    CsrUartChar #(
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE)
    ) uart (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (UartRData),
        .valid  (UartValid),

        .rx     (FTDI_UART0_TXD),
        .tx     (FTDI_UART0_RXD)
    );

    Pipeline #(
        .START_PC       (32'h0002_0000)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

        .retired        (retired),
        .irq_software   (irq_software),
        .irq_timer      (irq_timer),
        .irq_external   (irq_external),

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
        .mem_rgrubby    (mem_rgrubby)
    );

    Memory #(
        .WIDTH(13)
    ) mainmem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_main),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata[31:0]),
        .wgrubby(MemWGrubby),
        .addr   (mem_addr[14:2]),
        .rdata  (mem_rdata_main),
        .rgrubby(mem_rgrubby_main)
    );

    ROM32 bootmem (
        .clk    (clk),
        .addr   (mem_addr[6:2]),
        .rdata  (mem_rdata_boot)
    );

endmodule


module ROM32 (
    input clk, 
    input [4:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:31]; // only 0..25 used (104 bytes)

    initial begin
        $readmemh("grubby_char.hex", mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
    end
endmodule



module Mem9 #(
    parameter WIDTH = 13
) (
    input clk,
    input write,
    input [8:0] wdata,
    input [WIDTH-1:0] addr,
    output reg [8:0] rdata
);
    reg [8:0] mem [0:(1<<WIDTH)-1];

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) mem[addr] <= wdata;
    end
endmodule


// 36 bit single ported zero latency memory
module Memory #(
    parameter WIDTH = 13
) (
    input clk, 
    input valid,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input wgrubby,
    input [WIDTH-1:0] addr,
    output [31:0] rdata,
    output rgrubby
);
    wire WMask0 = valid & write & wmask[0];
    wire WMask1 = valid & write & wmask[1];
    wire WMask2 = valid & write & wmask[2];
    wire WMask3 = valid & write & wmask[3];
    wire [8:0] WData0 = {wgrubby, wdata[7:0]};
    wire [8:0] WData1 = {1'b0, wdata[15:8]};
    wire [8:0] WData2 = {1'b0, wdata[23:16]};
    wire [8:0] WData3 = {1'b0, wdata[31:24]};
    wire [8:0] RData0;
    wire [8:0] RData1;
    wire [8:0] RData2;
    wire [8:0] RData3;

    Mem9 #(
        .WIDTH(WIDTH)
    ) mem0 (
        .clk    (clk),
        .write  (WMask0),
        .wdata  (WData0),
        .addr   (addr),
        .rdata  (RData0)
    );
    Mem9 #(
        .WIDTH(WIDTH)
    ) mem1 (
        .clk    (clk),
        .write  (WMask1),
        .wdata  (WData1),
        .addr   (addr),
        .rdata  (RData1)
    );
    Mem9 #(
        .WIDTH(WIDTH)
    ) mem2 (
        .clk    (clk),
        .write  (WMask2),
        .wdata  (WData2),
        .addr   (addr),
        .rdata  (RData2)
    );
    Mem9 #(
        .WIDTH(WIDTH)
    ) mem3 (
        .clk    (clk),
        .write  (WMask3),
        .wdata  (WData3),
        .addr   (addr),
        .rdata  (RData3)
    );

    assign rdata = {RData3[7:0], RData2[7:0], RData1[7:0], RData0[7:0]};
    assign rgrubby = RData0[8];
endmodule


module RegSet36(
    input clk, 
    input we,
    input [5:0] wa,
    input [35:0] wd,
    input [5:0] ra1,
    input [5:0] ra2,
    output reg [35:0] rd1,
    output reg [35:0] rd2
);
    reg [35:0] regs [0:63];

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= ra1 ? regs[ra1] : 0;
        rd2 <= ra2 ? regs[ra2] : 0;
    end
endmodule


module RegisterSet(
    input clk, 
    input we,
    input [5:0] wa,
    input [31:0] wd,
    input wg,
    input [5:0] ra1,
    input [5:0] ra2,
    output [31:0] rd1,
    output rg1,
    output [31:0] rd2,
    output rg2
);
    wire [35:0] WriteData = {3'b0, wg, wd};
    wire [35:0] ReadData1;
    wire [35:0] ReadData2;

    RegSet36 regs (
        .clk    (clk),
        .we     (we),
        .wa     (wa),
        .wd     (WriteData),
        .ra1    (ra1),
        .ra2    (ra2),
        .rd1    (ReadData1),
        .rd2    (ReadData2)
    );

    assign rd1 = ReadData1[31:0];
    assign rg1 = ReadData1[32];
    assign rd2 = ReadData2[31:0];
    assign rg2 = ReadData2[32];
endmodule
