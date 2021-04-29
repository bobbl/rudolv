/* wrapper for Microsemi IGLOO2 M2GL025 (Future Electronics Creative Board)

Memory map
0000'0000h 56KiB main memory (BRAM)
0002'0000h 128B  boot loader (LUTs)

CSR
BC0h    UART
BC1h    LEDs
BC2h    Timer

press RESET button to reset
hold SW1 to disable grubby security feature
*/




// 9 bit
module Memory9Full #(
    parameter ADDR_WIDTH = 8,
    parameter SIZE = 0,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input [8:0] wdata,
    input [ADDR_WIDTH-1:0] addr,
    output reg [8:0] rdata
);
    localparam integer COMPUTED_SIZE = (SIZE==0) ? (1 << ADDR_WIDTH) : SIZE;

    reg [8:0] mem [0:COMPUTED_SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        if (write) mem[addr] <= wdata;
        else rdata <= mem[addr];
    end
endmodule


module Memory9Microsemi #(
    parameter ADDR_WIDTH = 8,
    parameter SIZE = 0,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input [8:0] wdata,
    input [13:0] addr,
    output reg [8:0] rdata
);

    wire wren0 = write & ~addr[13];
    wire wren1 = write &  addr[13] & ~addr[12];
    wire wren2 = write &  addr[13] &  addr[12];
    wire [8:0] rbyte0;
    wire [8:0] rbyte1;
    wire [8:0] rbyte2;

    reg addr13_q;
    reg addr12_q;
    always @(posedge clk) begin
        addr13_q <= addr[13];
        addr12_q <= addr[12];
    end
    assign rdata = addr13_q ? (addr12_q ? rbyte2 : rbyte1) : rbyte0;

    Memory9Full #(
        .ADDR_WIDTH(13),
        .SIZE('h2000)
    ) mem0 (
        .clk    (clk),
        .write  (wren0),
        .wdata  (wdata),
        .addr   (addr[12:0]),
        .rdata  (rbyte0)
    );

    Memory9Full #(
        .ADDR_WIDTH(12),
        .SIZE('h1000)
    ) mem1 (
        .clk    (clk),
        .write  (wren1),
        .wdata  (wdata),
        .addr   (addr[11:0]),
        .rdata  (rbyte1)
    );

    Memory9Full #(
        .ADDR_WIDTH(11),
        .SIZE('h0800)
    ) mem2 (
        .clk    (clk),
        .write  (wren2),
        .wdata  (wdata),
        .addr   (addr[10:0]),
        .rdata  (rbyte2)
    );

endmodule




// 32 bit plus 4 parity bits used for grubby, byte enable
// Microsemi
module Memory4x9Microsemi #(
    parameter ADDR_WIDTH = 14,
    parameter SIZE = 0,
    parameter CONTENT_BYTE0 = "",
    parameter CONTENT_BYTE1 = "",
    parameter CONTENT_BYTE2 = "",
    parameter CONTENT_BYTE3 = ""
) (
    input clk,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input wgrubby,
    input [ADDR_WIDTH-1:0] addr,
    output [31:0] rdata,
    output rgrubby
);

    wire wren0 = write & wmask[0];
    wire wren1 = write & wmask[1];
    wire wren2 = write & wmask[2];
    wire wren3 = write & wmask[3];
    wire [8:0] rbyte0;
    wire [8:0] rbyte1;
    wire [8:0] rbyte2;
    wire [8:0] rbyte3;

    assign rdata = {rbyte3[7:0], rbyte2[7:0], rbyte1[7:0], rbyte0[7:0]};
    assign rgrubby = rbyte3[8] | rbyte2[8] | rbyte1[8] | rbyte0[8];

    Memory9Microsemi #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .SIZE(SIZE/4),
        .CONTENT(CONTENT_BYTE0)
    ) mem0 (
        .clk    (clk),
        .write  (wren0),
        .wdata  ({wgrubby, wdata[7:0]}),
        .addr   (addr),
        .rdata  (rbyte0)
    );

    Memory9Microsemi #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .SIZE(SIZE/4),
        .CONTENT(CONTENT_BYTE1)
    ) mem1 (
        .clk    (clk),
        .write  (wren1),
        .wdata  ({wgrubby, wdata[15:8]}),
        .addr   (addr),
        .rdata  (rbyte1)
    );

    Memory9Microsemi #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .SIZE(SIZE/4),
        .CONTENT(CONTENT_BYTE2)
    ) mem2 (
        .clk    (clk),
        .write  (wren2),
        .wdata  ({wgrubby, wdata[23:16]}),
        .addr   (addr),
        .rdata  (rbyte2)
    );

    Memory9Microsemi #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .SIZE(SIZE/4),
        .CONTENT(CONTENT_BYTE3)
    ) mem3 (
        .clk    (clk),
        .write  (wren3),
        .wdata  ({wgrubby, wdata[31:24]}),
        .addr   (addr),
        .rdata  (rbyte3)
    );

endmodule



// 32 bit read-only memory
// Microsemi (BRAM cannot be pre-initialised)
module ROM32Microsemi #(
    parameter WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input [WIDTH-1:0] addr,
    output reg [31:0] rdata
);
    localparam integer SIZE = 1 << WIDTH;

    reg [31:0] mem [0:SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
    end
endmodule





module top #(
    parameter integer MHZ = 100
) (
    input  FTDI_UART0_TXD,
    output FTDI_UART0_RXD,
    input  USER_BUTTON1,
    output LED1_GREEN,
    output LED1_RED,
    output LED2_GREEN,
    output LED2_RED
);
    localparam integer CLOCK_RATE = 1_000_000 * MHZ;
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
    assign LED1_GREEN = leds[0] ^ grubby_switch;
        // Necessary to avoid that USER_BUTTON1 is removed from the net when
        // grubby is disabled.
        // If removed, the synthesis stops because a pin is mentioned in the
        // .pdc file but not connected.
    assign LED1_RED   = leds[1];
    assign LED2_GREEN = leds[2];
    assign LED2_RED   = leds[3];


    wire        mem_valid;
    wire        mem_write;
    wire        mem_write_main = mem_write & ~mem_addr[17];
    wire        mem_write_boot = mem_write & mem_addr[17];
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire        mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_main;
    wire [31:0] mem_rdata_boot;
    wire [31:0] mem_rdata   = q_SelBootMem ? mem_rdata_boot : mem_rdata_main[31:0];
    wire        mem_rgrubby_main;

    reg grubby_switch;
    always @(posedge clk) begin
        grubby_switch <= ~USER_BUTTON1;
    end
`ifdef ENABLE_GRUBBY
    wire mem_rgrubby = (q_SelBootMem ? 1'b0 : mem_rgrubby_main) & grubby_switch;
`else
    wire mem_rgrubby = 0;
`endif

    reg q_SelBootMem;
    always @(posedge clk) begin
        q_SelBootMem <= rstn ? mem_addr[17] : 0;
    end

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
        .OUTPINS_COUNT(4),
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
        .rx     (FTDI_UART0_TXD),
        .tx     (FTDI_UART0_RXD),
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
        .mem_rgrubby    (mem_rgrubby),

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

    Memory4x9Microsemi #(
        .ADDR_WIDTH(14),
        .SIZE('he000)
    ) mainmem (
        .clk    (clk),
        .write  (mem_write_main),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .wgrubby(mem_wgrubby),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_main),
        .rgrubby(mem_rgrubby_main)
    );

    ROM32Microsemi #(
        .WIDTH(5),
        .CONTENT("grubby_char.hex")
    ) bootmem (
        .clk    (clk),
        .addr   (mem_addr[6:2]),
        .rdata  (mem_rdata_boot)
    );

    RegSet36Microsemi regset (
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
