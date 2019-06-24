/* wrapper for iCE40 UP5K MDP board

Memory map
0000'0000h 64KiB main memory (SPRAM)
0002'0000h  1KiB boot loader (BRAM)

CSR
7c0h       UART
*/

module top (
    input uart_rx,
    output uart_tx
);
//    localparam integer CLOCK_RATE = 24_000_000;
    localparam integer CLOCK_RATE = 12_000_000;
    localparam integer BAUD_RATE = 115200;


    wire clk;

    SB_HFOSC OSCInst0(
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF(clk)   );
//    defparam OSCInst0.CLKHF_DIV = "0b01"; // 48 MHz / 2
    defparam OSCInst0.CLKHF_DIV = "0b10"; // 48 MHz / 4

    reg [5:0] reset_counter = 0;
    wire rstn = &reset_counter;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
    end

    wire        mem_valid;
    wire        mem_write;
    wire        mem_write_main =  mem_write & ~mem_addr[17];
    wire        mem_write_boot = 1'b1; // mem_write & mem_addr[17];
    wire  [3:0] mem_wmask;
    wire  [3:0] mem_wmask_main = (mem_write & ~mem_addr[17]) ? mem_wmask : 0;
    wire  [3:0] mem_wmask_boot = (mem_write &  mem_addr[17]) ? mem_wmask : 0;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_main;
    wire [31:0] mem_rdata_boot;
    wire [31:0] mem_rdata = q_SelBootMem ? mem_rdata_boot : mem_rdata_main;

    reg q_SelBootMem;
    always @(posedge clk) begin
        q_SelBootMem <= rstn ? mem_addr[17] : 0;
    end

    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        UartValid;
    wire [31:0] UartRData;

    wire        retired;
    wire        csr_read;
    wire [1:0]  csr_modify;
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

    CsrUart uart (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (UartRData),
        .valid  (UartValid),

        .rx     (uart_rx),
        .tx     (uart_tx)
    );

    Pipeline #(
        .START_PC       (32'h_0002_0000)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

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
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata)
    );

    SPRAMMemory mainmem (
        .clk    (clk),
        .write  (mem_write_main),
        .wmask  (mem_wmask_main),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_main)
    );

    BRAMMemory bootmem (
        .clk    (clk),
        .write  (mem_write_boot),
        .wmask  (mem_wmask_boot),
        .wdata  (mem_wdata),
        .addr   (mem_addr[9:2]),
        .rdata  (mem_rdata_boot)
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


module BRAMMemory (
    input clk, 
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [7:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:255];

    initial begin
        $readmemh("../../sw/bootloader/bootloader.hex", mem);
//        $readmemh("../../sw/bootloader/tiny.hex", mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) begin
            if (wmask[0]) mem[addr][7:0] <= wdata[7:0];
            if (wmask[1]) mem[addr][15:8] <= wdata[15:8];
            if (wmask[2]) mem[addr][23:16] <= wdata[23:16];
            if (wmask[3]) mem[addr][31:24] <= wdata[31:24];
        end
    end
endmodule

