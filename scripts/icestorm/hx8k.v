/* wrapper for iCE40 HX8K breakout board

Memory map
0000'0000h main memory (BRAM, 8 KiByte)
0000'1E00h start address of boot loader

CSR
bc0h       UART
bc1h       LEDs
*/

module top (
    input clk,
    input uart_rx,
    output uart_tx,
    output [7:0] leds
);
    localparam integer CLOCK_RATE = 12_000_000;
    localparam integer BAUD_RATE = 115200;

    reg [5:0] reset_counter = 0;
    wire rstn = &reset_counter;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
    end

    wire mem_valid;
    wire mem_write;
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire        mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire        mem_rgrubby = 0;


    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        UartValid;
    wire [31:0] UartRData;
    wire        LedsValid;
    wire [31:0] LedsRData;

    wire        retired;
    wire        irq_timer=0;

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

        .rx     (uart_rx),
        .tx     (uart_tx)
    );

/*
    CsrLeds csr_leds (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (LedsRData),
        .valid  (LedsValid),

        .leds   (leds)
    );
*/
    CsrPinsOut #(
        .BASE_ADDR(12'hbc1),
        .COUNT(8)
    ) csr_leds (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (LedsRData),
        .valid  (LedsValid),

        .pins   (leds)
    );

    Pipeline #(
        .START_PC       (32'h_0000_1e00)
//        .START_PC       (32'h_0000_0000)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

        .retired        (retired),
        .irq_timer      (irq_timer),

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

    BRAMMemory mem (
        .clk    (clk),
        .write  (mem_write),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[12:2]),
        .rdata  (mem_rdata)
    );
endmodule


module BRAMMemory (
    input clk, 
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [10:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:2047];

    initial begin
        $readmemh("hx8k_bootloader.hex", mem);
            // bootloader code is the same as on other platforms, but at the
            // beginning there must be '@1e00' to load the code at the correct
            // start adress
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

/*
module CsrLeds #(
    parameter [11:0]  BASE_ADDR  = 12'hbc1 // CSR address
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output [7:0] leds
);

    reg [7:0] q_Leds;
    reg Valid;
    reg [31:0] RData;

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= q_Leds;
            case (modify)
                3'b001: q_Leds <= wdata[7:0]; // write
                3'b010: q_Leds <= q_Leds | wdata[7:0]; // set
                3'b011: q_Leds <= q_Leds &~ wdata[7:0]; // clear
            endcase
        end
        if (~rstn) q_Leds <= 'h81;
    end

    assign valid = Valid;
    assign rdata = RData;
    assign leds = q_Leds;
endmodule
*/