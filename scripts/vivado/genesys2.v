/* wrapper for Digilent Genesys 2 board

Memory map
0000'0000h main memory (BRAM, 64 KiByte)
0000'FE00h start address of boot loader

CSR
BC0h       UART
BC1h       LEDs

press button BTN1 to reset
switch SW7 enables grubby security feature
*/


module top (
    input clk_p,
    input clk_n,
    input cpu_resetn,
    input [7:7] sw,
    input uart_rx,
    output uart_tx,
    output [7:0] leds
);
    localparam integer CLOCK_RATE = 200_000_000;
    localparam integer BAUD_RATE = 115200;

    localparam CSR_SIM   = 12'h3FF;
    localparam CSR_UART  = 12'hBC0;
    localparam CSR_LEDS  = 12'hBC1;
    localparam CSR_SWI   = 12'hBC1;
    localparam CSR_TIMER = 12'hBC2;
    localparam CSR_KHZ   = 12'hFC0;


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
    wire        mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire        mem_rgrubby_from_mem;

    reg grubby_switch;
    always @(posedge clk) begin
        grubby_switch <= sw[7];
    end
`ifdef ENABLE_GRUBBY
    wire mem_rgrubby_to_pipe = mem_rgrubby_from_mem & grubby_switch;
`else
    wire mem_rgrubby_to_pipe = 0;
`endif

    wire        IDsValid;
    wire [31:0] IDsRData;
    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        UartValid;
    wire [31:0] UartRData;
    wire        LedsValid;
    wire [31:0] LedsRData;
    wire        TimerValid;
    wire [31:0] TimerRData;

    wire        irq_software = 0;
    wire        irq_timer;
    wire        irq_external = 0;
    wire        retired;

    wire        csr_read;
    wire  [2:0] csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata = IDsRData | CounterRData | UartRData | TimerRData;
    wire        csr_valid = IDsValid | CounterValid | UartValid | TimerValid;

    CsrIDs #(
        .BASE_ADDR(CSR_KHZ),
        .KHZ(CLOCK_RATE/1000)
    ) csr_ids (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (IDsRData),
        .valid  (IDsValid),

        .AVOID_WARNING()
    );

    CsrCounter counter (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (CounterRData),
        .valid  (CounterValid),

        .retired(retired),

        .AVOID_WARNING()
    );

    CsrUartChar #(
        .BASE_ADDR(CSR_UART),
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
        .tx     (uart_tx),

        .AVOID_WARNING()
    );

    CsrPinsOut #(
        .BASE_ADDR(CSR_LEDS),
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

        .pins   (leds),

        .AVOID_WARNING()
    );

    CsrTimerAdd #(
        .BASE_ADDR(CSR_TIMER),
        .WIDTH(32)
    ) Timer (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (TimerRData),
        .valid  (TimerValid),

        .irq    (irq_timer),

        .AVOID_WARNING()
    );

    Pipeline #(
        .START_PC       (32'h_0000_fe00)
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
        .mem_rgrubby    (mem_rgrubby_to_pipe)
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

endmodule


// SPDX-License-Identifier: ISC
