/* CSR interface
 *
 * D stage: Set (async) `valid` if `addr` is one of the provided CSRs.
 *          Set (registered) internal enable signals by decoding `addr`.
 * E stage: Set `rdata` according to enable signals.
 *          Since `rdata` will be discarded by the pipeline if not reading,
 *              `read` can be ignored as long as there are no side effects.
 *          Write `wdata` to enabled CSR according to `modify`
 *              (001 write, 010 set, 011 clear)
 *
 * This spreads the CSR logic over 3 stages and relaxes the critical path:
 *     D stage: decode tree
 *     E stage: actual read/write logic
 *     M stage: mux tree for read data
 */




// Machine IDs (from priv spec) and clock frequency (RudolV extension)
module CsrIDs #(
    parameter [31:0] ISA = 0,
    parameter [31:0] VENDORID = 0,
    parameter [31:0] ARCHID = 0,
    parameter [31:0] IMPID = 0,
    parameter [31:0] HARTID = 0,

    parameter [11:0] BASE_ADDR = 12'hfc0, // CSR address for clock frequency
    parameter [31:0] KHZ = 100_000
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | |modify | |wdata;

    reg [31:0] RData_q;
    assign rdata = RData_q;

    // asynchronous in D stage: decode CSR address
    wire SelISA_w    = (addr == 12'h301);
    wire SelVendor_w = (addr == 12'hF11);
    wire SelArch_w   = (addr == 12'hF12);
    wire SelImp_w    = (addr == 12'hF13);
    wire SelHart_w   = (addr == 12'hF14);
    wire SelKHz_w    = (addr == BASE_ADDR);
    assign valid = SelISA_w | SelVendor_w | SelArch_w | SelImp_w | SelHart_w 
                            | SelKHz_w;

    reg SelISA_q;
    reg SelVendor_q;
    reg SelArch_q;
    reg SelImp_q;
    reg SelHart_q;
    reg SelKHz_q;

    always @(posedge clk) begin
        // E stage: read CSR
        RData_q <= (SelISA_q    ? ISA      : 32'b0)
                 | (SelVendor_q ? VENDORID : 32'b0)
                 | (SelArch_q   ? ARCHID   : 32'b0)
                 | (SelImp_q    ? IMPID    : 32'b0)
                 | (SelHart_q   ? HARTID   : 32'b0)
                 | (SelKHz_q    ? KHZ      : 32'b0);

        // D stage: select CSR for next cycle
        SelISA_q    <= SelISA_w;
        SelVendor_q <= SelVendor_w;
        SelArch_q   <= SelArch_w;
        SelImp_q    <= SelImp_w;
        SelHart_q   <= SelHart_w;
        SelKHz_q    <= SelKHz_w;
    end
endmodule





// Counters from unpriv spec 2.2 and lower (RDCYCLE and RDINSTRET)
module CsrCounter (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input retired,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |modify | |wdata;

    reg [31:0] q_RData;
    assign rdata = q_RData;

    // asynchronos in D stage: decode CSR address
    wire SelCycleL_w   = (addr == 12'hB00) |     // MCYCLE
                         (addr == 12'hC00) |     // CYCLE
                         (addr == 12'hC01);      // TIME
    wire SelCycleH_w   = (addr == 12'hB80) |     // MCYCLEH
                         (addr == 12'hC80) |     // CYCLEH
                         (addr == 12'hC81);      // TIMEH
    wire SelInstRetL_w = (addr == 12'hB02) |     // MINSTRET
                         (addr == 12'hC02);      // INSTRET
    wire SelInstRetH_w = (addr == 12'hB82) |     // MINSTRETH
                    (addr == 12'hC82);      // INSRETH
    assign valid = SelCycleL_w | SelCycleH_w | SelInstRetL_w | SelInstRetH_w;

    reg SelCycleL_q;
    reg SelCycleH_q;
    reg SelInstRetL_q;
    reg SelInstRetH_q;

    reg [32:0] CounterCYCLE_q;
    reg [31:0] CounterCYCLEH_q;
    reg [32:0] CounterINSTRET_q;
    reg [31:0] CounterINSTRETH_q;

    always @(posedge clk) begin
        // E stage: read CSR
        q_RData <= (SelCycleL_q   ? CounterCYCLE_q[31:0]   : 32'b0)
                 | (SelCycleH_q   ? CounterCYCLEH_q        : 32'b0)
                 | (SelInstRetL_q ? CounterINSTRET_q[31:0] : 32'b0)
                 | (SelInstRetH_q ? CounterINSTRETH_q      : 32'b0);

        // D stage: select CSR for next cycle
        SelCycleL_q   <= SelCycleL_w;
        SelCycleH_q   <= SelCycleH_w;
        SelInstRetL_q <= SelInstRetL_w;
        SelInstRetH_q <= SelInstRetH_w;

        // increment counters (spread over two cycles)
        CounterCYCLE_q    <= {1'b0, CounterCYCLE_q[31:0]} + 1;
        CounterCYCLEH_q   <= CounterCYCLEH_q + {31'b0, CounterCYCLE_q[32]};
        CounterINSTRET_q  <= {1'b0, CounterINSTRET_q[31:0]} + {32'b0, retired};
        CounterINSTRETH_q <= CounterINSTRETH_q + {31'b0, CounterINSTRET_q[32]};

        if (~rstn) begin
            CounterCYCLE_q    <= 0;
            CounterCYCLEH_q   <= 0;
            CounterINSTRET_q  <= 0;
            CounterINSTRETH_q <= 0;
        end
    end
endmodule





// Read input Pins (can be used to read buttons and switches)
module CsrPinsIn #(
    parameter [11:0]  BASE_ADDR  = 12'hfc1, // CSR address
    parameter integer COUNT = 4
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input [COUNT-1:0] pins,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | |modify | |wdata;

    wire SelPins_w = (addr == BASE_ADDR);
    reg [31:0] RData_q;
    reg SelPins_q;
    assign rdata = RData_q;
    assign valid = SelPins_w;

    always @(posedge clk) begin
        RData_q <= (SelPins_q ? pins : 32'b0);
        SelPins_q <= SelPins_w;
    end
endmodule





// Write to output pins (can be used to control LEDs)
module CsrPinsOut #(
    parameter [11:0]  BASE_ADDR  = 12'hbc1, // CSR address
    parameter integer COUNT = 4,
    parameter [COUNT-1:0] RESET_VALUE = 'b1
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output [COUNT-1:0] pins,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | |wdata;

    wire SelPins_w = (addr == BASE_ADDR);
    reg [31:0] RData_q;
    reg SelPins_q;
    reg [COUNT-1:0] Pins_q;

    assign rdata = RData_q;
    assign valid = SelPins_w;
    assign pins = Pins_q;

    always @(posedge clk) begin
        // E stage: read CSR
        RData_q <= (SelPins_q ? {{(32-COUNT){1'b0}}, Pins_q} : 32'b0);

        // E stage: write CSR
        if (SelPins_q) begin
            case (modify)
                3'b001: Pins_q <= wdata[COUNT-1:0]; // write
                3'b010: Pins_q <= Pins_q | wdata[COUNT-1:0]; // set
                3'b011: Pins_q <= Pins_q &~ wdata[COUNT-1:0]; // clear
                default: ;
            endcase
        end

        // D stage: select CSR for next cycle
        SelPins_q <= SelPins_w;

        if (~rstn) Pins_q <= RESET_VALUE;
    end
endmodule





// Minimal UART interface for software bitbanging
module CsrUartBitbang #(
    parameter [11:0]  BASE_ADDR  = 12'h7c0,    // CSR address
    parameter integer CLOCK_RATE = 12_000_000,
    parameter integer BAUD_RATE  = 115200
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input rx,
    output tx,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |wdata;

    wire [30:0] PERIOD = CLOCK_RATE / BAUD_RATE;
    wire SelUart_w = (addr==BASE_ADDR);

    reg [31:0] RData_q;
    reg SelUart_q;
    reg TX_q;

    assign rdata = RData_q;
    assign valid = SelUart_w;
    assign tx = TX_q;

    always @(posedge clk) begin
        // E stage: read CSR
        RData_q <= SelUart_q ? {PERIOD, rx} : 32'b0;

        // E stage: write CSR
        if (SelUart_q) begin
            case ({modify, wdata[0]})
                4'b0010: TX_q <= 0; // write 0
                4'b0011: TX_q <= 1; // write 1
                4'b0101: TX_q <= 1; // set
                4'b0111: TX_q <= 0; // clear
            endcase
        end

        // D stage: select CSR for next cycle
        SelUart_q <= SelUart_w;

        if (~rstn) TX_q <= 1;
    end
endmodule





// Character oriented UART interface
module CsrUartChar #(
    parameter [11:0]  BASE_ADDR  = 12'hbc0,    // CSR address
    parameter integer CLOCK_RATE = 12_000_000,
    parameter integer BAUD_RATE  = 115200
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input rx,
    output tx,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |wdata;

    wire SelUart_w = (addr==BASE_ADDR);

    reg [31:0] RData_q;
    assign valid = SelUart_w;
    assign rdata = RData_q;

    localparam integer CLOCK_DIV32 = CLOCK_RATE / BAUD_RATE;
    localparam [15:0] CLOCK_DIV = CLOCK_DIV32[15:0];
        // 16 bit is enough for up to 7.5 GHz (at 115200 baud)
        // two steps to avoid warning from Quartus

    reg  [3:0] q_UartRecvBitCounter;
    reg [15:0] q_UartRecvClkCounter;
    reg  [6:0] q_UartRecvBits;
    reg  [7:0] q_UartRecvChar;
    reg        q_UartRecvEmpty;
    reg        q_RX;
    reg  [3:0] q_UartSendBitCounter;
    reg [15:0] q_UartSendClkCounter;
    reg  [7:0] q_UartSendBits;
    reg        q_TX;
    assign tx = q_TX;

    reg SelUart_q;

    wire UartSendFull = (q_UartSendBitCounter!=0);

    always @(posedge clk) begin
        // E stage: read CSR
        RData_q <= (SelUart_q ? {22'b0, UartSendFull, q_UartRecvEmpty, q_UartRecvChar} : 32'b0);

        // E stage: write CSR
        if (SelUart_q) begin
            case (modify)
                3'b001: begin // write
                    if (!UartSendFull) begin
                        q_TX <= 0;
                        q_UartSendClkCounter <= CLOCK_DIV;
                        q_UartSendBitCounter <= 10;
                        q_UartSendBits <= wdata[7:0];
                    end
                end
                3'b010: begin // set
                    q_UartRecvEmpty <= 1;
                end
                default: ;
            endcase
        end

        // D stage: decode CSR address
        SelUart_q <= SelUart_w;

        // receive
        if (q_UartRecvBitCounter) begin
            if (q_UartRecvClkCounter) begin
                q_UartRecvClkCounter <= q_UartRecvClkCounter - 16'b1;
            end else begin
                q_UartRecvClkCounter <= CLOCK_DIV;
                q_UartRecvBits <= {q_RX, q_UartRecvBits[6:1]};
                if (q_UartRecvBitCounter==2) begin
                    q_UartRecvEmpty <= 0;
                    q_UartRecvChar <= {q_RX, q_UartRecvBits};
                end
                q_UartRecvBitCounter <= q_UartRecvBitCounter - 4'b1;
            end
        end else if (~q_RX) begin
            q_UartRecvClkCounter <= CLOCK_DIV / 16'h2;
            q_UartRecvBitCounter <= 10;
        end
        q_RX <= rx;

        // send
        if (UartSendFull) begin
            if (q_UartSendClkCounter) begin
                q_UartSendClkCounter <= q_UartSendClkCounter - 16'b1;
            end else begin
                q_UartSendClkCounter <= CLOCK_DIV;
                q_TX <= q_UartSendBits[0];
                q_UartSendBits <= {1'b1, q_UartSendBits[7:1]};
                q_UartSendBitCounter <= q_UartSendBitCounter - 4'b1;
            end
        end

        if (!rstn) begin
            q_UartRecvEmpty <= 1;
            q_UartRecvBitCounter <= 0;
            q_UartSendBitCounter <= 0;
            q_TX <= 1;
            q_RX <= 1;
        end
    end
endmodule


// simple timer that raises an interrupt after a specified number of cycles
module CsrTimerAdd #(
    parameter [11:0]  BASE_ADDR  = 12'hbc2,     // CSR address
    parameter integer WIDTH = 32
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output irq,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |wdata;

    reg [WIDTH-1:0] Counter_q;
    reg TimerOn_q;
    reg Request_q;
    assign irq = Request_q;
    assign valid = SelTimer_w;
    assign rdata = 0;

    wire SelTimer_w = (addr==BASE_ADDR);
    reg SelTimer_q;

    always @(posedge clk) begin
        // E stage: write CSR
        if (SelTimer_q) begin
            case (modify)
                3'b001: begin // write: set relative timer
                    TimerOn_q <= 1;
                    Counter_q <= wdata[WIDTH-1:0];
                end
                3'b011: begin // clear with any value disables the timer
                    TimerOn_q <= 0;
                end
                default: ;
            endcase
        end

        // D stage: decode CSR address
        SelTimer_q <= SelTimer_w;

        // increment timer
        if (Counter_q==0) begin
            Request_q <= TimerOn_q;
        end else begin
            Request_q <= 0;
            Counter_q <= Counter_q - 'b1;
        end

        if (!rstn) begin
            TimerOn_q <= 0;
            Counter_q <= 0;
        end
    end
endmodule



// Combines the usally used CSR: ID, Counter, UART, Timer, LEDs
module CsrDefault #(
    parameter [31:0] ISA = 0,
    parameter [31:0] VENDORID = 0,
    parameter [31:0] ARCHID = 0,
    parameter [31:0] IMPID = 0,
    parameter [31:0] HARTID = 0,

    parameter integer OUTPINS_COUNT = 4,
    parameter integer CLOCK_RATE    = 12_000_000,
    parameter integer BAUD_RATE     = 115200,
    parameter integer TIMER_WIDTH   = 32,

    parameter integer CSR_UART  = 12'hBC0,
    parameter integer CSR_LEDS  = 12'hBC1,
    parameter integer CSR_TIMER = 12'hBC2,
    parameter integer CSR_KHZ   = 12'hFC0
) (
    input         clk,
    input         rstn,

    input         read,
    input   [2:0] modify,
    input  [31:0] wdata,
    input  [11:0] addr,
    output [31:0] rdata,
    output        valid,

    input         retired,
    output [OUTPINS_COUNT-1:0] outpins,
    input         rx,
    output        tx,
    output        irq_timer,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | |modify | |wdata;


    wire        IDsValid;
    wire [31:0] IDsRData;
    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        PinsOutValid;
    wire [31:0] PinsOutRData;
    wire        UartValid;
    wire [31:0] UartRData;
    wire        TimerValid;
    wire [31:0] TimerRData;

    assign rdata = IDsRData | CounterRData | UartRData | TimerRData;
    assign valid = IDsValid | CounterValid | UartValid | TimerValid;

    CsrIDs #(
        .ISA(ISA),
        .VENDORID(VENDORID),
        .ARCHID(ARCHID),
        .IMPID(IMPID),
        .HARTID(HARTID),

        .BASE_ADDR(CSR_KHZ),
        .KHZ(CLOCK_RATE/1000)
    ) IDs (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .modify (modify),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (IDsRData),
        .valid  (IDsValid),

        .AVOID_WARNING()
    );

    CsrCounter Counter (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .modify (modify),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (CounterRData),
        .valid  (CounterValid),

        .retired(retired),

        .AVOID_WARNING()
    );

    CsrPinsOut #(
        .BASE_ADDR(CSR_LEDS),
        .COUNT(OUTPINS_COUNT)
    ) Pins (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .modify (modify),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (PinsOutRData),
        .valid  (PinsOutValid),

        .pins   (outpins),

        .AVOID_WARNING()
    );

    CsrUartChar #(
        .BASE_ADDR(CSR_UART),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE)
    ) UART (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .modify (modify),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (UartRData),
        .valid  (UartValid),

        .rx     (rx),
        .tx     (tx),

        .AVOID_WARNING()
    );

    CsrTimerAdd #(
        .BASE_ADDR(CSR_TIMER),
        .WIDTH(32)
    ) Timer (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .modify (modify),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (TimerRData),
        .valid  (TimerValid),

        .irq    (irq_timer),

        .AVOID_WARNING()
    );
endmodule


// SPDX-License-Identifier: ISC
