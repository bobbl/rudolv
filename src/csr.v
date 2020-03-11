/* CSR interface
 *
 * D stage: Set (registered) internal enable signals by decoding `addr`
 * E stage: Set `valid` and `rdata` according to enable signals.
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

    reg q_Valid;
    reg [31:0] q_RData;
    assign valid = q_Valid;
    assign rdata = q_RData;

    reg q_enVendor;
    reg q_enArch;
    reg q_enImp;
    reg q_enHart;
    reg q_enKHz;

    always @(posedge clk) begin
        // E stage: read CSR
        q_Valid <= q_enVendor | q_enArch | q_enImp | q_enHart | q_enKHz;
        q_RData <= (q_enVendor ? VENDORID : 32'b0)
                 | (q_enArch   ? ARCHID   : 32'b0)
                 | (q_enImp    ? IMPID    : 32'b0)
                 | (q_enHart   ? HARTID   : 32'b0)
                 | (q_enKHz    ? KHZ      : 32'b0);

        // D stage: decode CSR address
        q_enVendor <= (addr == 12'hF11);
        q_enArch   <= (addr == 12'hF12);
        q_enImp    <= (addr == 12'hF13);
        q_enHart   <= (addr == 12'hF14);
        q_enKHz    <= (addr == BASE_ADDR);
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

    reg q_Valid;
    reg [31:0] q_RData;
    assign valid = q_Valid;
    assign rdata = q_RData;

    reg q_enCycleL;
    reg q_enCycleH;
    reg q_enInstRetL;
    reg q_enInstRetH;

    reg [32:0] q_CounterCYCLE;
    reg [31:0] q_CounterCYCLEH;
    reg [32:0] q_CounterINSTRET;
    reg [31:0] q_CounterINSTRETH;

    always @(posedge clk) begin
        // E stage: read CSR
        q_Valid <= q_enCycleL | q_enCycleH | q_enInstRetL | q_enInstRetH;
        q_RData <= (q_enCycleL   ? q_CounterCYCLE[31:0]   : 32'b0)
                 | (q_enCycleH   ? q_CounterCYCLEH        : 32'b0)
                 | (q_enInstRetL ? q_CounterINSTRET[31:0] : 32'b0)
                 | (q_enInstRetH ? q_CounterINSTRETH      : 32'b0);

        // D stage: decode CSR address
        q_enCycleL   <= (addr == 12'hB00) |     // MCYCLE
                        (addr == 12'hC00) |     // CYCLE
                        (addr == 12'hC01);      // TIME
        q_enCycleH   <= (addr == 12'hB80) |     // MCYCLEH
                        (addr == 12'hC80) |     // CYCLEH
                        (addr == 12'hC81);      // TIMEH
        q_enInstRetL <= (addr == 12'hB02) |     // MINSTRET
                        (addr == 12'hC02);      // INSTRET
        q_enInstRetH <= (addr == 12'hB82) |     // MINSTRETH
                        (addr == 12'hC82);      // INSRETH

        // increment counters (spread over two cycles)
        q_CounterCYCLE    <= {1'b0, q_CounterCYCLE[31:0]} + 1;
        q_CounterCYCLEH   <= q_CounterCYCLEH + q_CounterCYCLE[32];
        q_CounterINSTRET  <= {1'b0, q_CounterINSTRET[31:0]} + {32'b0, retired};
        q_CounterINSTRETH <= q_CounterINSTRETH + q_CounterINSTRET[32];

        if (~rstn) begin
            q_CounterCYCLE    <= 0;
            q_CounterCYCLEH   <= 0;
            q_CounterINSTRET  <= 0;
            q_CounterINSTRETH <= 0;
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

    reg q_Valid;
    reg [31:0] q_RData;
    assign valid = q_Valid;
    assign rdata = q_RData;
    reg q_enPins;

    always @(posedge clk) begin
        q_enPins <= (addr == BASE_ADDR);
        q_Valid <= q_enPins;
        q_RData <= (q_enPins ? pins : 32'b0);
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

    reg q_Valid;
    reg [31:0] q_RData;
    assign valid = q_Valid;
    assign rdata = q_RData;

    reg [COUNT-1:0] q_Pins;
    assign pins = q_Pins;

    reg q_enPins;

    always @(posedge clk) begin
        // E stage: read CSR
        q_Valid <= q_enPins;
        q_RData <= (q_enPins ? q_Pins : 32'b0);

        // E stage: write CSR
        if (q_enPins) begin
            case (modify)
                3'b001: q_Pins <= wdata[COUNT-1:0]; // write
                3'b010: q_Pins <= q_Pins | wdata[COUNT-1:0]; // set
                3'b011: q_Pins <= q_Pins &~ wdata[COUNT-1:0]; // clear
                default: ;
            endcase
        end

        // D stage: decode CSR address
        q_enPins <= (addr == BASE_ADDR);

        //if (~rstn) q_Pins <= RESET_VALUE;
        // not synthesisable with Quartus
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

    reg q_Valid;
    reg [31:0] q_RData;
    assign valid = q_Valid;
    assign rdata = q_RData;

    reg q_enUart;

    reg q_TX;
    assign tx = q_TX;

    wire [30:0] PERIOD = CLOCK_RATE / BAUD_RATE;

    always @(posedge clk) begin
        // E stage: read CSR
        q_Valid <= q_enUart;
        q_RData <= (q_enUart ? {PERIOD, rx} : 32'b0);

        // E stage: write CSR
        if (q_enUart) begin
            case ({modify, wdata[0]})
                4'b0010: q_TX <= 0; // write 0
                4'b0011: q_TX <= 1; // write 1
                4'b0101: q_TX <= 1; // set
                4'b0111: q_TX <= 0; // clear
            endcase
        end

        // D stage: decode CSR address
        q_enUart <= (addr==BASE_ADDR);

        if (~rstn) q_TX <= 1;
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

    reg q_Valid;
    reg [31:0] q_RData;
    assign valid = q_Valid;
    assign rdata = q_RData;

    localparam [15:0] CLOCK_DIV = CLOCK_RATE / BAUD_RATE;
        // 16 bit is enough for up to 7.5 GHz (at 115200 baud)

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

    reg q_enUart;

    wire UartSendFull = (q_UartSendBitCounter!=0);

    always @(posedge clk) begin
        // E stage: read CSR
        q_Valid <= q_enUart;
        q_RData <= (q_enUart ? {22'b0, UartSendFull, q_UartRecvEmpty, q_UartRecvChar} : 32'b0);

        // E stage: write CSR
        if (q_enUart) begin
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
        q_enUart <= (addr==BASE_ADDR);

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

    reg [WIDTH-1:0] q_Counter;
    reg q_TimerOn;
    reg q_Request;
    assign irq = q_Request;
    assign valid = 0;
    assign rdata = 0;

    reg q_enTimer;

    always @(posedge clk) begin
        // E stage: write CSR
        if (q_enTimer) begin
            case (modify)
                3'b001: begin // write: set relative timer
                    q_TimerOn <= 1;
                    q_Counter <= wdata[WIDTH-1:0];
                end
                3'b011: begin // clear with any value disables the timer
                    q_TimerOn <= 0;
                end
            endcase
        end

        // D stage: decode CSR address
        q_enTimer <= (addr==BASE_ADDR);

        // increment timer
        if (q_Counter==0) begin
            q_Request <= q_TimerOn;
        end else begin
            q_Request <= 0;
            q_Counter <= q_Counter - 'b1;
        end

        if (!rstn) begin
            q_TimerOn <= 0;
            q_Counter <= 0;
        end
    end
endmodule



// Combines the usally used CSR: ID, Counter, UART, Timer, LEDs
module CsrDefault #(
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
