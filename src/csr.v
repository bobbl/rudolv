/* CSR interface
 *
 * `addr` is valid one cycle earlier that the other signals and stable for 5
 * cycles. Thus, address decoding is separated from the actual read or write
 * action. If `addr` holds an CSR address that is supported by the extension,
 * the `valid` must be set in the following cycle. The latter is used to detect
 * illegal CSR numbers.
 *
 * In the second cycle, `read` is set if the CSR value should be read. If a
 * valid register is selected by `addr` and `read` is high, `rdata` should be
 * set to the value of the register in the following cycle. It is only asserted
 * for one cycle, otherwise it must be cleared to 0.
 *
 * If the CSR should be modified, `write` is set in the sixth cycle and `wdata`
 * holds the new value for the CSR. The bit set/clear operation is done inside
 * the pipeline and integrated in `wdata`.
 *
 *
 *                   /-------------------------------------\
 *  addr   ~+~~~~~~~X                                       X~~~~~~~+~
 *                   \-------------------------------------/
 *
 *                          +-------+
 *  read                    |       |
 *         -+-------+-------+       +-------+-------+-------+-------+-
 *
 *                                                  +-------+
 *  write                                           |       |
 *         -+-------+-------+-------+-------+-------+       +-------+-
 *
 *                                                   /-----\
 *  wdata  ~+~~~~~~~+~~~~~~~+~~~~~~~+~~~~~~~+~~~~~~~X       X~~~~~~~+~
 *                                                   \-----/
 *
 *                          +-------+-------+-------+-------+-------+
 *  valid                   |                                       |
 *         -+-------+-------+                                       +-
 *
 *                                   /-----\
 *  rdata  -+-------+-------+-------X       X-------+-------+-------+-
 *                                   \-----/
 *
 */




// Machine IDs (from priv spec) and clock frequency (RudolV extension)
module CsrIDs #(
    parameter [31:0] ISA = 0,
    parameter [31:0] VENDORID = 0,
    parameter [31:0] ARCHID = 23, // official RudolV marchid
    parameter [31:0] IMPID = 0,
    parameter [31:0] HARTID = 0,

    parameter [11:0] BASE_ADDR = 12'hfc0, // CSR address for clock frequency
    parameter [31:0] KHZ = 100_000
) (
    input clk,
    input rstn,

    input read,
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | write | |wdata;

    reg Valid_q;
    reg [31:0] RData_q;
    assign valid = Valid_q;
    assign rdata = RData_q;

    // asynchronous in D stage: decode CSR address
    wire SelISA_w    = (addr == 12'h301);
    wire SelVendor_w = (addr == 12'hF11);
    wire SelArch_w   = (addr == 12'hF12);
    wire SelImp_w    = (addr == 12'hF13);
    wire SelHart_w   = (addr == 12'hF14);
    wire SelKHz_w    = (addr == BASE_ADDR);

    always @(posedge clk) begin
        RData_q <= (SelISA_w    ? ISA      : 32'b0)
                 | (SelVendor_w ? VENDORID : 32'b0)
                 | (SelArch_w   ? ARCHID   : 32'b0)
                 | (SelImp_w    ? IMPID    : 32'b0)
                 | (SelHart_w   ? HARTID   : 32'b0)
                 | (SelKHz_w    ? KHZ      : 32'b0);
        Valid_q <= SelISA_w
                 | SelVendor_w
                 | SelArch_w
                 | SelImp_w
                 | SelHart_w
                 | SelKHz_w;
    end
endmodule




// Minimal set of counters: only one 32 bit cycle counter for
// RDCYCLE, RDTIME and RDINSTRET
//
// To simplify logic, HPMCOUNTER3 also mirrors the counter
//   0C00hex CYCLE        0C80hex 0
//   0C01hex TIME         0C81hex 0
//   0C02hex INSTRET      0C82hex 0
//   0C03hex HPMCOUNTER3  0C83hex 0
module CsrRDCYCLE32 (
    input clk,
    input rstn,

    input read,
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | write | |wdata;

    reg Valid_q;
    reg [31:0] RData_q;
    assign valid = Valid_q;
    assign rdata = RData_q;

    wire Valid_w = (addr[11:8]==4'hC) & (addr[6:2]==0);

    reg [31:0] CounterCYCLE_q;

    always @(posedge clk) begin
        RData_q <= (Valid_w & ~addr[7]) ? CounterCYCLE_q : 32'b0;
        Valid_q <= Valid_w;

        // increment counters (spread over two cycles)
        CounterCYCLE_q    <= rstn ? (CounterCYCLE_q + 1) : 32'b0;
    end
endmodule




// Counters from unprivileged spec (RDCYCLE, RDTIME and RDINSTRET)
module CsrCounter (
    input clk,
    input rstn,

    input read,
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input retired,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | write | |wdata;

    reg Valid_q;
    reg [31:0] RData_q;
    assign valid = Valid_q;
    assign rdata = RData_q;

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

    reg [32:0] CounterCYCLE_q;
    reg [31:0] CounterCYCLEH_q;
    reg [32:0] CounterINSTRET_q;
    reg [31:0] CounterINSTRETH_q;

    always @(posedge clk) begin
        Valid_q <= SelCycleL_w
                 | SelCycleH_w
                 | SelInstRetL_w
                 | SelInstRetH_w;
        RData_q <= (SelCycleL_w   ? CounterCYCLE_q[31:0]   : 32'b0)
                 | (SelCycleH_w   ? CounterCYCLEH_q        : 32'b0)
                 | (SelInstRetL_w ? CounterINSTRET_q[31:0] : 32'b0)
                 | (SelInstRetH_w ? CounterINSTRETH_q      : 32'b0);

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




// Counters from unprileged and privileged spec (MCYCLE and MINSTRET)
module CsrMCYCLE (
    input clk,
    input rstn,

    input read,
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input retired,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | write | |wdata;

    reg Valid_q;
    reg [31:0] RData_q;
    assign valid = Valid_q;
    assign rdata = RData_q;

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

    reg [32:0] CounterCYCLE_q;
    reg [31:0] CounterCYCLEH_q;
    reg [32:0] CounterINSTRET_q;
    reg [31:0] CounterINSTRETH_q;

    always @(posedge clk) begin
        Valid_q <= SelCycleL_w
                 | SelCycleH_w
                 | SelInstRetL_w
                 | SelInstRetH_w;
        RData_q <= (SelCycleL_w   ? CounterCYCLE_q[31:0]   : 32'b0)
                 | (SelCycleH_w   ? CounterCYCLEH_q        : 32'b0)
                 | (SelInstRetL_w ? CounterINSTRET_q[31:0] : 32'b0)
                 | (SelInstRetH_w ? CounterINSTRETH_q      : 32'b0);

        if (write) begin // set and clear not supported
            if (SelCycleL_w) CounterCYCLE_q <= wdata;
            if (SelCycleL_w) CounterCYCLEH_q <= wdata;
            if (SelInstRetL_w) CounterINSTRET_q <= wdata;
            if (SelInstRetH_w) CounterINSTRETH_q <= wdata;

            // The select signals are also valid for the read-only CSRs.
            // Thats no problem, because the processor pipeline throws and
            // exception if these CSR should be written and we never reach
            // this point for a readonly CSR.
        end

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
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    input [COUNT-1:0] pins,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | write | |wdata;

    wire SelPins_w = (addr == BASE_ADDR);

    reg [31:0] RData_q;
    reg SelPins_q;
    assign rdata = RData_q;
    assign valid = SelPins_q;

    always @(posedge clk) begin
        RData_q   <= SelPins_w ? pins : 32'b0;
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
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output [COUNT-1:0] pins,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read | |wdata;

    reg [COUNT-1:0] Pins_q;

    reg [31:0] RData_q;
    reg SelPins_q;
    assign rdata = RData_q;
    assign valid = SelPins_q;
    assign pins = Pins_q;

    wire SelPins_w = (addr == BASE_ADDR);

    always @(posedge clk) begin
        SelPins_q <= SelPins_w;
        RData_q <= (SelPins_w ? {{(32-COUNT){1'b0}}, Pins_q} : 32'b0);

        if (write & SelPins_w) Pins_q <= wdata[COUNT-1:0];
        if (~rstn)             Pins_q <= RESET_VALUE;
    end
endmodule



// Scratch register
module CsrScratch #(
    parameter [11:0]  BASE_ADDR  = 12'hBC3, // CSR address
    parameter [31:0] RESET_VALUE = 0
) (
    input clk,
    input rstn,

    input read,
    input write,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output AVOID_WARNING
);
    assign AVOID_WARNING = rstn | read;

    wire SelScratch_w = (addr == BASE_ADDR);
    reg [31:0] RData_q;
    reg SelScratch_q;
    reg [31:0] Scratch_q;

    assign rdata = RData_q;
    assign valid = SelScratch_q;

    always @(posedge clk) begin
        SelScratch_q <= SelScratch_w;
        RData_q <= (SelScratch_w ? Scratch_q : 32'b0);

        if (write & SelScratch_w) Scratch_q <= wdata;
        if (~rstn)                Scratch_q <= RESET_VALUE;
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
    input write,
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
    assign valid = SelUart_q;
    assign tx = TX_q;

    always @(posedge clk) begin
        SelUart_q <= SelUart_w;
        RData_q <= SelUart_w ? {PERIOD, rx} : 32'b0;

        if (write & SelUart_w) TX_q <= wdata[0];
        if (~rstn)             TX_q <= 1;
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
    input write,
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

    reg SelUart_q;
    reg [31:0] RData_q;
    assign valid = SelUart_q;
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


    wire UartSendFull = (q_UartSendBitCounter!=0);

    always @(posedge clk) begin
        SelUart_q <= SelUart_w;
        RData_q <= (SelUart_w ? {22'b0, UartSendFull, q_UartRecvEmpty, q_UartRecvChar} : 32'b0);


        if (write & SelUart_w) begin
            if (wdata[9] & ~UartSendFull) begin
//`ifdef DEBUG
//    $display("UART send %h", wdata[7:0]);
//`endif
                q_TX <= 0;
                q_UartSendClkCounter <= CLOCK_DIV;
                q_UartSendBitCounter <= 10;
                q_UartSendBits <= wdata[7:0];
            end
            if (wdata[8]) q_UartRecvEmpty <= 1;
        end

/*
        if (write) begin
//`ifdef DEBUG
//    $display("UART send 65 %h", wdata[7:0]);
//`endif
                q_TX <= 0;
                q_UartSendClkCounter <= CLOCK_DIV;
                q_UartSendBitCounter <= 10;
                q_UartSendBits <= 'h65;
            end
*/



        // receive
        if (q_UartRecvBitCounter != 0) begin
            if (q_UartRecvClkCounter != 0) begin
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
            if (q_UartSendClkCounter != 0) begin
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
    input write,
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
    reg SelTimer_q;
    assign irq = Request_q;
    assign valid = SelTimer_q;
    assign rdata = 0;

    wire SelTimer_w = (addr==BASE_ADDR);

    always @(posedge clk) begin
        SelTimer_q <= SelTimer_w;

        if (write & SelTimer_w) begin
            TimerOn_q <= (wdata!=0); // disable by wrtiting 0
            Counter_q <= wdata[WIDTH-1:0];
        end

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
    parameter [31:0] ARCHID = 23,
    parameter [31:0] IMPID = 0,
    parameter [31:0] HARTID = 0,

    parameter integer OUTPINS_COUNT = 4,
    parameter integer CLOCK_RATE    = 12_000_000,
    parameter integer BAUD_RATE     = 115200,
    parameter integer TIMER_WIDTH   = 32,

    parameter integer CSR_UART  = 12'hBC0,
    parameter integer CSR_LEDS  = 12'hBC1,
    parameter integer CSR_TIMER = 12'hBC2,
    parameter integer CSR_SCRATCH = 12'hBC3,
    parameter integer CSR_KHZ   = 12'hFC0
) (
    input         clk,
    input         rstn,

    input         read,
    input         write,
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
    assign AVOID_WARNING = rstn | read | write | |wdata;


    wire        IDsValid;
    wire [31:0] IDsRData;
    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        PinsOutValid;
    wire [31:0] PinsOutRData;
    wire        ScratchValid = 0;
    wire [31:0] ScratchRData = 0;
    wire        UartValid;
    wire [31:0] UartRData;
    wire        TimerValid;
    wire [31:0] TimerRData;

    assign rdata = IDsRData | CounterRData | PinsOutRData | ScratchRData | UartRData | TimerRData;
    assign valid = IDsValid | CounterValid | PinsOutValid | ScratchValid | UartValid | TimerValid;

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
        .write  (write),
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
        .write  (write),
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
        .write  (write),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (PinsOutRData),
        .valid  (PinsOutValid),

        .pins   (outpins),

        .AVOID_WARNING()
    );

/*
    CsrScratch #(
        .BASE_ADDR(CSR_SCRATCH)
    ) Scratch (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .write  (write),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (ScratchRData),
        .valid  (ScratchValid),

        .AVOID_WARNING()
    );
*/

    CsrUartChar #(
        .BASE_ADDR(CSR_UART),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE)
    ) UART (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .write  (write),
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
        .write  (write),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (TimerRData),
        .valid  (TimerValid),

        .irq    (irq_timer),

        .AVOID_WARNING()
    );
endmodule




// Same interface as CsrDefault, but not everything is implemented
module CsrMinimal #(
    parameter [31:0] ISA = 0,
    parameter [31:0] VENDORID = 0,
    parameter [31:0] ARCHID = 23,
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
    input         write,
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
    assign AVOID_WARNING = rstn | read | write | |wdata;


    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        UartValid;
    wire [31:0] UartRData;

    assign rdata = CounterRData | UartRData;
    assign valid = CounterValid | UartValid;

    assign outpins = 0;
    assign irq_timer = 0;

    CsrRDCYCLE32 Counter (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .write  (write),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (CounterRData),
        .valid  (CounterValid),

        .AVOID_WARNING()
    );

    CsrUartBitbang #(
        .BASE_ADDR(CSR_UART),
        .CLOCK_RATE(CLOCK_RATE),
        .BAUD_RATE(BAUD_RATE)
    ) UART (
        .clk    (clk),
        .rstn   (rstn),

        .read   (read),
        .write  (write),
        .wdata  (wdata),
        .addr   (addr),
        .rdata  (UartRData),
        .valid  (UartValid),

        .rx     (rx),
        .tx     (tx),

        .AVOID_WARNING()
    );

endmodule




// SPDX-License-Identifier: ISC
