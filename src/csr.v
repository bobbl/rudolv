// Miscellaneous CSR extensions





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

    reg Valid;
    reg [31:0] RData;

    always @(posedge clk) begin
        Valid <= 1;
        RData <= 0;
        case (addr)
            12'hF11: RData <= VENDORID;
            12'hF12: RData <= ARCHID;
            12'hF13: RData <= IMPID;
            12'hF14: RData <= HARTID;
            BASE_ADDR: RData <= KHZ;
            default: Valid <= 0;
        endcase
    end

    assign valid = Valid;
    assign rdata = RData;
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

    reg Valid;
    reg [31:0] RData;
    reg [32:0] q_CounterCYCLE;
    reg [31:0] q_CounterCYCLEH;
    reg [32:0] q_CounterINSTRET;
    reg [31:0] q_CounterINSTRETH;

    always @(posedge clk) begin
        Valid <= 1;
        RData <= 0;
        case (addr)
            12'hB00: RData <= q_CounterCYCLE[31:0];     // MCYCLE
            12'hB02: RData <= q_CounterINSTRET[31:0];   // MINSTRET
            12'hB80: RData <= q_CounterCYCLEH;          // MCYCLEH
            12'hB82: RData <= q_CounterINSTRETH;        // MINSRETH
            12'hC00: RData <= q_CounterCYCLE[31:0];     // CYCLE
            12'hC80: RData <= q_CounterCYCLEH;          // CYCLEH
            12'hC01: RData <= q_CounterCYCLE[31:0];     // TIME
            12'hC81: RData <= q_CounterCYCLEH;          // TIMEH
            12'hC02: RData <= q_CounterINSTRET[31:0];   // INSTRET
            12'hC82: RData <= q_CounterINSTRETH;        // INSRETH
            default: Valid <= 0;
        endcase

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

    assign valid = Valid;
    assign rdata = RData;
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

    reg Valid;
    reg [31:0] RData;

    always @(posedge clk) begin
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= pins;
        end else begin
            Valid <= 0;
            RData <= 0;
        end
    end

    assign valid = Valid;
    assign rdata = RData;
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
    assign AVOID_WARNING = read;

    reg [COUNT-1:0] q_Pins;
    reg Valid;
    reg [31:0] RData;

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= q_Pins;
            case (modify)
                3'b001: q_Pins <= wdata[COUNT-1:0]; // write
                3'b010: q_Pins <= q_Pins | wdata[COUNT-1:0]; // set
                3'b011: q_Pins <= q_Pins &~ wdata[COUNT-1:0]; // clear
                default: ;
            endcase
        end
        if (~rstn) q_Pins <= RESET_VALUE;
    end

    assign valid = Valid;
    assign rdata = RData;
    assign pins = q_Pins;
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

    reg q_TX;
    reg Valid;
    reg [31:0] RData;

    wire [30:0] PERIOD = CLOCK_RATE / BAUD_RATE;

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= {PERIOD, rx};
            case ({modify, wdata[0]})
                4'b0010: q_TX <= 0; // write 0
                4'b0011: q_TX <= 1; // write 1
                4'b0101: q_TX <= 1; // set
                4'b0111: q_TX <= 0; // clear
            endcase
        end
        if (~rstn) q_TX <= 1;
    end

    assign valid = Valid;
    assign rdata = RData;
    assign tx = q_TX;
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

    localparam [15:0] CLOCK_DIV = CLOCK_RATE / BAUD_RATE;
        // 16 bit is enough for up to 7.5 GHz (at 115200 baud)

    reg Valid;
    reg [31:0] RData;

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


    wire UartSendFull = (q_UartSendBitCounter!=0);

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= {UartSendFull, q_UartRecvEmpty, q_UartRecvChar};
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
            endcase
        end

        q_RX <= rx;
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
        end
    end

    assign valid = Valid;
    assign rdata = RData;
    assign tx = q_TX;
endmodule




// Timer
module CsrTimerAdd #(
    parameter [11:0]  BASE_ADDR  = 12'hbc2,     // CSR address
    parameter integer WIDTH = 16
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


    reg Valid;
    reg [31:0] RData;
    reg [WIDTH-1:0] q_Time;
    reg [WIDTH-1:0] q_TimeCmp;
    reg q_Enable;
    reg q_Request;

    always @(posedge clk) begin
        q_Request <= q_Enable & (q_TimeCmp <= q_Time);

        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= q_Time;
            case (modify)
                3'b001: begin // write
                    q_Enable <= 1;
                    q_TimeCmp <= q_Time + wdata[WIDTH-1:0];
                end
                3'b011: begin // clear
                    q_Enable <= 0;
                end
            endcase
        end

        if (!rstn) begin
            q_Enable <= 0;
            q_Time <= 0;
        end
    end

    assign valid = Valid;
    assign rdata = RData;
    assign irq = q_Request;
endmodule





// SPDX-License-Identifier: ISC
