/* Architecture specific memory implementation for RudolV
 *
 * All memories are single ported with zero latency
 */

// 1 bit
module Memory1 #(
    parameter ADDR_WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input wdata,
    input [ADDR_WIDTH-1:0] addr,
    output reg rdata
);
    localparam integer SIZE = 1 << ADDR_WIDTH;

    reg mem [0:SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) mem[addr] <= wdata;
    end
endmodule


// 8 bit
module Memory8 #(
    parameter ADDR_WIDTH = 8,
    parameter SIZE = 0,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input [7:0] wdata,
    input [ADDR_WIDTH-1:0] addr,
    output reg [7:0] rdata
);
    localparam integer COMPUTED_SIZE = (SIZE==0) ? (1 << ADDR_WIDTH) : SIZE;

    reg [7:0] mem [0:COMPUTED_SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) mem[addr] <= wdata;
    end
endmodule


// 9 bit
module Memory9 #(
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
        rdata <= mem[addr];
        if (write) mem[addr] <= wdata;
    end
endmodule


// 32 bit plus 4 parity bits used for grubby, byte enable
// Microsemi, Vivado
module Memory4x9 #(
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

    Memory9 #(
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

    Memory9 #(
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

    Memory9 #(
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

    Memory9 #(
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


// 32 bit with byte enable, no grubby
module Memory32 #(
    parameter ADDR_WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [ADDR_WIDTH-1:0] addr,
    output reg [31:0] rdata
);
    localparam integer SIZE = 1 << ADDR_WIDTH;

    reg [31:0] mem [0:SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
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


// 33 bit, MSB used for grubby, byte enable
// For simulation: return XXX if not valid
// Icarus
module Memory33 #(
    parameter ADDR_WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input valid,
    input write,
    input  [3:0] wmask,
    input [31:0] wdata,
    input        wgrubby,
    input [ADDR_WIDTH-1:0] addr,
    output reg [31:0] rdata,
    output reg        rgrubby
);
    localparam integer SIZE = 1 << ADDR_WIDTH;

    reg [32:0] mem [0:SIZE-1];

    initial begin
        /* verilator lint_off WIDTH */
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= valid ? mem[addr][31:0] : 32'bx;
        rgrubby <= valid ? mem[addr][32] : 1'bx;
        if (valid & write) begin
            if (wmask[0]) mem[addr][7:0] <= wdata[7:0];
            if (wmask[1]) mem[addr][15:8] <= wdata[15:8];
            if (wmask[2]) mem[addr][23:16] <= wdata[23:16];
            if (wmask[3]) mem[addr][31:24] <= wdata[31:24];
            if (wmask != 0) mem[addr][32] <= (wmask != 4'b1111) | wgrubby;
        end
    end
endmodule


// 32 bit read-only memory
// Microsemi (BRAM cannot be pre-initialised)
module ROM32 #(
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


// SPDX-License-Identifier: ISC
