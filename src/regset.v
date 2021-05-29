/* Architecture specific register set implementation for RudolV
 *
 * All have the same interface with grubby bit, but not all support it.
 */


// BRAM with preinit for x0, no grubby bit
// Lattice
module RegSet32(
    input             clk,
    input             we,
    input       [5:0] wa,
    input      [31:0] wd,
    input       [5:0] ra1,
    input       [5:0] ra2,
    output reg [31:0] rd1,
    output reg [31:0] rd2
);
    reg [31:0] regs [0:63];

    initial begin
        regs[0] = 0;
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= regs[ra1];
        rd2 <= regs[ra2];
    end
endmodule


// BRAM with preinit for x0, parity bit for grubby
// Xilinx, Icarus
module RegSet33(
    input             clk,
    input             we,
    input       [5:0] wa,
    input      [31:0] wd,
    input             wg,
    input       [5:0] ra1,
    input       [5:0] ra2,
    output reg [31:0] rd1,
    output reg        rg1,
    output reg [31:0] rd2,
    output reg        rg2
);
    reg [32:0] regs [0:63];

    initial begin
        regs[0] = 0;
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= {wg, wd};
        rd1 <= regs[ra1][31:0];
        rg1 <= regs[ra1][32];
        rd2 <= regs[ra2][31:0];
        rg2 <= regs[ra2][32];
    end
endmodule


// BRAM with preinit for x0, no parity bit
// separate BRAM for grubby bit
// Lattice
module RegSet32g1(
    input             clk,
    input             we,
    input       [5:0] wa,
    input      [31:0] wd,
    input             wg,
    input       [5:0] ra1,
    input       [5:0] ra2,
    output reg [31:0] rd1,
    output reg        rg1,
    output reg [31:0] rd2,
    output reg        rg2
);
    reg [31:0] regs [0:63];
    reg grubby[0:63];

    initial begin
        regs[0] = 0;
        grubby[0]  = 0;
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= wd;
        rd1 <= regs[ra1];
        rd2 <= regs[ra2];
    end

    always @(posedge clk) begin
        if (we) grubby[wa] <= wg;
        rg1 <= grubby[ra1];
        rg2 <= grubby[ra2];
    end
endmodule


module HelperRegSet36z(
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


// BRAM without preinit, parity bit for grubby
// Microsemi
module RegSet36Microsemi(
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

    HelperRegSet36z regs (
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


// SPDX-License-Identifier: ISC
