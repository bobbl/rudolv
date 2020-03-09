/* Register set implementation for RudolV
 *
 * for BRAM with preinit for x0 and parity for grubby bit
 */

module RegisterSet(
    input clk, 
    input we,
    input [5:0] wa,
    input [31:0] wd,
    input wg,
    input [5:0] ra1,
    input [5:0] ra2,
    output reg [31:0] rd1,
    output reg rg1,
    output reg [31:0] rd2,
    output reg rg2
);
    reg [35:0] regs [0:63];

    initial begin
        regs[0] <= 0;
    end

    always @(posedge clk) begin
        if (we) regs[wa] <= {3'b0, wg, wd};
        rd1 <= regs[ra1][31:0];
        rg1 <= regs[ra1][32];
        rd2 <= regs[ra2][31:0];
        rg2 <= regs[ra2][32];
    end

endmodule



