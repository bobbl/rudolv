/* Register set implementation for RudolV
 *
 * with grubby bit
 * for BRAM without preinit
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

    always @(posedge clk) begin
        if (we) regs[wa] <= {3'b0, wg, wd};
        rd1 <= ra1 ? regs[ra1][31:0] : 0;
        rg1 <= ra1 ? regs[ra1][32] : 0;
        rd2 <= ra2 ? regs[ra2][31:0] : 0;
        rg2 <= ra1 ? regs[ra2][32] : 0;
    end
endmodule
