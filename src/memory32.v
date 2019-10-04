// 32 bit single ported zero latency memory
module Memory32 #(
    parameter WIDTH = 13,
    parameter CONTENT = ""
) (
    input clk, 
    input valid,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
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
        if (valid & write) begin
            if (wmask[0]) mem[addr][7:0] <= wdata[7:0];
            if (wmask[1]) mem[addr][15:8] <= wdata[15:8];
            if (wmask[2]) mem[addr][23:16] <= wdata[23:16];
            if (wmask[3]) mem[addr][31:24] <= wdata[31:24];
        end
    end
endmodule


