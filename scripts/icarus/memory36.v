// 36 bit single ported zero latency memory
module Memory36 #(
    parameter WIDTH = 13,
    parameter CONTENT = ""
) (
    input clk, 
    input valid,
    input write,
    input [3:0] wmask,
    input [35:0] wdata,
    input [WIDTH-1:0] addr,
    output reg [35:0] rdata
);
    localparam integer SIZE = 1 << WIDTH;

    reg [35:0] mem [0:SIZE-1];

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
            mem[addr][35:32] <= wdata[35:32];
        end
    end
endmodule


