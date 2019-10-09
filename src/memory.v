// 32 bit single ported zero latency memory
module Memory32 #(
    parameter WIDTH = 8,
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


// 36 bit single ported zero latency memory
// Synthesisable, but cannot be pre-initialised because of parity bits
// Use for FPGA
module Memory36 #(
    parameter WIDTH = 8
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

    always @(posedge clk) begin
        rdata <= {mem[addr][35], mem[addr][26], mem[addr][17], mem[addr][8],
                  mem[addr][34:27], mem[addr][25:18], mem[addr][16:9], mem[addr][7:0]};
        if (valid & write) begin
            if (wmask[0]) mem[addr][8:0]   <= {wdata[32], wdata[7:0]};
            if (wmask[1]) mem[addr][17:9]  <= {wdata[33], wdata[15:8]};
            if (wmask[2]) mem[addr][26:18] <= {wdata[34], wdata[23:16]};
            if (wmask[3]) mem[addr][35:27] <= {wdata[35], wdata[31:24]};
        end
    end
endmodule


// 36 bit single ported zero latency memory
// Pre-initialisable, but maybe hard to synthesize
// Use for simulation
module Memory36Content #(
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


// 32 bit read-only memory
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
