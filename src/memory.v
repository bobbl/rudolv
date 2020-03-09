// 8 bit single ported zero latency memory
module Memory8 #(
    parameter ADDR_WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input [7:0] wdata,
    input [ADDR_WIDTH-1:0] addr,
    output reg [7:0] rdata
);
    localparam integer SIZE = 1 << ADDR_WIDTH;

    reg [7:0] mem [0:SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) mem[addr] <= wdata;
    end
endmodule


// 9 bit single ported zero latency memory
module Memory9 #(
    parameter ADDR_WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input write,
    input [8:0] wdata,
    input [ADDR_WIDTH-1:0] addr,
    output reg [8:0] rdata
);
    localparam integer SIZE = 1 << ADDR_WIDTH;

    reg [8:0] mem [0:SIZE-1];

    initial begin
        if (CONTENT != "") $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) mem[addr] <= wdata;
    end
endmodule


// 32 bit plus grubby
module Memory4x9 #(
    parameter ADDR_WIDTH = 14,
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

    wire wgrubby2 = (wmask != 4'b1111) | wgrubby;
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
        .CONTENT(CONTENT_BYTE0)
    ) mem0 (
        .clk    (clk),
        .write  (wren0),
        .wdata  ({wgrubby2, wdata[7:0]}),
        .addr   (addr),
        .rdata  (rbyte0)
    );

    Memory9 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .CONTENT(CONTENT_BYTE1)
    ) mem1 (
        .clk    (clk),
        .write  (wren1),
        .wdata  ({wgrubby2, wdata[15:8]}),
        .addr   (addr),
        .rdata  (rbyte1)
    );

    Memory9 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .CONTENT(CONTENT_BYTE2)
    ) mem2 (
        .clk    (clk),
        .write  (wren2),
        .wdata  ({wgrubby2, wdata[23:16]}),
        .addr   (addr),
        .rdata  (rbyte2)
    );

    Memory9 #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .CONTENT(CONTENT_BYTE3)
    ) mem3 (
        .clk    (clk),
        .write  (wren3),
        .wdata  ({wgrubby2, wdata[31:24]}),
        .addr   (addr),
        .rdata  (rbyte3)
    );
endmodule




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


// 33 bit single ported zero latency memory
// only for simulation: return xxxx if valid not set
module Memory33Sim #(
    parameter WIDTH = 8,
    parameter CONTENT = ""
) (
    input clk, 
    input valid,
    input write,
    input  [3:0] wmask,
    input [31:0] wdata,
    input        wgrubby,
    input [WIDTH-1:0] addr,
    output reg [31:0] rdata,
    output reg        rgrubby
);
    localparam integer SIZE = 1 << WIDTH;

    reg [32:0] mem [0:SIZE-1];

    initial begin
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


// 32 bit single ported zero latency memory
// only for simulation: return xxxx if valid not set
module Memory32Sim #(
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
        rdata <= valid ? mem[addr] : 32'bx;
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
