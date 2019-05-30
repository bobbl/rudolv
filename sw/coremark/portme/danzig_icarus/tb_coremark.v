module tb_coremark;


    reg clk = 1;
    always #5 clk = !clk;

    reg rstn = 0;
    initial begin
        #40 rstn = 1;
    end

    wire mem_wren;
    wire [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;

    Memory mem (
        .clk    (clk),
        .wren   (mem_wren),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[14:2]),
        .rdata  (mem_rdata)
    );

    Pipeline dut (
        .clk            (clk),
        .rstn           (rstn),

        .mem_wren       (mem_wren),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata)
    );


    //always #1000000 $monitor("  time %t", $time);

    integer i;
    always @(posedge clk) begin
        if (mem_wren) begin
            case (mem_addr)
                'h10000000: begin
                    //if (mem_wmask[0]) $write("\033[1;37m%c\033[0m", mem_wdata[7:0]);
                    if (mem_wmask[0]) $write("%c", mem_wdata[7:0]);
                end
                'h10001000: begin
                    if (mem_wmask[0] & mem_wdata[0]) begin
                        //$display("exit simulation (store to 0x10001000)");
                        $finish;
                    end
                end
            endcase
        end
    end

    initial begin
        #100_000_000 $display("***** TIMEOUT"); $stop;
    end

endmodule



// instruction memory
module Memory (
    input clk, 
    input wren,
    input [3:0] wmask,
    input [31:0] wdata,
    input [12:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:8191]; // 32 KiByte

    initial begin
        $readmemh(`CODE, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (wren) begin
            if (wmask[0]) mem[addr][7:0] <= wdata[7:0];
            if (wmask[1]) mem[addr][15:8] <= wdata[15:8];
            if (wmask[2]) mem[addr][23:16] <= wdata[23:16];
            if (wmask[3]) mem[addr][31:24] <= wdata[31:24];
        end
    end
endmodule

