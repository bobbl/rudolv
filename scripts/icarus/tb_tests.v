module tb_tests;


    reg clk = 1;
    always #5 clk = !clk;

    reg rstn = 0;
    initial begin
        #40 rstn = 1;
    end

    wire mem_valid;
    wire mem_write;
    wire [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;

    Memory32 #(
        .WIDTH(13), // 4 * (2**13) = 32 KiByte
        .CONTENT(`CODE)
    ) mem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[14:2]),
        .rdata  (mem_rdata)
    );

    wire csr_read;
    wire [1:0] csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata;
    wire csr_valid;

    CsrCounter counter (
        .clk    (clk),
        .rstn   (rstn),
        .retired(retired),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (csr_rdata),
        .valid  (csr_valid)
    );

    Pipeline dut (
        .clk            (clk),
        .rstn           (rstn),

        .retired        (retired),
        .csr_read       (csr_read),
        .csr_modify     (csr_modify),
        .csr_wdata      (csr_wdata),
        .csr_addr       (csr_addr),
        .csr_rdata      (csr_rdata),
        .csr_valid      (csr_valid),

        .mem_valid      (mem_valid),
        .mem_write      (mem_write),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata)
    );


`ifdef DEBUG
    always #10 $monitor("  time %t", $time);
`endif

/*
    integer i;
    always @(posedge clk) begin
        if (mem_wren & mem_wmask[0] & (mem_addr==32'h10001000)) begin
            if (mem_wdata[7:0]==8'h03) begin
                $write("ok");
            end else begin
                $write("FAILED");
            end
            $finish;
        end
    end
*/

    integer i;
    always @(posedge clk) begin
        if (mem_write & mem_wmask[0] & (mem_addr==32'h10001000)) begin
            if (mem_wdata[7:0]==8'h03) begin
                for (i=0; i<64; i=i+4) begin
                    $display("%h%h%h%h",
                        mem.mem['h1FC0 + i+3],
                        mem.mem['h1FC0 + i+2],
                        mem.mem['h1FC0 + i+1],
                        mem.mem['h1FC0 + i+0]);
                end
            end else begin
                $display("***** Test FAILED");
            end
            $finish;
        end
        if (mem_write & mem_wmask[0] & (mem_addr==32'h00001000)) begin
            $display("tohost (at 0x1000) exit");
            $finish;
        end
    end

    initial begin
//        #200000 $write("TIMEOUT"); $stop;
        #5000_000 $write("TIMEOUT"); $stop;
    end


endmodule // testbench




// instruction memory
module Memory (
    input clk,
    input valid,
    input wren,
    input [3:0] wmask,
    input [31:0] wdata,
    input [12:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:8191];

    initial begin
        $readmemh(`CODE, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (valid & wren) begin
            if (wmask[0]) mem[addr][7:0] <= wdata[7:0];
            if (wmask[1]) mem[addr][15:8] <= wdata[15:8];
            if (wmask[2]) mem[addr][23:16] <= wdata[23:16];
            if (wmask[3]) mem[addr][31:24] <= wdata[31:24];
        end
    end
endmodule

