module tb_tests;


    reg clk = 1;
    always #5 clk = !clk;

    reg rstn = 0;
    initial begin
        #40 rstn = 1;
    end

    wire irq_timer = 1;

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
    wire [2:0] csr_modify;
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

        .irq_timer      (irq_timer),
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
                for (i=0; i<64; i=i+1) begin
                    $display("%h", mem.mem['h1FC0+i]);
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
        if (mem_write & mem_wmask[0] & (mem_addr==32'h10000000)) begin
`ifdef DEBUG
            $display("\033[1;37mputchar '%c'\033[0m", mem_wdata[7:0]);
`else
            $write("\033[1;37m%c\033[0m", mem_wdata[7:0]);
`endif
        end
    end

    initial begin
        #200000 $write("TIMEOUT"); $stop;
    end


endmodule
