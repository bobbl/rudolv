module tb_dhrystone;

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
        .WIDTH(13),
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


    always #1_000_000 $monitor("  time %t", $time);

    integer i;
    always @(posedge clk) begin
        if (mem_valid & mem_write & mem_wmask[0] & 
                (mem_addr=='h1000) & mem_wdata[0]) begin
            $display("tohost (at 0x1000) exit");
            $finish;
        end
        if (mem_valid & mem_write) begin
            case (mem_addr)
                'h1000_0000: begin
                    if (mem_wmask[0]) $write("\033[1;37m%c\033[0m", mem_wdata[7:0]);
                end
                'h1000_1000: begin
                    if (mem_wmask[0] & mem_wdata[0]) begin
                        $display("tohost (at 0x10001000) exit");
                        $finish;
                    end
                end
            endcase
        end
    end

    initial begin
        #5_000_001 $display("***** TIMEOUT"); $stop;
    end

endmodule
