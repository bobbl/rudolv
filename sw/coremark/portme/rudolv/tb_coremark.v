module tb_coremark;

    localparam CSR_UART = 12'hbc0;
    localparam CSR_SIM  = 12'h3ff;

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
    wire mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire mem_rgrubby = 0;

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

    reg q_ReadUART;
    // wire [31:0] CsrRData = q_ReadUART ? 0 : csr_rdata; 
    //   csr_rdata is 0 anyway
    wire CsrValid = q_ReadUART | csr_valid;


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
        .csr_valid      (CsrValid),

        .mem_valid      (mem_valid),
        .mem_write      (mem_write),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_wgrubby    (mem_wgrubby),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata),
        .mem_rgrubby    (mem_rgrubby)
    );


    //always #1000000 $monitor("  time %t", $time);

    integer i;
    always @(posedge clk) begin
        q_ReadUART <= csr_read & (csr_addr==CSR_UART);

        if (csr_modify==1) begin
            case (csr_addr)
                CSR_SIM: begin
                    $finish;
                end
                CSR_UART: begin
                    $write("%c", csr_wdata[7:0]);
                end
            endcase
        end
    end

    initial begin
        #300_000_000 $display("***** TIMEOUT"); $stop;
    end


endmodule

