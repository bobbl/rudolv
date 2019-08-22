module tb_femcb;

    reg clk = 1;
    always #5 clk = !clk;

    reg rstn = 0;
    initial begin
        #40 rstn = 1;
    end

    reg irq_timer;
    reg [63:0] mtime;
    reg [63:0] mtimecmp;

    wire mem_valid;
    wire mem_write;
    wire mem_write_rom = 0;
    wire mem_write_ram = mem_write & (mem_addr[31:16]==16'h8004);
    wire [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_rom;
    wire [31:0] mem_rdata_ram;

    reg [31:0] q_MemAddr;
    reg [31:0] MemRData;
    always @* casez (q_MemAddr)
        32'h4400_4000: MemRData = mtimecmp[31:0];
        32'h4400_4004: MemRData = mtimecmp[63:32];
        32'h4400_bff8: MemRData = mtime[31:0];
        32'h4400_bffc: MemRData = mtime[63:32];
        32'h7000_0010: MemRData = 1; // uart ready to send, nothing received
        32'h8000_????: MemRData = mem_rdata_rom;
        32'h8001_????: MemRData = mem_rdata_rom;
        32'h8004_????: MemRData = mem_rdata_ram;
        default:       MemRData = ~0;
    endcase


    Memory32 #(
        .WIDTH(15), // 128K
        .CONTENT(`CODE)
    ) rom (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_rom),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[16:2]),
        .rdata  (mem_rdata_rom)
    );

    Memory32 #(
        .WIDTH(14), // 64K
        .CONTENT(`CODE) // don't care
    ) ram (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_ram),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_ram)
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

    Pipeline #(
        .START_PC(32'h8000_0000)
//        .START_PC(0)
    ) dut (
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
        .mem_rdata      (MemRData)
    );


//    always #1_000_000 $monitor("  time %t", $time);
`ifdef DEBUG
    always #10 $monitor("  time %t", $time);
`endif


    integer i;
    always @(posedge clk) begin
        q_MemAddr <= mem_addr;
        if (mem_valid & mem_write) begin
            case (mem_addr)
                'h4400_4000: begin // mtimecmp
                    if (mem_wmask[0] & mem_wmask[1] & mem_wmask[2] & mem_wmask[3]) begin
                        mtimecmp[31:0] <= mem_wdata;
                    end
                end
                'h4400_bff8: begin // mtime
                    if (mem_wmask[0] & mem_wmask[1] & mem_wmask[2] & mem_wmask[3]) begin
                        mtime[31:0] <= mem_wdata;
                    end
                end

                'h7000_0000: begin // uart_tx_char
                    if (mem_wmask[0])
`ifdef DEBUG
                     $display("\033[1;35m  putchar '%c'\033[0m", mem_wdata[7:0]);
`else
                     $write("\033[1;35m%c\033[0m", mem_wdata[7:0]);
`endif
                end

            endcase
        end

        if (csr_addr==12'hbff && csr_modify!=0) begin
            $display("exit due to write to CSR 0xbff / WFI");
            $finish;
        end

        irq_timer <= (mtime >= mtimecmp);
        mtime <= mtime + 1;

        if (!rstn) begin
            mtime <= 0;
            mtimecmp <= ~0;
        end
    end

    initial begin
        #5_000_001 $display("***** TIMEOUT"); $stop;
    end

endmodule
