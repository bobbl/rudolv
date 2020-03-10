module tb_tests;

    localparam CSR_SIM   = 12'h3FF;
    localparam CSR_UART  = 12'hBC0;
    localparam CSR_LEDS  = 12'hBC1;
    localparam CSR_SWI   = 12'hBC1;
    localparam CSR_TIMER = 12'hBC2;
    localparam CSR_KHZ   = 12'hFC0;

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
    wire mem_wgrubby;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;

    wire mem_rgrubby_from_mem;
`ifdef ENABLE_GRUBBY
    wire mem_rgrubby_to_pipe = mem_rgrubby_from_mem;
`else
    wire mem_rgrubby_to_pipe = 0;
`endif

    wire        IDsValid;
    wire [31:0] IDsRData;
    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        PinsValid;
    wire [31:0] PinsRData;
    wire        TimerValid;
    wire [31:0] TimerRData;

    wire        irq_software;
    wire        irq_timer;
    wire        irq_external = 0;
    wire        retired;

    wire        csr_read;
    wire  [2:0] csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata = IDsRData | CounterRData | PinsRData | TimerRData;
    wire        csr_valid = IDsValid | CounterValid | PinsValid | TimerValid;

    CsrIDs #(
        .BASE_ADDR(CSR_KHZ),
        .KHZ(1000) // assume 1 MHz
    ) csr_ids (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (IDsRData),
        .valid  (IDsValid),

        .AVOID_WARNING()
    );

    CsrCounter counter (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (CounterRData),
        .valid  (CounterValid),

        .retired(retired),

        .AVOID_WARNING()
    );

    CsrPinsOut #(
        .BASE_ADDR(CSR_SWI),
        .COUNT(1),
        .RESET_VALUE(0)
    ) SoftwareInt (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (PinsRData),
        .valid  (PinsValid),

        .pins   (irq_software),

        .AVOID_WARNING()
    );

    CsrTimerAdd #(
        .BASE_ADDR(CSR_TIMER),
        .WIDTH(32)
    ) Timer (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (TimerRData),
        .valid  (TimerValid),

        .irq    (irq_timer),

        .AVOID_WARNING()
    );



    reg q_ReadUART;
    // wire [31:0] CsrRData = q_ReadUART ? 0 : csr_rdata; 
    //   csr_rdata is 0 anyway
    wire CsrValid = q_ReadUART | csr_valid;


    Pipeline dut (
        .clk            (clk),
        .rstn           (rstn),

        .irq_software   (irq_software),
        .irq_timer      (irq_timer),
        .irq_external   (irq_external),
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
        .mem_rgrubby    (mem_rgrubby_to_pipe)
    );



    Memory33Sim #(
        .WIDTH(14), // 4 * (2**14) = 64 KiByte
        .CONTENT(`CODE)
    ) mem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write),
        .wmask  (mem_wmask),
        .wgrubby(mem_wgrubby),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata),
        .rgrubby(mem_rgrubby_from_mem)
    );


`ifdef DEBUG
    always #10 $monitor("  time %t", $time);
`endif


    reg [11:0] q_CsrAddr = 0;
    integer i;
    integer sig_begin;
    integer sig_end;
    always @(posedge clk) begin

        q_ReadUART <= csr_read & (q_CsrAddr==CSR_UART);
        q_CsrAddr  <= csr_addr;

        if (csr_modify==1) begin
            case (q_CsrAddr)
                (CSR_SIM-2): begin
                    sig_begin <= csr_wdata / 4;
                end
                (CSR_SIM-1): begin
                    sig_end <= csr_wdata / 4;
                end
                CSR_SIM: begin
                    case (csr_wdata)
                        2: begin // signature from compliance tests
                            while (sig_begin < sig_end) begin
                                $display("%h", mem.mem[sig_begin][31:0]);
                                sig_begin = sig_begin + 1;
                            end
                        end
                        default: $display("exit due to write to CSR 0x3ff");
                    endcase
                    $finish;
                end
                CSR_UART: begin
`ifdef DEBUG
                    $write("\033[1;37mputchar '%c'\033[0m\n", csr_wdata[7:0]);
`else
                    $write("\033[1;37m%c\033[0m", csr_wdata[7:0]);
                    //$write("%c", csr_wdata[7:0]);
`endif
                end
            endcase
        end
/*
        if (dut.d_Insn == 'h006F) begin
            $display("exit due to write to infinite loop");
            $finish;
        end
*/

    end

    initial begin
`ifdef DEBUG
        #200_000 $write("*** TIMEOUT"); $stop;
`else
        #5_000_001 $write("*** TIMEOUT"); $stop;
`endif
    end


endmodule


// SPDX-License-Identifier: ISC
