module top

`ifdef VERILATOR
(
    input clk,
    input rstn
);
`elsif __ICARUS__
;
    reg clk = 1;
    always #5 clk = !clk;

    reg rstn = 0;
    initial begin
        #40 rstn = 1;
    end
`else
    $display("Unknown simulator");
    $stop;
`endif

    localparam CSR_IRQBOMB = 12'h3F8;
    localparam CSR_SIM     = 12'h3FF;
    localparam CSR_UART    = 12'hBC0;
    localparam CSR_LEDS    = 12'hBC1;
    localparam CSR_SWI     = 12'hBC1;
    localparam CSR_TIMER   = 12'hBC2;
    localparam CSR_SCRATCH = 12'hBC3;
    localparam CSR_ALT_UART = 12'hBC4; // real UART for testing
    localparam CSR_KHZ     = 12'hFC0;

    reg IRQBombEnable = 0;

    wire        mem_valid;
    wire        mem_write;
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;

    wire        IDsValid;
    wire [31:0] IDsRData;
    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        PinsValid;
    wire [31:0] PinsRData;
    wire        ScratchValid;
    wire [31:0] ScratchRData;
    wire        TimerValid;
    wire [31:0] TimerRData;
    wire        UartValid;
    wire [31:0] UartRData;

    wire        tx;
    reg         rx;

    wire        regset_we;
    wire  [5:0] regset_wa;
    wire [31:0] regset_wd;
    wire        regset_wg;
    wire  [5:0] regset_ra1;
    wire  [5:0] regset_ra2;
    wire [31:0] regset_rd1;
    wire        regset_rg1;
    wire [31:0] regset_rd2;
    wire        regset_rg2;

    wire        irq_software;
    wire        irq_timer;
    wire        irq_external = IRQBombEnable;
    wire        retired;

`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_XLEN 32
`ifdef RISCV_FORMAL
(* keep *) wire [`RISCV_FORMAL_NRET                        - 1 : 0] rvfi_valid;
(* keep *) wire [`RISCV_FORMAL_NRET *                 64   - 1 : 0] rvfi_order;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_ILEN   - 1 : 0] rvfi_insn;
(* keep *) wire [`RISCV_FORMAL_NRET                        - 1 : 0] rvfi_trap;
(* keep *) wire [`RISCV_FORMAL_NRET                        - 1 : 0] rvfi_halt;
(* keep *) wire [`RISCV_FORMAL_NRET                        - 1 : 0] rvfi_intr;
(* keep *) wire [`RISCV_FORMAL_NRET *                  2   - 1 : 0] rvfi_mode;
(* keep *) wire [`RISCV_FORMAL_NRET *                  2   - 1 : 0] rvfi_ixl;
(* keep *) wire [`RISCV_FORMAL_NRET *                  5   - 1 : 0] rvfi_rs1_addr;
(* keep *) wire [`RISCV_FORMAL_NRET *                  5   - 1 : 0] rvfi_rs2_addr;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_rs1_rdata;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_rs2_rdata;
(* keep *) wire [`RISCV_FORMAL_NRET *                  5   - 1 : 0] rvfi_rd_addr;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_rd_wdata;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_pc_rdata;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_pc_wdata;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_mem_addr;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN/8 - 1 : 0] rvfi_mem_rmask;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN/8 - 1 : 0] rvfi_mem_wmask;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_mem_rdata;
(* keep *) wire [`RISCV_FORMAL_NRET * `RISCV_FORMAL_XLEN   - 1 : 0] rvfi_mem_wdata;
`endif

    wire        csr_read;
    wire        csr_write;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;

    wire [31:0] csr_rdata = IDsRData | CounterRData | PinsRData | ScratchRData | TimerRData | UartRData;
    wire        csr_valid = IDsValid | CounterValid | PinsValid | ScratchValid | TimerValid | UartValid
        | (csr_addr==CSR_IRQBOMB)
        | (csr_addr==CSR_IRQBOMB+1)
        | (csr_addr==CSR_SIM-2)
        | (csr_addr==CSR_SIM-1)
        | (csr_addr==CSR_SIM)
        | (csr_addr==CSR_UART);


    CsrIDs #(
        .ISA(32'h40001104), // RV32IMC
        .BASE_ADDR(CSR_KHZ),
        .KHZ(10) // assume 10 kHz
    ) csr_ids (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .write  (csr_write),
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
        .write  (csr_write),
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
        .write  (csr_write),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (PinsRData),
        .valid  (PinsValid),

        .pins   (irq_software),

        .AVOID_WARNING()
    );

    CsrScratch #(
        .BASE_ADDR(CSR_SCRATCH)
    ) Scratch (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .write  (csr_write),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (ScratchRData),
        .valid  (ScratchValid),

        .AVOID_WARNING()
    );

    CsrTimerAdd #(
        .BASE_ADDR(CSR_TIMER),
        .WIDTH(32)
    ) Timer (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .write  (csr_write),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (TimerRData),
        .valid  (TimerValid),

        .irq    (irq_timer),

        .AVOID_WARNING()
    );

    CsrUartChar #(
        .BASE_ADDR(CSR_ALT_UART),
        .CLOCK_RATE(10000), // assume 10 kHz
        .BAUD_RATE(2500) // 4 cycles per bit
    ) UART (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .write  (csr_write),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (UartRData),
        .valid  (UartValid),

        .rx     (rx),
        .tx     (tx),

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

`ifdef RISCV_FORMAL
        .rvfi_valid     (rvfi_valid    ),
        .rvfi_order     (rvfi_order    ),
        .rvfi_insn      (rvfi_insn     ),
        .rvfi_trap      (rvfi_trap     ),
        .rvfi_halt      (rvfi_halt     ),
        .rvfi_intr      (rvfi_intr     ),
        .rvfi_mode      (rvfi_mode     ),
        .rvfi_ixl       (rvfi_ixl      ),
        .rvfi_rs1_addr  (rvfi_rs1_addr ),
        .rvfi_rs2_addr  (rvfi_rs2_addr ),
        .rvfi_rs1_rdata (rvfi_rs1_rdata),
        .rvfi_rs2_rdata (rvfi_rs2_rdata),
        .rvfi_rd_addr   (rvfi_rd_addr  ),
        .rvfi_rd_wdata  (rvfi_rd_wdata ),
        .rvfi_pc_rdata  (rvfi_pc_rdata ),
        .rvfi_pc_wdata  (rvfi_pc_wdata ),
        .rvfi_mem_addr  (rvfi_mem_addr ),
        .rvfi_mem_rmask (rvfi_mem_rmask),
        .rvfi_mem_wmask (rvfi_mem_wmask),
        .rvfi_mem_rdata (rvfi_mem_rdata),
        .rvfi_mem_wdata (rvfi_mem_wdata),
`endif

        .csr_read       (csr_read),
        .csr_write      (csr_write),
        .csr_wdata      (csr_wdata),
        .csr_addr       (csr_addr),
        .csr_rdata      (csr_rdata),
        .csr_valid      (CsrValid),

        .mem_valid      (mem_valid),
        .mem_write      (mem_write),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (mem_rdata),

        .regset_we      (regset_we),
        .regset_wa      (regset_wa),
        .regset_wd      (regset_wd),
        .regset_ra1     (regset_ra1),
        .regset_ra2     (regset_ra2),
        .regset_rd1     (regset_rd1),
        .regset_rd2     (regset_rd2)
    );



    Memory32 #(
        .ADDR_WIDTH(14), // 4 * (2**14) = 64 KiByte
        .CONTENT(`CODE)
    ) mem (
        .clk    (clk),
        .write  (mem_write),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata)
    );

    RegSet32 RegSet (
        .clk    (clk),
        .we     (regset_we),
        .wa     (regset_wa),
        .wd     (regset_wd),
        .ra1    (regset_ra1),
        .ra2    (regset_ra2),
        .rd1    (regset_rd1),
        .rd2    (regset_rd2)
    );




    integer i;
    integer sig_begin;
    integer sig_end;
    time irqbomb_marker = 2000000000;
    reg uart_active = 0;

    always @(posedge clk) begin

`ifdef DEBUG
`ifdef __ICARUS__
        $monitor("  time %t", $time);
`elsif VERILATOR
        $display("  time %t", $time);
`endif
`endif

`ifdef IRQBOMB
        if ($time == 10*`IRQBOMB) begin
            $display("IRQBOMB request in cycle %0d", $time/10);
            IRQBombEnable <= 1;
        end
`endif

`ifdef RISCV_FORMAL
        if (rvfi_valid) begin
            $display("%0d %0d %x %x %x x%0d=%x x%0d=%x x%0d=%x %c%c",
                rvfi_order,
                $time/10,
                rvfi_pc_rdata,
                rvfi_pc_wdata,
                rvfi_insn,

                rvfi_rd_addr,
                rvfi_rd_wdata,
                rvfi_rs1_addr,
                rvfi_rs1_rdata,
                rvfi_rs2_addr,
                rvfi_rs2_rdata,

                rvfi_trap ? "T" : ".",
                rvfi_intr ? "I" : "."
            );
        end
`endif

        if ($time > (irqbomb_marker + 10000)) begin
            $display("***** IRQBOMB 'TIMEOUT 1000 cycles after marker in cycle %0d", $time/10);
            $finish;
        end

        q_ReadUART <= csr_read & (csr_addr==CSR_UART);

        if (csr_write) begin

            case (csr_addr)

                (CSR_IRQBOMB): begin
                    $display("IRQBOMB marker in cycle %0d for %0d cycles",
                        $time/10, csr_wdata);
                    irqbomb_marker <= $time;
                end
                (CSR_IRQBOMB+1): begin
`ifdef IRQBOMB
                    $display("IRQBOMB response in cycle %0d response time: %0d",
                        $time/10, $time/10 - `IRQBOMB);
`endif
                    IRQBombEnable <= 0;
                end


                (CSR_SIM-2): begin
                    sig_begin <= csr_wdata / 4;
                end
                (CSR_SIM-1): begin
                    sig_end <= csr_wdata / 4;
                end
                CSR_SIM: begin
                    case (csr_wdata)
                        2: begin // signature from compliance tests
`ifndef RISCV_FORMAL
                            i = sig_begin;
                            while (i < sig_end) begin
                                $display("%h", mem.mem[i][31:0]);
                                i = i + 1;
                            end
`endif
                        end
                        default: $display("exit after %0d cycles due to write to CSR 0x3ff",
                                          $time/10);
                    endcase
                    $finish;
                end
                CSR_UART: begin
`ifdef DEBUG
                    $write("\033[1;37mputchar '%c'\033[0m\n", csr_wdata[7:0]);
`else
                    //$write("\033[1;37m%c\033[0m", csr_wdata[7:0]);
                    $write("%c", csr_wdata[7:0]);
`endif
                end
            endcase
        end

//`ifdef UART_TEST
        uart_active = uart_active | ~tx;
//        if (uart_active) begin
//            $write("UART \033[1;37mTX %b RX %b\033[0m\n", tx, rx);
/*
            $display("U TX=%b SCC=%h SBits=%b SBC=%h SF=%b RX=%b RCC=%h RBits=%b RBC=%h RC=%h RE=%b",
                tx,
                UART.q_UartSendClkCounter,
                UART.q_UartSendBits,
                UART.q_UartSendBitCounter,
                UART.UartSendFull,
                rx,
                UART.q_UartRecvClkCounter,
                UART.q_UartRecvBits,
                UART.q_UartRecvBitCounter,
                UART.q_UartRecvChar,
                UART.q_UartRecvEmpty
                );
*/
//        end
//`endif

        if (dut.Insn_d == 'h006F && dut.PC_q==dut.FetchAddr_q+8) begin
            $display("exit due to infinite loop");
            $finish;
        end
    end

`ifdef __ICARUS__
    initial begin
        rx = 1;
        #20000 rx = 0;
        #50    rx = 1;
        #50    rx = 0;
        #50    rx = 0;
        #50    rx = 0;
        #50    rx = 0;
        #50    rx = 1;
        #50    rx = 1;
        #50    rx = 0;
        #50    rx = 1;
        #50    rx = 1;
        #50    rx = 1;

`ifdef DEBUG
        #200_000 $write("*** TIMEOUT"); $stop;
`else
        #5_000_001 $write("*** TIMEOUT"); $stop;
`endif
    end
`endif





/* memory-mapped Microsemi Mi-V compatibility


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
        32'h8004_????: MemRData = mem_rdata_ram[31:0];
        default:       MemRData = ~0;
    endcase

    always @(posedge clk) begin
        if (mem_valid & mem_write) begin
            case (mem_addr)
                'h4400_4000: begin // mtimecmp
                    if (mem_wmask[0] & mem_wmask[1] & mem_wmask[2] & mem_wmask[3]) begin
                        mtimecmp[31:0] <= mem_wdata;
                    end
                end
                'h4400_4004: begin // mtimecmp
                    if (mem_wmask[0] & mem_wmask[1] & mem_wmask[2] & mem_wmask[3]) begin
                        mtimecmp[63:32] <= mem_wdata;
                    end
                end
                'h4400_bff8: begin // mtime
                    if (mem_wmask[0] & mem_wmask[1] & mem_wmask[2] & mem_wmask[3]) begin
                        mtime[31:0] <= mem_wdata;
                    end
                end
                'h4400_bffc: begin // mtime
                    if (mem_wmask[0] & mem_wmask[1] & mem_wmask[2] & mem_wmask[3]) begin
                        mtime[63:32] <= mem_wdata;
                    end
                end

                'h7000_0000: begin // uart_tx_char
                    if (mem_wmask[0])
`ifdef DEBUG
                     $display("\033[1;35m  putchar '%c'\033[0m", mem_wdata[7:0]);
`else
                     $write("\033[1;37m%c\033[0m", mem_wdata[7:0]);
`endif
                end

            endcase
        end
    end
*/





endmodule


// SPDX-License-Identifier: ISC
