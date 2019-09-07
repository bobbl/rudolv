/* MiV-compatible image for iCE40 UP5K MDP board

Memory map
0000'0000h 64KiB main memory (SPRAM)
0002'0000h  1KiB boot loader (BRAM)

CSR
7c0h       UART
*/

module top (
    input uart_rx,
    output uart_tx
);
//    localparam integer CLOCK_RATE = 24_000_000;
    localparam integer CLOCK_RATE = 12_000_000;
    localparam integer BAUD_RATE = 115200;
    localparam integer CLOCK_DIV = CLOCK_RATE / BAUD_RATE;

    localparam integer ADDR_UART = 32'h7000_0000;

    wire clk;

    SB_HFOSC OSCInst0(
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF(clk)   );
//    defparam OSCInst0.CLKHF_DIV = "0b01"; // 48 MHz / 2 = 24 MHz
    defparam OSCInst0.CLKHF_DIV = "0b10"; // 48 MHz / 4 = 12 MHz

    reg [5:0] reset_counter = 0;
    wire rstn = &reset_counter;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
    end

    wire        retired;
    reg         irq_timer;
    reg  [63:0] mtime;
    reg  [63:0] mtimecmp;



    wire        mem_valid;
    wire        mem_write;
    wire        mem_write_code = mem_write & mem_addr[31] & ~mem_addr[18];
    wire        mem_write_data = mem_write & mem_addr[31] & mem_addr[18];
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_code;
    wire [31:0] mem_rdata_data;
    wire [31:0] mem_rdata_bootrom;

    reg [31:0] q_MemAddr;
    reg [31:0] MemRData;
    always @* casez (q_MemAddr)
//        32'h0000_????: MemRData = mem_rdata_main;
//        32'h0002_????: MemRData = mem_rdata_boot;
        32'h0000_????: MemRData = mem_rdata_bootrom;
        32'h4400_4000: MemRData = mtimecmp[31:0];
        32'h4400_4004: MemRData = mtimecmp[63:32];
        32'h4400_bff8: MemRData = mtime[31:0];
        32'h4400_bffc: MemRData = mtime[63:32];
        ADDR_UART+4:   MemRData = q_UartRecvChar;
        ADDR_UART+16:  MemRData = {30'b0, q_UartRecvFull, q_UartSendBitCounter==0};
        32'h8000_????: MemRData = mem_rdata_code;
//        32'h8001_????: MemRData = mem_rdata_rom;
        32'h8004_????: MemRData = mem_rdata_data;
        default:       MemRData = ~0;
    endcase

    // uart
    reg  [3:0] q_UartRecvBitCounter;
    reg [15:0] q_UartRecvClkCounter;
    reg  [7:0] q_UartRecvBits;
    reg  [7:0] q_UartRecvChar;
    reg        q_UartRecvFull;
    reg        q_UartRecvRX;
    reg  [3:0] q_UartSendBitCounter;
    reg [15:0] q_UartSendClkCounter;
    reg  [7:0] q_UartSendBits;
    reg        q_UartSendTX;

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

                ADDR_UART: begin // UART send char
                    if (mem_wmask[0]) begin
                        q_UartSendTX <= 0;
                        q_UartSendClkCounter <= CLOCK_RATE / BAUD_RATE;
                        q_UartSendBitCounter <= 10;
                        q_UartSendBits <= mem_wdata[7:0];
                    end
                end
            endcase
        end

        irq_timer <= (mtime >= mtimecmp);
        mtime <= mtime + 1;


        if (mem_valid & ~mem_write & (mem_addr==ADDR_UART+4)) begin
            q_UartRecvFull <= 0; // clear flag if char is read
        end

        q_UartRecvRX <= uart_rx;
        if (q_UartRecvBitCounter) begin
            if (q_UartRecvClkCounter) begin
                q_UartRecvClkCounter <= q_UartRecvClkCounter - 1;
            end else begin
                q_UartRecvClkCounter <= CLOCK_DIV;
                q_UartRecvBits <= {q_UartRecvRX, q_UartRecvBits[7:1]};
/*
                if (q_UartRecvBitCounter==1) begin
                    q_UartRecvFull <= 1;
                    q_UartRecvChar <= q_UartRecvBits;
                end
*/
                if (q_UartRecvBitCounter==2) begin
                    q_UartRecvFull <= 1;
                    q_UartRecvChar <= {q_UartRecvRX, q_UartRecvBits[7:1]};
                end
                q_UartRecvBitCounter <= q_UartRecvBitCounter - 1;
            end
        end else if (~q_UartRecvRX) begin
            q_UartRecvClkCounter <= CLOCK_DIV / 2;
            q_UartRecvBitCounter <= 10;
/*
            q_UartRecvBits <= 0;
        end else begin
            q_UartRecvClkCounter <= 0;
            q_UartRecvBitCounter <= 0;
            q_UartRecvBits <= 0;
*/
        end

        if (q_UartSendBitCounter) begin
            if (q_UartSendClkCounter) begin
                q_UartSendClkCounter <= q_UartSendClkCounter - 1;
            end else begin
                q_UartSendClkCounter <= CLOCK_RATE / BAUD_RATE;
                q_UartSendTX <= q_UartSendBits[0];
                q_UartSendBits <= {1'b1, q_UartSendBits[7:1]};
                q_UartSendBitCounter <= q_UartSendBitCounter - 1;
            end
        end

        if (!rstn) begin
            mtime <= 0;
            mtimecmp <= ~0;

            q_UartRecvFull <= 0;
            q_UartRecvBitCounter <= 0;
            q_UartSendBitCounter <= 0;
            q_UartSendTX <= 1;

/*
            q_UartSendTX <= 1;
            q_UartSendClkCounter <= CLOCK_RATE / BAUD_RATE;
            q_UartSendBitCounter <= 10;
            q_UartSendBits <= 'hff;
*/
        end
    end

    assign uart_tx = q_UartSendTX;


    wire        CounterValid;
    wire [31:0] CounterRData;
//    wire        UartValid;
//    wire [31:0] UartRData;

    wire        csr_read;
    wire [2:0]  csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata = CounterRData; // | UartRData;
    wire        csr_valid = CounterValid; // | UartValid;

    CsrCounter counter (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (CounterRData),
        .valid  (CounterValid),

        .retired(retired)
    );


    Pipeline #(
//        .START_PC       (32'h_0002_0000)
        .START_PC       (32'h_0000_0000)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

        .retired        (retired),
        .irq_timer      (irq_timer),

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

    SPRAMMemory codemem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_code),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_code)
    );

    SPRAMMemory datamem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_data),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_data)
    );


/*

    Memory32 #(
        .WIDTH(11),
        .CONTENT("../../sw/uart/uart.hex")
    ) mainmem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_main),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[12:2]),
        .rdata  (mem_rdata_main)
    );


    BRAMMemory bootmem (
        .clk    (clk),
        .write  (mem_write_boot),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[9:2]),
        .rdata  (mem_rdata_boot)
    );
*/


    ROM32 #(
        .WIDTH(5),
        .CONTENT("../../sw/bootloader/miv.hex")
    ) bootrom (
        .clk    (clk),
        .addr   (mem_addr[6:2]),
        .rdata  (mem_rdata_bootrom)
    );

endmodule



module SPRAMMemory (
    input clk,
    input valid,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [13:0] addr,
    output reg [31:0] rdata
);

SB_SPRAM256KA spram_lo(
    .DATAIN     (wdata[15:0]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[1], wmask[1], wmask[0], wmask[0]}),
    .WREN       (write),
    .CHIPSELECT (valid),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[15:0])
    );

SB_SPRAM256KA spram_hi(
    .DATAIN     (wdata[31:16]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[3], wmask[3], wmask[2], wmask[2]}),
    .WREN       (write),
    .CHIPSELECT (valid),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[31:16])
    );

endmodule


module BRAMMemory (
    input clk, 
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [7:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:255];

    initial begin
        $readmemh("../../sw/bootloader/bootloader.hex", mem);
//        $readmemh("../../sw/bootloader/tiny.hex", mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
        if (write) begin
            if (wmask[0]) mem[addr][7:0] <= wdata[7:0];
            if (wmask[1]) mem[addr][15:8] <= wdata[15:8];
            if (wmask[2]) mem[addr][23:16] <= wdata[23:16];
            if (wmask[3]) mem[addr][31:24] <= wdata[31:24];
        end
    end
endmodule


// 32 bit single ported zero latency memory
module Memory32 #(
    parameter WIDTH = 13,
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





module ROM32 #(
    parameter WIDTH = 13,
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





