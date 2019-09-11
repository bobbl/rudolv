/* Connect pipeline with memory that can be infered by 
 * Microsemi IGLOO2 M2GL025 (Future Electronics Creative Board)

Memory map
0000'0000h 64KiB main memory (SPRAM)
0002'0000h  1KiB boot loader (BRAM)

CSR
7c0h       UART
*/

module withmem (
    input clk,
    input rstn,
    input uart_rx,
    output uart_tx,
    output [3:0] leds
);
    localparam integer CLOCK_RATE = 12_000_000;
    localparam integer BAUD_RATE = 115200;
    localparam integer CLOCK_DIV = CLOCK_RATE / BAUD_RATE;

    localparam integer ADDR_UART = 32'h7000_0000;

    reg         irq_timer;
    wire        retired;
    reg [63:0]  mtime;
    reg [63:0]  mtimecmp;

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


    wire        mem_valid;
    wire        mem_write;
    wire        mem_write_code  = mem_write & mem_addr[31] & ~mem_addr[18] & ~mem_addr[15];
    wire        mem_write_code2 = mem_write & mem_addr[31] & ~mem_addr[18] & mem_addr[15];
    wire        mem_write_data  = mem_write & mem_addr[31] & mem_addr[18];
    wire [3:0]  mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire        mem_wgrubby;
    wire [31:0] mem_rdata_code;
    wire [31:0] mem_rdata_code2;
    wire [35:0] mem_rdata_data;
    wire [31:0] mem_rdata_bootrom;

    reg [31:0] q_MemAddr;
    reg [31:0] MemRData;
    always @* casez (q_MemAddr)
        32'h0000_????: MemRData = mem_rdata_bootrom;
        32'h4400_4000: MemRData = mtimecmp[31:0];
        32'h4400_4004: MemRData = mtimecmp[63:32];
        32'h4400_bff8: MemRData = mtime[31:0];
        32'h4400_bffc: MemRData = mtime[63:32];
        ADDR_UART+4:   MemRData = q_UartRecvChar;
        ADDR_UART+16:  MemRData = {30'b0, q_UartRecvFull, q_UartSendBitCounter==0};
        32'b1000_0000_0000_0000_0???_????_????_????: MemRData = mem_rdata_code;
        32'b1000_0000_0000_0000_100?_????_????_????: MemRData = mem_rdata_code2;
        32'b1000_0000_0000_0100_00??_????_????_????: MemRData = mem_rdata_data[31:0];
        default:       MemRData = ~0;
    endcase
    wire MemRGrubby = (q_MemAddr[31:16]==16'h8004) ? mem_rdata_data[32] : 0;
    wire MemWGrubby = (mem_wmask != 4'b1111) | mem_wgrubby;
    wire [35:0] MemWData36 = {3'b0, MemWGrubby, mem_wdata[31:0]};


    Memory32 #(
        .WIDTH(13)  // 32 KiByte
    ) codemem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_code),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[14:2]),
        .rdata  (mem_rdata_code)
    );

    Memory32 #(
        .WIDTH(11)  // 8 KiByte
    ) codemem2 (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_code2),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[12:2]),
        .rdata  (mem_rdata_code2)
    );

    Memory36 #(
        .WIDTH(12)  // 16 KiByte
    ) datamem (
        .clk    (clk),
        .valid  (mem_valid),
        .write  (mem_write_data),
        .wmask  (mem_wmask),
        .wdata  (MemWData36),
        .addr   (mem_addr[13:2]),
        .rdata  (mem_rdata_data)
    );

    ROM32 #(
        .WIDTH(5),
        .CONTENT("miv.hex")
    ) bootrom (
        .clk    (clk),
        .addr   (mem_addr[6:2]),
        .rdata  (mem_rdata_bootrom)
    );




    wire        CounterValid;
    wire [31:0] CounterRData;
    wire        LedsValid;
    wire [31:0] LedsRData;

    wire        csr_read;
    wire [2:0]  csr_modify;
    wire [31:0] csr_wdata;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata = CounterRData | LedsRData;
    wire        csr_valid = CounterValid | LedsValid;

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


    CsrLeds #(
        .COUNT(4)
    ) csr_leds (
        .clk    (clk),
        .rstn   (rstn),

        .read   (csr_read),
        .modify (csr_modify),
        .wdata  (csr_wdata),
        .addr   (csr_addr),
        .rdata  (LedsRData),
        .valid  (LedsValid),

        .leds   (leds)
    );


    Pipeline #(
        .START_PC       (0)
    ) pipe (
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
        .mem_wgrubby    (mem_wgrubby),
        .mem_addr       (mem_addr),
        .mem_rdata      (MemRData),
        .mem_rgrubby    (MemRGrubby)
    );




    always @(posedge clk) begin
        q_MemAddr <= mem_addr;
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
        end
    end

    assign uart_tx = q_UartSendTX;

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
        $readmemh(CONTENT, mem);
    end

    always @(posedge clk) begin
        rdata <= mem[addr];
    end
endmodule



// 32 bit single ported zero latency memory
module Memory32 #(
    parameter WIDTH = 13
) (
    input clk, 
    input valid,
    input write,
    input [3:0] wmask,
    input [31:0] wdata,
    input [WIDTH-1:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:(1<<WIDTH)-1];

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



// 36 bit single ported zero latency memory
module Memory36 #(
    parameter WIDTH = 13
) (
    input clk, 
    input valid,
    input write,
    input [3:0] wmask,
    input [35:0] wdata,
    input [WIDTH-1:0] addr,
    output reg [35:0] rdata
);
    reg [35:0] mem [0:(1<<WIDTH)-1];

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



module CsrLeds #(
    parameter [11:0]  BASE_ADDR  = 12'h7c1, // CSR address
    parameter integer COUNT = 4
) (
    input clk,
    input rstn,

    input read,
    input [2:0] modify,
    input [31:0] wdata,
    input [11:0] addr,
    output [31:0] rdata,
    output valid,

    output [COUNT-1:0] leds,

    output AVOID_WARNING
);
    assign AVOID_WARNING = read | |wdata;

    reg [COUNT-1:0] q_Leds;
    reg Valid;
    reg [31:0] RData;

    always @(posedge clk) begin
        Valid <= 0;
        RData <= 0;
        if (addr==BASE_ADDR) begin
            Valid <= 1;
            RData <= q_Leds;
            case (modify)
                3'b01: q_Leds <= wdata[COUNT-1:0]; // write 0
                3'b10: q_Leds <= q_Leds | wdata[COUNT-1:0]; // set
                3'b11: q_Leds <= q_Leds &~ wdata[COUNT-1:0]; // clear
                default: ;
            endcase
        end
        if (~rstn) q_Leds <= 1;
    end

    assign valid = Valid;
    assign rdata = RData;
    assign leds = q_Leds;
endmodule
