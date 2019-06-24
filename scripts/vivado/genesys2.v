/* wrapper for Digilent Genesys 2 board

Memory map
0000'0000h main memory (BRAM, 8 KiByte)
0000'1E00h start address of boot loader

1000'1000h LEDs (each bit), lowest bit indicates program termination
1000'2000h UART RX
1000'3000h UART TX
1000'4000h UART signal width of one bit in clock cycles
*/


module top (
    input clk_p,
    input clk_n,
    input uart_rx,
    output uart_tx,
    output [7:0] leds
);
    localparam integer CLOCK_RATE = 200_000_000;
    localparam integer BAUD_RATE = 115200;

    wire clk;
    IBUFGDS clk_inst (
        .O(clk),
        .I(clk_p),
        .IB(clk_n)
    );

    reg [5:0] reset_counter = 0;
    wire rstn = &reset_counter;
    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
    end

    reg  [7:0] ff_Leds;
    reg        ff_TX;
    reg        ff_MemWrEn;
    reg [31:0] ff_MemAddr;
    reg [31:0] ff_MemWData;

    wire MemWrEn;
    wire mem_valid;
    wire mem_wren = MemWrEn & ~mem_addr[28];
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;

    reg [31:0] MappedRData;
    always @* begin
        MappedRData <= 32'h0000006f; // avoid that code is executed
        case (ff_MemAddr[15:12])
            4'h2: MappedRData <= {31'b0, uart_rx};
            4'h4: MappedRData <= CLOCK_RATE / BAUD_RATE;
        endcase
    end

    wire [31:0] MemRData = ff_MemAddr[28] ? MappedRData : mem_rdata;

    always @(posedge clk) begin
        if (~rstn) begin
            ff_MemWrEn <= 0;
            ff_MemAddr <= 0;
            ff_Leds <= 0;
            ff_TX <= 1;
        end else begin
            if (ff_MemWrEn & ff_MemAddr[28]) begin
                case (ff_MemAddr[15:12])
                    4'h0: ; // char output: ignored
                    4'h1: ff_Leds <= ff_MemWData[7:0];
                    4'h2: ;
                    4'h3: ff_TX <= ff_MemWData[0];
                endcase
            end

            ff_MemWrEn  <= MemWrEn;
            ff_MemAddr  <= mem_addr;
            ff_MemWData <= mem_wdata;
        end
    end

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

    Pipeline #(
        .START_PC       (32'h_0000_fe00)
    ) pipe (
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
        .mem_write      (MemWrEn),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (MemRData)
    );

    BRAMMemory mem (
        .clk    (clk),
        .wren   (mem_wren),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata)
    );

    assign leds = ff_Leds;
    assign uart_tx = ff_TX;
endmodule


// instruction memory
module BRAMMemory (
    input clk, 
    input wren,
    input [3:0] wmask,
    input [31:0] wdata,
    input [13:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:'h3fff];

    initial begin
        $readmemh("bootloader.hex", mem);
            // bootloader code is the same as on other platforms, but at the
            // beginning there must be '@1e00' to load the code at the correct
            // start adress
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

