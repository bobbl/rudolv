/* wrapper for iCE40 UP5K MDP board

Memory map
0000'0000h main memory (SPRAM)

0002'0000h boot loader (BRAM)

1000'1000h LEDs (each bit), lowest bit indicates program termination
1000'2000h UART RX
1000'3000h UART TX
1000'4000h UART signal width of one bit in clock cycles
*/

module top (
    input uart_rx,
    output uart_tx
);
    localparam integer CLOCK_RATE = 24_000_000;
    localparam integer BAUD_RATE = 115200;


    wire clk;

    SB_HFOSC OSCInst0(
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF(clk)   );
    defparam OSCInst0.CLKHF_DIV = "0b01"; // 48 MHz / 2

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
    wire mem_wren_main = MemWrEn & ~mem_addr[28] & ~mem_addr[17];
    wire mem_wren_boot = MemWrEn & ~mem_addr[28] & mem_addr[17];
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata_main;
    wire [31:0] mem_rdata_boot;

    reg [31:0] MappedRData;
    always @* begin
        MappedRData <= 32'h0000006f; // avoid that code is executed
        case (ff_MemAddr[15:12])
            4'h2: MappedRData <= {31'b0, uart_rx};
            4'h4: MappedRData <= CLOCK_RATE / BAUD_RATE;
        endcase
    end

    wire [31:0] MemRData = ff_MemAddr[28]
        ? MappedRData
        : (ff_MemAddr[17]
            ? mem_rdata_boot
            : mem_rdata_main);

    always @(posedge clk) begin
        if (~rstn) begin
            ff_MemWrEn <= 0;
            ff_MemAddr <= 0;
            ff_Leds <= 0;
            ff_TX <= 0;
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

    Pipeline #(
        .START_PC       (32'h_0002_0000)
    ) pipe (
        .clk            (clk),
        .rstn           (rstn),

        .mem_wren       (MemWrEn),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (MemRData)
    );

    SPRAMMemory mainmem (
        .clk    (clk),
        .wren   (mem_wren_main),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[15:2]),
        .rdata  (mem_rdata_main)
    );

    BRAMMemory bootmem (
        .clk    (clk),
        .wren   (mem_wren_boot),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[9:2]),
        .rdata  (mem_rdata_boot)
    );



//    assign leds = ff_Leds;
    assign uart_tx = ff_TX;
//    assign flash_ss = 1;
endmodule



module SPRAMMemory (
    input clk, 
    input wren,
    input [3:0] wmask,
    input [31:0] wdata,
    input [13:0] addr,
    output reg [31:0] rdata
);

SB_SPRAM256KA spram_lo(
    .DATAIN     (wdata[15:0]),
    .ADDRESS    (addr),
    .MASKWREN   ({wmask[1], wmask[1], wmask[0], wmask[0]}),
    .WREN       (wren),
    .CHIPSELECT (1'b1),
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
    .WREN       (wren),
    .CHIPSELECT (1'b1),
    .CLOCK      (clk),
    .STANDBY    (1'b0),
    .SLEEP      (1'b0),
    .POWEROFF   (1'b1),
    .DATAOUT    (rdata[31:16])
    );

endmodule


module BRAMMemory (
    input clk, 
    input wren,
    input [3:0] wmask,
    input [31:0] wdata,
    input [7:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:255];

    initial begin
        $readmemh("../../sw/bootloader/bootloader.hex", mem);
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

