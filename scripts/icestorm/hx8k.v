// wrapper for iCE40 HX8K breakout board
module top (
    input clk,
    input uart_rx,
    output uart_tx,
    output [7:0] leds
);
    localparam integer CLOCK_RATE = 12_000_000;
//    localparam integer BAUD_RATE = 115200;
    localparam integer BAUD_RATE = 9600;

    reg [5:0] reset_counter = 0;
    wire rstn = &reset_counter;

    always @(posedge clk) begin
        reset_counter <= reset_counter + !rstn;
    end

    reg        ff_SelMapped;
    reg [31:0] ff_MappedRData;
    reg  [7:0] ff_Leds;
    reg        ff_TX;

    wire MemWrEn;
    wire mem_wren = MemWrEn & mem_addr[28];
    wire  [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire [31:0] MemRData = ff_SelMapped ? ff_MappedRData : mem_rdata;

    always @(posedge clk) begin
        if (~rstn) begin
            ff_SelMapped <= 0;
            ff_MappedRData <= 0;
            ff_Leds <= 0;
            ff_TX <= 0;
        end else begin
            ff_SelMapped <= mem_addr[28];
            ff_MappedRData <= {31'b0, uart_rx};

            ff_MappedRData <= 0;
            case (mem_addr[15:12])
                4'h2: ff_MappedRData <= {31'b0, uart_rx};
                4'h4: ff_MappedRData <= CLOCK_RATE / BAUD_RATE;
            endcase

            if (mem_wren & mem_addr[28]) begin
                case (mem_addr[15:12])
                    4'h0: ; // char output: ignored
                    4'h1: ff_Leds <= mem_wdata[7:0];
                    4'h2: ;
                    4'h3: ff_TX <= mem_wdata[0];
                endcase
            end
        end
    end

    Memory mem (
        .clk    (clk),
        .wren   (mem_wren),
        .wmask  (mem_wmask),
        .wdata  (mem_wdata),
        .addr   (mem_addr[12:2]),
        .rdata  (mem_rdata)
    );

    Pipeline pipe (
        .clk            (clk),
        .rstn           (rstn),

        .mem_wren       (MemWrEn),
        .mem_wmask      (mem_wmask),
        .mem_wdata      (mem_wdata),
        .mem_addr       (mem_addr),
        .mem_rdata      (MemRData)
    );

    assign leds = ff_Leds;
    assign uart_tx = ff_TX;
endmodule


// instruction memory
module Memory (
    input clk, 
    input wren,
    input [3:0] wmask,
    input [31:0] wdata,
    input [10:0] addr,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:2047];

    initial begin
        $readmemh("code.hex", mem);
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

