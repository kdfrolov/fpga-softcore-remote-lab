// icache.v - 4KB, 4-way set-associative, 16-byte lines
module icache #(
    parameter SETS             = 256,  // 4KB / 16B = 256 sets
    parameter ASSOCIATIVITY    = 4,
    parameter LINE_WIDTH_BITS  = 128,
    parameter ADDR_WIDTH_BITS  = 32,
    parameter WORD_WIDTH_BITS  = 32,
)(
    input  wire                       clk,
    input  wire                       rst_n,
    
    // CPU interface (schoolRISCV)
    input  wire [ADDR_WIDTH_BITS-1:0] cpu_addr,
    output reg  [WORD_WIDTH_BITS-1:0] cpu_rdata,
    output reg                        cpu_rdata_valid,
    
    // Cache Bus (UART Agent)
    output wire  [ADDR_WIDTH_BITS-1:0] bus_addr,
    output wire                        bus_addr_valid,
    input  wire [LINE_WIDTH_BITS-1:0]  bus_rdata,
    input  wire                        bus_rdata_valid
);

    localparam LINE_WIDTH_BYTES = LINE_WIDTH_BITS / 8;
    localparam SET_INDEX_WIDTH  = $clog2(SETS);
    localparam OFFSET_WIDTH     = $clog2(LINE_WIDTH_BYTES);
    localparam TAG_WIDTH        = ADDR_WIDTH_BITS - SET_INDEX_WIDTH - OFFSET_WIDTH;
    localparam PLRU_WIDTH       = $clog2(ASSOCIATIVITY);

    localparam WORD_WIDTH_BYTES  = WORD_WIDTH_BITS / 8;
    localparam BYTE_OFFSET_WIDTH = $clog2(WORD_WIDTH_BYTES);
    localparam WORD_OFFSET_WIDTH = OFFSET_WIDTH - BYTE_OFFSET_WIDTH;

    // Cache arrays
    reg  [TAG_WIDTH-1:0]         tag_array   [SETS-1:0][ASSOCIATIVITY-1:0];
    reg                          valid_array [SETS-1:0][ASSOCIATIVITY-1:0];
    reg  [PLRU_WIDTH-1:0]        plru_array  [SETS-1:0];
    reg  [LINE_WIDTH_BITS-1:0]   data_array  [SETS-1:0][ASSOCIATIVITY-1:0];
    
    // Addressing
    wire [TAG_WIDTH-1:0]         cpu_tag     = cpu_addr[ADDR_WIDTH_BITS-1:SET_INDEX_WIDTH+OFFSET_WIDTH];
    wire [SET_INDEX_WIDTH-1:0]   set_idx     = cpu_addr[SET_INDEX_WIDTH+OFFSET_WIDTH-1:OFFSET_WIDTH];
    wire [WORD_OFFSET_WIDTH-1:0] word_offset = cpu_addr[OFFSET_WIDTH-1:BYTE_OFFSET_WIDTH];
    
    // FSM states
    localparam CHK_HIT = 1'b0;
    localparam MISS    = 1'b1;
    
    reg state, next_state;
    wire [PLRU_WIDTH-1:0] plru_idx;
    
    // Hit detection
    wire [ASSOCIATIVITY-1:0] way_hit;
    wire hit = |way_hit;
    
    genvar i;
    generate
        for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin : gen_hit
            assign way_hit[i] = valid_array[set_idx][i] && (tag_array[set_idx][i] == cpu_tag);
        end
    endgenerate
    
    // Pseudo-LRU
    assign plru_idx = plru_array[set_idx];
    
    always @(*) begin
        next_state = state;
        bus_addr_valid = 1'b0;
        bus_addr = '0;
    
        if (state == CHK_HIT) begin
            if (hit) 
                next_state = CHK_HIT;
            else begin
                next_state = MISS;
                bus_addr_valid = 1'b1;
                bus_addr = {cpu_tag, set_idx, {OFFSET_WIDTH{1'b0}}};
            end
        end else if (state == MISS && bus_rdata_valid) begin
            next_state = CHK_HIT;
        end
    end


    always @(posedge clk) begin
        if (!rst_n) begin
            state <= CHK_HIT;
            cpu_rdata_valid <= 1'b0;
            
            for (int s = 0; s < SETS; s++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    valid_array[s][w] <= 1'b0;
                    tag_array[s][w] <= '0;
                end
                plru_array[s] <= 2'b00;
            end
        end else begin
            state <= next_state;
            
            if (state == CHK_HIT && hit) begin
                casez (way_hit)
                    4'b???1: begin
                        cpu_rdata <= data_array[set_idx][0][word_offset*WORD_WIDTH_BITS +: WORD_WIDTH_BITS];
                        plru_array[set_idx] <= 2'b11;
                    end
                    4'b??1?: begin
                        cpu_rdata <= data_array[set_idx][1][word_offset*WORD_WIDTH_BITS +: WORD_WIDTH_BITS];
                        plru_array[set_idx] <= 2'b10;
                    end
                    4'b?1??: begin 
                        cpu_rdata <= data_array[set_idx][2][word_offset*WORD_WIDTH_BITS +: WORD_WIDTH_BITS];
                        plru_array[set_idx] <= 2'b01;
                    end
                    4'b1???: begin 
                        cpu_rdata <= data_array[set_idx][3][word_offset*WORD_WIDTH_BITS +: WORD_WIDTH_BITS];
                        plru_array[set_idx] <= 2'b00;
                    end
                endcase
                
                cpu_rdata_valid <= 1'b1;
            end else begin
                cpu_rdata_valid <= 1'b0;
            end
            
            if (state == MISS && bus_rdata_valid) begin
                data_array[set_idx][plru_idx] <= bus_rdata;
                tag_array[set_idx][plru_idx] <= cpu_tag;
                valid_array[set_idx][plru_idx] <= 1'b1;
                
                case (plru_idx)
                    2'b00: plru_array[set_idx] <= 2'b11;
                    2'b01: plru_array[set_idx] <= 2'b10;
                    2'b10: plru_array[set_idx] <= 2'b01;
                    2'b11: plru_array[set_idx] <= 2'b00;
                endcase
            end
        end
    end


endmodule
