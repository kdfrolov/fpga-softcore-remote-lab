// dcach.v - 4KB, 2-way set-associative, 16-byte lines, Write-Through
module dcach #(
    parameter SETS             = 256,  // 4KB / 16B = 256 sets
    parameter ASSOCIATIVITY    = 2,    // 2-way для D-Cache
    parameter LINE_WIDTH_BITS  = 128,
    parameter ADDR_WIDTH_BITS  = 32,
    parameter WORD_WIDTH_BITS  = 32
)(
    input  wire                       clk,
    input  wire                       rst_n,
    
    // CPU interface (schoolRISCV) - Load/Store
    input  wire [ADDR_WIDTH_BITS-1:0] cpu_addr,
    input  wire [WORD_WIDTH_BITS-1:0] cpu_wdata,
    input  wire                       cpu_we,
    input  wire                       cpu_sign,      // Sign-extend
    input  wire                       cpu_op_word,   // LW/SW
    input  wire                       cpu_op_half,   // LH/LHU/SH
    input  wire                       cpu_op_byte,   // LB/LBU/SB
    output reg  [WORD_WIDTH_BITS-1:0] cpu_rdata,
    output reg                        cpu_rdata_valid,
    
    // Cache Bus (UART Agent) - Write-Through
    output wire  [ADDR_WIDTH_BITS-1:0] bus_addr,
    output wire                        bus_addr_valid,
    output wire  [WORD_WIDTH_BITS-1:0] bus_wdata,
    output wire                        bus_we,
    input  wire [LINE_WIDTH_BITS-1:0]  bus_rdata,
    input  wire                        bus_rdata_valid
);

    localparam LINE_WIDTH_BYTES = LINE_WIDTH_BITS / 8;
    localparam SET_INDEX_WIDTH  = $clog2(SETS);
    localparam OFFSET_WIDTH     = $clog2(LINE_WIDTH_BYTES);
    localparam TAG_WIDTH        = ADDR_WIDTH_BITS - SET_INDEX_WIDTH - OFFSET_WIDTH;
    localparam PLRU_WIDTH       = $clog2(ASSOCIATIVITY);  // 1 бит

    localparam WORD_WIDTH_BYTES  = WORD_WIDTH_BITS / 8;
    localparam BYTE_OFFSET_WIDTH = $clog2(WORD_WIDTH_BYTES);
    localparam WORD_OFFSET_WIDTH = OFFSET_WIDTH - BYTE_OFFSET_WIDTH;

    // Cache arrays
    reg  [TAG_WIDTH-1:0]         tag_array   [SETS-1:0][ASSOCIATIVITY-1:0];
    reg                          valid_array [SETS-1:0][ASSOCIATIVITY-1:0];
    reg  [PLRU_WIDTH-1:0]        plru_array  [SETS-1:0];
    reg  [LINE_WIDTH_BITS-1:0]   data_array  [SETS-1:0][ASSOCIATIVITY-1:0];
    
    // Addressing
    wire [TAG_WIDTH-1:0]         cpu_tag        = cpu_addr[ADDR_WIDTH_BITS-1:SET_INDEX_WIDTH+OFFSET_WIDTH];
    wire [SET_INDEX_WIDTH-1:0]   set_idx        = cpu_addr[SET_INDEX_WIDTH+OFFSET_WIDTH-1:OFFSET_WIDTH];
    wire [WORD_OFFSET_WIDTH-1:0] word_offset    = cpu_addr[OFFSET_WIDTH-1:BYTE_OFFSET_WIDTH];
    wire [BYTE_OFFSET_WIDTH-1:0] byte_offset    = cpu_addr[BYTE_OFFSET_WIDTH-1:0];
    
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
    
    // LRU (1 бит: 0=way0 LRU, 1=way1 LRU)
    assign plru_idx = plru_array[set_idx];
    
    wire [WORD_WIDTH_BITS-1:0] load_data_raw;
    assign load_data_raw = 
        way_hit[0] ? data_array[set_idx][0][word_offset*WORD_WIDTH_BITS +: WORD_WIDTH_BITS] :
        way_hit[1] ? data_array[set_idx][1][word_offset*WORD_WIDTH_BITS +: WORD_WIDTH_BITS] : '0;
    
    // Next state logic
    always @(*) begin
        next_state = state;
        if (state == CHK_HIT) begin
            next_state = hit ? CHK_HIT : MISS;
        end else if (state == MISS && bus_rdata_valid) begin
            next_state = CHK_HIT;
        end
    end

    // Bus outputs
    assign bus_addr_valid = state == CHK_HIT && (cpu_we || !hit);
    assign bus_addr = bus_addr_valid ? (cpu_we && hit ? cpu_addr : {cpu_tag, set_idx, {OFFSET_WIDTH{1'b0}}}) : '0;
    assign bus_wdata = cpu_wdata;
    assign bus_we = state == CHK_HIT && cpu_we && hit;
    
    // Write mask for subword
    wire [LINE_WIDTH_BITS-1:0] write_mask;
    assign write_mask = cpu_op_byte  ? (128'hFF  << (byte_offset << 3)) :
                       cpu_op_half  ? (128'hFFFF << (byte_offset << 3)) :
                       (128'hFFFFFFFF << (word_offset << 5));

    wire [LINE_WIDTH_BITS-1:0] wdata_aligned;
    assign wdata_aligned = cpu_wdata << (cpu_op_byte  ? (byte_offset << 3) : cpu_op_half  ? (byte_offset << 3) : (word_offset << 5));
    
    // Sequential logic
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= CHK_HIT;
            cpu_rdata_valid <= 1'b0;
            
            for (int s = 0; s < SETS; s++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    valid_array[s][w] <= 1'b0;
                    tag_array[s][w] <= '0;
                end
                plru_array[s] <= 1'b0;
            end
        end else begin
            state <= next_state;
            
            // LOAD HIT
            if (state == CHK_HIT && !cpu_we && hit) begin
                cpu_rdata_valid <= 1'b1;
                
                case ({cpu_op_word, cpu_op_half, cpu_op_byte})
                    3'b001: cpu_rdata <= {{24{cpu_sign ? load_data_raw[7]  : 1'b0}}, load_data_raw[7:0]  };  // LB/LBU
                    3'b010: cpu_rdata <= {{16{cpu_sign ? load_data_raw[15] : 1'b0}}, load_data_raw[15:0] };  // LH/LHU
                    3'b100: cpu_rdata <= load_data_raw;                                                      // LW
                    default: cpu_rdata <= 32'b0;
                endcase
                
                // PLRU: hit way → MRU
                plru_array[set_idx] <= way_hit[0] ? 1'b1 : 1'b0;
            end else begin
                cpu_rdata_valid <= 1'b0;
            end
            
            // STORE HIT
            if (state == CHK_HIT && cpu_we && hit) begin
                if (way_hit[0]) begin
                    data_array[set_idx][0] <= (data_array[set_idx][0] & ~write_mask) | (wdata_aligned & write_mask);
                    plru_array[set_idx] <= 1'b1;
                end else begin
                    data_array[set_idx][1] <= (data_array[set_idx][1] & ~write_mask) | (wdata_aligned & write_mask);
                    plru_array[set_idx] <= 1'b0;
                end
            end
            
            // CACHE LINE FILL (load miss / write miss)
            if (state == MISS && bus_rdata_valid) begin
                data_array[set_idx][plru_idx] <= bus_rdata;
                tag_array[set_idx][plru_idx] <= cpu_tag;
                valid_array[set_idx][plru_idx] <= 1'b1;
                
                // PLRU
                plru_array[set_idx] <= ~plru_idx;
            end
        end
    end

endmodule
