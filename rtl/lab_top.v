
//lab hardware top level module
module lab_top
(
    input           clkIn,
    input           rst_n,
    output          clk,
    output  [31:0]  regData,

    // UART pins
    input           uart_rxd_i,
    output          uart_txd_o
);
    // UART-controlled core config
    wire [3:0] uart_clk_div;
    wire       uart_hold;
    wire       uart_core_reset_pulse;
    wire [4:0] uart_reg_addr;

    // core-local reset stretcher
    reg [3:0] core_reset_cnt;
    wire      core_rst_n = rst_n & (core_reset_cnt == 4'd0);
    wire      core_rst = ~core_rst_n;

    always @(posedge clkIn or negedge rst_n) begin
        if (!rst_n)
            core_reset_cnt <= 4'd0;
        else if (uart_core_reset_pulse)
            core_reset_cnt <= 4'd8;
        else if (core_reset_cnt != 4'd0)
            core_reset_cnt <= core_reset_cnt - 1'b1;
    end

    // Clock divider
    sm_clk_divider sm_clk_divider
    (
        .clkIn      ( clkIn        ),
        .rst_n      ( rst_n        ),
        .divide     ( uart_clk_div ),
        .enable     ( 1'b1         ),
        .clkOut     ( clk          )
    );

    // CPU <-> I-cache
    wire [31:0] imAddr;
    wire        imValid;
    wire        imReady;
    wire        imRvalid;
    wire [31:0] imData;


    // I-cache backend
    wire        ic_be_valid;
    wire [23:0] ic_be_addr;
    wire [31:0] ic_be_wdata;
    wire [ 3:0] ic_be_wstrb;
    wire        ic_be_ready;
    wire        ic_be_rvalid;
    wire [31:0] ic_be_rdata;

    // Instruction cache
    iob_cache_iob u_icache (
        .clk_i           ( clk             ),
        .cke_i           ( 1'b1            ),
        .arst_i          ( core_rst        ),

        .iob_valid_i     ( imValid         ),
        .iob_addr_i      ( imAddr[23:0]    ),
        .iob_wdata_i     ( 32'b0           ),
        .iob_wstrb_i     ( 4'b0000         ),
        .iob_rvalid_o    ( imRvalid        ),
        .iob_rdata_o     ( imData          ),
        .iob_ready_o     ( imReady         ),

        .invalidate_i    ( 1'b0            ),
        .invalidate_o    (),
        .wtb_empty_i     ( 1'b1            ),
        .wtb_empty_o     (),

        .be_iob_valid_o  ( ic_be_valid     ),
        .be_iob_addr_o   ( ic_be_addr      ),
        .be_iob_wdata_o  ( ic_be_wdata     ),
        .be_iob_wstrb_o  ( ic_be_wstrb     ),
        .be_iob_rvalid_i ( ic_be_rvalid    ),
        .be_iob_rdata_i  ( ic_be_rdata     ),
        .be_iob_ready_i  ( ic_be_ready     )
    );

    // CPU <-> D-cache
    wire [31:0] dmAddr;
    wire [31:0] dmDataW;
    wire [ 3:0] dmWstrb;
    wire        dmValid;
    wire        dmReady;
    wire        dmRvalid;
    wire [31:0] dmDataR;

    // D-cache back-end
    wire        dc_be_valid;
    wire [23:0] dc_be_addr;
    wire [31:0] dc_be_wdata;
    wire [ 3:0] dc_be_wstrb;
    wire        dc_be_ready;
    wire        dc_be_rvalid;
    wire [31:0] dc_be_rdata;

    // D-cache
    iob_cache_iob u_dcache (
        .clk_i           ( clk             ),
        .cke_i           ( 1'b1            ),
        .arst_i          ( core_rst        ),

        .iob_valid_i     ( dmValid         ),
        .iob_addr_i      ( dmAddr[23:0]    ),
        .iob_wdata_i     ( dmDataW         ),
        .iob_wstrb_i     ( dmWstrb         ),
        .iob_rvalid_o    ( dmRvalid        ),
        .iob_rdata_o     ( dmDataR         ),
        .iob_ready_o     ( dmReady         ),

        .invalidate_i    ( 1'b0            ),
        .invalidate_o    (),
        .wtb_empty_i     ( 1'b1            ),
        .wtb_empty_o     (),

        .be_iob_valid_o  ( dc_be_valid     ),
        .be_iob_addr_o   ( dc_be_addr      ),
        .be_iob_wdata_o  ( dc_be_wdata     ),
        .be_iob_wstrb_o  ( dc_be_wstrb     ),
        .be_iob_rvalid_i ( dc_be_rvalid    ),
        .be_iob_rdata_i  ( dc_be_rdata     ),
        .be_iob_ready_i  ( dc_be_ready     )
    );

    // UART memory + control agent
    wire [2:0] dbg_uart_state;
    wire [3:0] dbg_rx_state;
    wire       dbg_core_busy;

    uart_mem_agent_2clk #(
        .UART_CLK_HZ       (50000000),
        .UART_BAUD         (115200),
        .UART_TIMEOUT_CLKS (5000000)
    ) u_uart_mem_agent (
        .rst_n                 ( rst_n              ),

        .uart_clk              ( clkIn              ),
        .uart_txd_o            ( uart_txd_o         ),
        .uart_rxd_i            ( uart_rxd_i         ),

        .core_clk              ( clk                ),

        .ic_valid_i            ( ic_be_valid        ),
        .ic_addr_i             ( {8'b0, ic_be_addr} ),
        .ic_ready_o            ( ic_be_ready        ),
        .ic_rvalid_o           ( ic_be_rvalid       ),
        .ic_rdata_o            ( ic_be_rdata        ),

        .dc_valid_i            ( dc_be_valid        ),
        .dc_addr_i             ( {8'b0, dc_be_addr} ),
        .dc_wdata_i            ( dc_be_wdata        ),
        .dc_wstrb_i            ( dc_be_wstrb        ),
        .dc_ready_o            ( dc_be_ready        ),
        .dc_rvalid_o           ( dc_be_rvalid       ),
        .dc_rdata_o            ( dc_be_rdata        ),

        .ctrl_clk_div_o        ( uart_clk_div       ),
        .ctrl_hold_o           ( uart_hold          ),
        .ctrl_core_reset_pulse_o ( uart_core_reset_pulse ),
        .ctrl_reg_addr_o       ( uart_reg_addr      ),

        .dbg_uart_state_o      ( dbg_uart_state     ),
        .dbg_rx_state_o        ( dbg_rx_state       ),
        .dbg_core_busy_o       ( dbg_core_busy      )
    );

    // CPU
    sr_cpu sm_cpu (
        .clk        ( clk            ),
        .rst_n      ( core_rst_n     ),
        .hold_ext_i ( uart_hold      ),
        .regAddr    ( uart_reg_addr  ),
        .regData    ( regData        ),

        .imAddr     ( imAddr         ),
        .imValid    ( imValid        ),
        .imReady    ( imReady        ),
        .imRvalid   ( imRvalid       ),
        .imData     ( imData         ),

        .dmAddr     ( dmAddr         ),
        .dmDataW    ( dmDataW        ),
        .dmWstrb    ( dmWstrb        ),
        .dmValid    ( dmValid        ),
        .dmReady    ( dmReady        ),
        .dmRvalid   ( dmRvalid       ),
        .dmDataR    ( dmDataR        )
    );

endmodule

// Test ROM backend for I-cache
module lab_icache_rom_backend
#(
    parameter HEX_FILE  = "program.hex",
    parameter ADDR_W    = 24,
    parameter DATA_W    = 32,
    parameter ROM_WORDS = 4096
)
(
    input                     clk,
    input                     rst_n,
    input                     valid_i,
    input      [ADDR_W-1:0]   addr_i,
    input      [DATA_W-1:0]   wdata_i,
    input      [DATA_W/8-1:0] wstrb_i,
    output                    ready_o,
    output reg                rvalid_o,
    output reg [DATA_W-1:0]   rdata_o
);

    localparam ROM_AW = $clog2(ROM_WORDS);

    reg [DATA_W-1:0] rom [0:ROM_WORDS-1];
    integer i;

    wire read_req = valid_i && (wstrb_i == {DATA_W/8{1'b0}});
    wire [ROM_AW-1:0] rom_index = addr_i[ROM_AW+1:2];

    assign ready_o = 1'b1;

    initial begin
        for (i = 0; i < ROM_WORDS; i = i + 1)
            rom[i] = {DATA_W{1'b0}};
        $readmemh(HEX_FILE, rom);
    end

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rvalid_o <= 1'b0;
            rdata_o  <= {DATA_W{1'b0}};
        end else begin
            rvalid_o <= read_req;
            if (read_req)
                rdata_o <= rom[rom_index];
        end
    end

endmodule

//debouncer module
module sm_debouncer
#(
    parameter SIZE = 1
)
(
    input                      clk,
    input      [ SIZE - 1 : 0] d,
    output reg [ SIZE - 1 : 0] q
);
    reg        [ SIZE - 1 : 0] data;

    always @ (posedge clk) begin
        data <= d;
        q    <= data;
    end

endmodule

//clock divider
module sm_clk_divider
#(
    parameter shift  = 16,
              bypass = 0
)
(
    input           clkIn,
    input           rst_n,
    input   [ 3:0 ] divide,
    input           enable,
    output          clkOut
);
    wire [31:0] cntr;
    wire [31:0] cntrNext = cntr + 1;
    sm_register_we r_cntr(clkIn, rst_n, enable, cntrNext, cntr);

    assign clkOut = bypass ? clkIn 
                           : cntr[shift + divide];
endmodule


// Test RAM backend for D-cache
module lab_dcache_ram_backend
#(
    parameter HEX_FILE  = "",
    parameter ADDR_W    = 24,
    parameter DATA_W    = 32,
    parameter RAM_WORDS = 4096
)
(
    input                     clk,
    input                     rst_n,
    input                     valid_i,
    input      [ADDR_W-1:0]   addr_i,
    input      [DATA_W-1:0]   wdata_i,
    input      [DATA_W/8-1:0] wstrb_i,
    output                    ready_o,
    output reg                rvalid_o,
    output reg [DATA_W-1:0]   rdata_o
);

    localparam RAM_AW = $clog2(RAM_WORDS);

    reg [DATA_W-1:0] mem [0:RAM_WORDS-1];
    reg [RAM_AW-1:0] read_index_q;

    integer i;

    wire is_read  = valid_i && (wstrb_i == {DATA_W/8{1'b0}});
    wire is_write = valid_i && (wstrb_i != {DATA_W/8{1'b0}});

    wire [RAM_AW-1:0] word_index = addr_i[RAM_AW+1:2];

    assign ready_o = 1'b1;

    initial begin
        for (i = 0; i < RAM_WORDS; i = i + 1)
            mem[i] = {DATA_W{1'b0}};

        if (HEX_FILE != "")
            $readmemh(HEX_FILE, mem);
    end

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rvalid_o     <= 1'b0;
            rdata_o      <= {DATA_W{1'b0}};
            read_index_q <= {RAM_AW{1'b0}};
        end else begin
            rvalid_o <= is_read;

            if (is_read)
                read_index_q <= word_index;

            if (is_write) begin
                if (wstrb_i[0]) mem[word_index][ 7: 0] <= wdata_i[ 7: 0];
                if (wstrb_i[1]) mem[word_index][15: 8] <= wdata_i[15: 8];
                if (wstrb_i[2]) mem[word_index][23:16] <= wdata_i[23:16];
                if (wstrb_i[3]) mem[word_index][31:24] <= wdata_i[31:24];
            end

            if (is_read)
                rdata_o <= mem[word_index];
            else
                rdata_o <= mem[read_index_q];
        end
    end

endmodule