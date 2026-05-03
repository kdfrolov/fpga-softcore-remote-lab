/*
 * schoolRISCV - UART-hosted memory simulation testbench
 * fixed version:
 *   - continuous DUT->HOST UART sniffer
 *   - byte FIFO between sniffer and frame parser
 *   - request parser no longer waits directly on uart_txd_o edge
 */

`timescale 1 ns / 100 ps

`include "sr_cpu.vh"

`ifndef SIMULATION_CYCLES
    `define SIMULATION_CYCLES 800000
`endif

module sm_testbench;

    parameter Tt = 20;

    localparam UART_CLK_HZ          = 50000000;
    localparam UART_BAUD            = 115200;
    localparam UART_CLKS_PER_BIT    = UART_CLK_HZ / UART_BAUD;
    localparam UART_HALF_CLKS       = UART_CLKS_PER_BIT / 2;

    localparam MEM_WORDS            = 4096;
    localparam MEM_AW               = 12;

    localparam [7:0] UA_SOF             = 8'hA5;
    localparam [7:0] UA_TYPE_IREAD_REQ  = 8'h01;
    localparam [7:0] UA_TYPE_DREAD_REQ  = 8'h02;
    localparam [7:0] UA_TYPE_WRITE      = 8'h03;
    localparam [7:0] UA_TYPE_IREAD_RESP = 8'h81;
    localparam [7:0] UA_TYPE_DREAD_RESP = 8'h82;
    localparam [7:0] UA_TYPE_WRITE_RESP = 8'h83;

    localparam [7:0] UA_STATUS_OK       = 8'h00;
    localparam [7:0] UA_STATUS_BAD_XOR  = 8'h01;
    localparam [7:0] UA_STATUS_BAD_TYPE = 8'h02;
    localparam [7:0] UA_STATUS_BAD_ADDR = 8'h04;

    localparam integer HOST_RX_FIFO_DEPTH = 256;

    reg         clk;
    reg         rst_n;
    reg  [4:0]  regAddr;
    wire        cpuClk;

    reg         uart_rxd_i;
    wire        uart_txd_o;

    reg [31:0] pc_mem [0:MEM_WORDS-1];
    reg [7:0]  rx_payload [0:15];

    reg [7:0]  host_rx_fifo [0:HOST_RX_FIFO_DEPTH-1];
    integer    host_rx_wr_ptr;
    integer    host_rx_rd_ptr;
    integer    host_rx_count;
    reg [7:0]  host_mon_byte;
    event      host_rx_fifo_event;

    integer i;
    integer cycle;

    integer last_progress_cycle;
    reg [31:0] last_pc;

    // ------------------------------------------------------------
    // Previous values for event-based logging
    // ------------------------------------------------------------
    reg        prev_dbg_ic_valid;
    reg [31:0] prev_dbg_ic_addr;
    reg [31:0] prev_dbg_ic_wdata;
    reg [3:0]  prev_dbg_ic_wstrb;
    reg        prev_dbg_ic_ready;
    reg        prev_dbg_ic_rvalid;
    reg [31:0] prev_dbg_ic_rdata;

    reg        prev_dbg_dc_valid;
    reg [31:0] prev_dbg_dc_addr;
    reg [31:0] prev_dbg_dc_wdata;
    reg [3:0]  prev_dbg_dc_wstrb;
    reg        prev_dbg_dc_ready;
    reg        prev_dbg_dc_rvalid;
    reg [31:0] prev_dbg_dc_rdata;

    reg [31:0] prev_cpu_pc;
    reg [31:0] prev_cpu_instr;
    reg        prev_cpu_instr_valid;
    reg [31:0] prev_a0;

    reg [2:0]  prev_dbg_ua_uart_state;
    reg [3:0]  prev_dbg_ua_rx_state;
    reg        prev_dbg_ua_core_busy;

    reg [7:0]  prev_dbg_cdc_req_type;
    reg [7:0]  prev_dbg_cdc_req_tag;
    reg [31:0] prev_dbg_cdc_req_addr;
    reg [31:0] prev_dbg_cdc_req_wdata;
    reg [3:0]  prev_dbg_cdc_req_wstrb;
    reg        prev_dbg_req_toggle_core;

    reg [7:0]  prev_dbg_cdc_resp_status;
    reg [31:0] prev_dbg_cdc_resp_data;
    reg        prev_dbg_resp_toggle_uart;

    reg        prev_dbg_req_sync1_uart;
    reg        prev_dbg_req_sync2_uart;
    reg        prev_dbg_req_seen_uart;
    reg        prev_dbg_resp_sync1_core;
    reg        prev_dbg_resp_sync2_core;
    reg        prev_dbg_resp_seen_core;

    reg [7:0]  prev_dbg_req_type_uart;
    reg [7:0]  prev_dbg_req_tag_uart;
    reg [31:0] prev_dbg_req_addr_uart;
    reg [31:0] prev_dbg_req_wdata_uart;
    reg [3:0]  prev_dbg_req_wstrb_uart;
    reg [7:0]  prev_dbg_req_seq_uart;
    reg [7:0]  prev_dbg_seq_ctr_uart;

    reg        prev_dbg_tx_valid_i;
    reg [7:0]  prev_dbg_tx_data_i;
    reg        prev_dbg_tx_ready_i;
    reg [4:0]  prev_dbg_tx_total;
    reg [4:0]  prev_dbg_tx_idx;

    reg        prev_dbg_rx_valid_i;
    reg [7:0]  prev_dbg_rx_byte_i;
    reg [31:0] prev_dbg_timeout_cnt;
    reg [7:0]  prev_dbg_rx_type_i;
    reg [7:0]  prev_dbg_rx_seq_i;
    reg [7:0]  prev_dbg_rx_len_i;
    reg [7:0]  prev_dbg_rx_pl_idx_i;
    reg [7:0]  prev_dbg_rx_xor_acc_i;

    reg [1:0]  prev_dbg_utx_state;
    reg [31:0] prev_dbg_utx_clk_cnt;
    reg [2:0]  prev_dbg_utx_bit_cnt;
    reg [7:0]  prev_dbg_utx_shreg;
    reg        prev_dbg_utx_txd_reg;

    reg [1:0]  prev_dbg_urx_state;
    reg [31:0] prev_dbg_urx_clk_cnt;
    reg [2:0]  prev_dbg_urx_bit_cnt;
    reg [7:0]  prev_dbg_urx_shreg;
    reg [7:0]  prev_dbg_urx_data_o;
    reg        prev_dbg_urx_valid_o;

    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------
    lab_top lab_top (
        .clkIn      ( clk        ),
        .rst_n      ( rst_n      ),
        .clkDivide  ( 4'b0       ),
        .clkEnable  ( 1'b1       ),
        .clk        ( cpuClk     ),
        .regAddr    ( 5'b0       ),
        .regData    (            ),
        .uart_rxd_i ( uart_rxd_i ),
        .uart_txd_o ( uart_txd_o )
    );

    defparam lab_top.sm_clk_divider.bypass = 1;

`ifdef ICARUS
    initial $dumpfile("dump.vcd");
    initial $dumpvars(0, sm_testbench);
`endif

    // ------------------------------------------------------------
    // Internal DUT debug aliases
    // ------------------------------------------------------------

    // I-cache backend debug
    wire        dbg_ic_valid;
    wire [31:0] dbg_ic_addr;
    wire [31:0] dbg_ic_wdata;
    wire [3:0]  dbg_ic_wstrb;
    wire        dbg_ic_ready;
    wire        dbg_ic_rvalid;
    wire [31:0] dbg_ic_rdata;

    assign dbg_ic_valid  = lab_top.ic_be_valid;
    assign dbg_ic_addr   = {8'h00, lab_top.ic_be_addr};
    assign dbg_ic_wdata  = lab_top.ic_be_wdata;
    assign dbg_ic_wstrb  = lab_top.ic_be_wstrb;
    assign dbg_ic_ready  = lab_top.ic_be_ready;
    assign dbg_ic_rvalid = lab_top.ic_be_rvalid;
    assign dbg_ic_rdata  = lab_top.ic_be_rdata;

    // D-cache backend debug
    wire        dbg_dc_valid;
    wire [31:0] dbg_dc_addr;
    wire [31:0] dbg_dc_wdata;
    wire [3:0]  dbg_dc_wstrb;
    wire        dbg_dc_ready;
    wire        dbg_dc_rvalid;
    wire [31:0] dbg_dc_rdata;

    assign dbg_dc_valid  = lab_top.dc_be_valid;
    assign dbg_dc_addr   = {8'h00, lab_top.dc_be_addr};
    assign dbg_dc_wdata  = lab_top.dc_be_wdata;
    assign dbg_dc_wstrb  = lab_top.dc_be_wstrb;
    assign dbg_dc_ready  = lab_top.dc_be_ready;
    assign dbg_dc_rvalid = lab_top.dc_be_rvalid;
    assign dbg_dc_rdata  = lab_top.dc_be_rdata;

    // ------------------------------------------------------------
    // Direct taps to current uart_mem_agent_2clk hierarchy
    // ------------------------------------------------------------
    wire [2:0]  dbg_ua_uart_state;
    wire [3:0]  dbg_ua_rx_state;
    wire        dbg_ua_core_busy;

    assign dbg_ua_uart_state = lab_top.dbg_uart_state;
    assign dbg_ua_rx_state   = lab_top.dbg_rx_state;
    assign dbg_ua_core_busy  = lab_top.dbg_core_busy;

    wire [7:0]  dbg_cdc_req_type;
    wire [7:0]  dbg_cdc_req_tag;
    wire [31:0] dbg_cdc_req_addr;
    wire [31:0] dbg_cdc_req_wdata;
    wire [3:0]  dbg_cdc_req_wstrb;
    wire        dbg_req_toggle_core;

    assign dbg_cdc_req_type    = lab_top.u_uart_mem_agent.cdc_req_type;
    assign dbg_cdc_req_tag     = lab_top.u_uart_mem_agent.cdc_req_tag;
    assign dbg_cdc_req_addr    = lab_top.u_uart_mem_agent.cdc_req_addr;
    assign dbg_cdc_req_wdata   = lab_top.u_uart_mem_agent.cdc_req_wdata;
    assign dbg_cdc_req_wstrb   = lab_top.u_uart_mem_agent.cdc_req_wstrb;
    assign dbg_req_toggle_core = lab_top.u_uart_mem_agent.req_toggle_core;

    wire [7:0]  dbg_cdc_resp_status;
    wire [31:0] dbg_cdc_resp_data;
    wire        dbg_resp_toggle_uart;

    assign dbg_cdc_resp_status  = lab_top.u_uart_mem_agent.cdc_resp_status;
    assign dbg_cdc_resp_data    = lab_top.u_uart_mem_agent.cdc_resp_data;
    assign dbg_resp_toggle_uart = lab_top.u_uart_mem_agent.resp_toggle_uart;

    wire dbg_req_sync1_uart;
    wire dbg_req_sync2_uart;
    wire dbg_req_seen_uart;
    wire dbg_resp_sync1_core;
    wire dbg_resp_sync2_core;
    wire dbg_resp_seen_core;

    assign dbg_req_sync1_uart  = lab_top.u_uart_mem_agent.req_sync1_uart;
    assign dbg_req_sync2_uart  = lab_top.u_uart_mem_agent.req_sync2_uart;
    assign dbg_req_seen_uart   = lab_top.u_uart_mem_agent.req_seen_uart;
    assign dbg_resp_sync1_core = lab_top.u_uart_mem_agent.resp_sync1_core;
    assign dbg_resp_sync2_core = lab_top.u_uart_mem_agent.resp_sync2_core;
    assign dbg_resp_seen_core  = lab_top.u_uart_mem_agent.resp_seen_core;

    wire [7:0]  dbg_req_type_uart;
    wire [7:0]  dbg_req_tag_uart;
    wire [31:0] dbg_req_addr_uart;
    wire [31:0] dbg_req_wdata_uart;
    wire [3:0]  dbg_req_wstrb_uart;
    wire [7:0]  dbg_req_seq_uart;
    wire [7:0]  dbg_seq_ctr_uart;

    assign dbg_req_type_uart  = lab_top.u_uart_mem_agent.req_type_uart;
    assign dbg_req_tag_uart   = lab_top.u_uart_mem_agent.req_tag_uart;
    assign dbg_req_addr_uart  = lab_top.u_uart_mem_agent.req_addr_uart;
    assign dbg_req_wdata_uart = lab_top.u_uart_mem_agent.req_wdata_uart;
    assign dbg_req_wstrb_uart = lab_top.u_uart_mem_agent.req_wstrb_uart;
    assign dbg_req_seq_uart   = lab_top.u_uart_mem_agent.req_seq_uart;
    assign dbg_seq_ctr_uart   = lab_top.u_uart_mem_agent.seq_ctr_uart;

    wire        dbg_tx_valid_i;
    wire [7:0]  dbg_tx_data_i;
    wire        dbg_tx_ready_i;
    wire [4:0]  dbg_tx_total;
    wire [4:0]  dbg_tx_idx;

    assign dbg_tx_valid_i = lab_top.u_uart_mem_agent.tx_valid;
    assign dbg_tx_data_i  = lab_top.u_uart_mem_agent.tx_data;
    assign dbg_tx_ready_i = lab_top.u_uart_mem_agent.tx_ready;
    assign dbg_tx_total   = lab_top.u_uart_mem_agent.tx_total;
    assign dbg_tx_idx     = lab_top.u_uart_mem_agent.tx_idx;

    wire        dbg_rx_valid_i;
    wire [7:0]  dbg_rx_byte_i;
    wire [31:0] dbg_timeout_cnt;
    wire [7:0]  dbg_rx_type_i;
    wire [7:0]  dbg_rx_seq_i;
    wire [7:0]  dbg_rx_len_i;
    wire [7:0]  dbg_rx_pl_idx_i;
    wire [7:0]  dbg_rx_xor_acc_i;

    assign dbg_rx_valid_i   = lab_top.u_uart_mem_agent.rx_valid;
    assign dbg_rx_byte_i    = lab_top.u_uart_mem_agent.rx_byte;
    assign dbg_timeout_cnt  = lab_top.u_uart_mem_agent.timeout_cnt;
    assign dbg_rx_type_i    = lab_top.u_uart_mem_agent.rx_type;
    assign dbg_rx_seq_i     = lab_top.u_uart_mem_agent.rx_seq;
    assign dbg_rx_len_i     = lab_top.u_uart_mem_agent.rx_len;
    assign dbg_rx_pl_idx_i  = lab_top.u_uart_mem_agent.rx_pl_idx;
    assign dbg_rx_xor_acc_i = lab_top.u_uart_mem_agent.rx_xor_acc;

    // ------------------------------------------------------------
    // Direct taps to current uart_tx internals
    // ------------------------------------------------------------
    wire [1:0]  dbg_utx_state;
    wire [31:0] dbg_utx_clk_cnt;
    wire [2:0]  dbg_utx_bit_cnt;
    wire [7:0]  dbg_utx_shreg;
    wire        dbg_utx_txd_reg;

    assign dbg_utx_state   = lab_top.u_uart_mem_agent.u_tx.state;
    assign dbg_utx_clk_cnt = lab_top.u_uart_mem_agent.u_tx.clk_cnt;
    assign dbg_utx_bit_cnt = lab_top.u_uart_mem_agent.u_tx.bit_cnt;
    assign dbg_utx_shreg   = lab_top.u_uart_mem_agent.u_tx.shreg;
    assign dbg_utx_txd_reg = lab_top.u_uart_mem_agent.u_tx.txd_reg;

    // ------------------------------------------------------------
    // Direct taps to current uart_rx internals
    // ------------------------------------------------------------
    wire [1:0]  dbg_urx_state;
    wire [31:0] dbg_urx_clk_cnt;
    wire [2:0]  dbg_urx_bit_cnt;
    wire [7:0]  dbg_urx_shreg;
    wire [7:0]  dbg_urx_data_o;
    wire        dbg_urx_valid_o;

    assign dbg_urx_state   = lab_top.u_uart_mem_agent.u_rx.state;
    assign dbg_urx_clk_cnt = lab_top.u_uart_mem_agent.u_rx.clk_cnt;
    assign dbg_urx_bit_cnt = lab_top.u_uart_mem_agent.u_rx.bit_cnt;
    assign dbg_urx_shreg   = lab_top.u_uart_mem_agent.u_rx.shreg;
    assign dbg_urx_data_o  = lab_top.u_uart_mem_agent.u_rx.data_o;
    assign dbg_urx_valid_o = lab_top.u_uart_mem_agent.u_rx.valid_o;

    // ------------------------------------------------------------
    // Clock / reset
    // ------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever clk = #(Tt/2) ~clk;
    end

    initial begin
        rst_n      = 1'b0;
        regAddr    = 5'b0;
        uart_rxd_i = 1'b1;

        for (i = 0; i < MEM_WORDS; i = i + 1)
            pc_mem[i] = 32'b0;

        $readmemh("program.hex", pc_mem);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
    end

    initial begin
        cycle               = 0;
        last_progress_cycle = 0;
        last_pc             = 32'hxxxxxxxx;

        host_rx_wr_ptr      = 0;
        host_rx_rd_ptr      = 0;
        host_rx_count       = 0;
        host_mon_byte       = 8'h00;

        prev_dbg_ic_valid   = 1'b0;
        prev_dbg_ic_addr    = 32'hxxxxxxxx;
        prev_dbg_ic_wdata   = 32'hxxxxxxxx;
        prev_dbg_ic_wstrb   = 4'hx;
        prev_dbg_ic_ready   = 1'b0;
        prev_dbg_ic_rvalid  = 1'b0;
        prev_dbg_ic_rdata   = 32'hxxxxxxxx;

        prev_dbg_dc_valid   = 1'b0;
        prev_dbg_dc_addr    = 32'hxxxxxxxx;
        prev_dbg_dc_wdata   = 32'hxxxxxxxx;
        prev_dbg_dc_wstrb   = 4'hx;
        prev_dbg_dc_ready   = 1'b0;
        prev_dbg_dc_rvalid  = 1'b0;
        prev_dbg_dc_rdata   = 32'hxxxxxxxx;

        prev_cpu_pc          = 32'hxxxxxxxx;
        prev_cpu_instr       = 32'hxxxxxxxx;
        prev_cpu_instr_valid = 1'b0;
        prev_a0              = 32'hxxxxxxxx;

        prev_dbg_ua_uart_state = 3'hx;
        prev_dbg_ua_rx_state   = 4'hx;
        prev_dbg_ua_core_busy  = 1'bx;

        prev_dbg_cdc_req_type    = 8'hxx;
        prev_dbg_cdc_req_tag     = 8'hxx;
        prev_dbg_cdc_req_addr    = 32'hxxxxxxxx;
        prev_dbg_cdc_req_wdata   = 32'hxxxxxxxx;
        prev_dbg_cdc_req_wstrb   = 4'hx;
        prev_dbg_req_toggle_core = 1'bx;

        prev_dbg_cdc_resp_status  = 8'hxx;
        prev_dbg_cdc_resp_data    = 32'hxxxxxxxx;
        prev_dbg_resp_toggle_uart = 1'bx;

        prev_dbg_req_sync1_uart  = 1'bx;
        prev_dbg_req_sync2_uart  = 1'bx;
        prev_dbg_req_seen_uart   = 1'bx;
        prev_dbg_resp_sync1_core = 1'bx;
        prev_dbg_resp_sync2_core = 1'bx;
        prev_dbg_resp_seen_core  = 1'bx;

        prev_dbg_req_type_uart  = 8'hxx;
        prev_dbg_req_tag_uart   = 8'hxx;
        prev_dbg_req_addr_uart  = 32'hxxxxxxxx;
        prev_dbg_req_wdata_uart = 32'hxxxxxxxx;
        prev_dbg_req_wstrb_uart = 4'hx;
        prev_dbg_req_seq_uart   = 8'hxx;
        prev_dbg_seq_ctr_uart   = 8'hxx;

        prev_dbg_tx_valid_i = 1'bx;
        prev_dbg_tx_data_i  = 8'hxx;
        prev_dbg_tx_ready_i = 1'bx;
        prev_dbg_tx_total   = 5'hx;
        prev_dbg_tx_idx     = 5'hx;

        prev_dbg_rx_valid_i   = 1'bx;
        prev_dbg_rx_byte_i    = 8'hxx;
        prev_dbg_timeout_cnt  = 32'hxxxxxxxx;
        prev_dbg_rx_type_i    = 8'hxx;
        prev_dbg_rx_seq_i     = 8'hxx;
        prev_dbg_rx_len_i     = 8'hxx;
        prev_dbg_rx_pl_idx_i  = 8'hxx;
        prev_dbg_rx_xor_acc_i = 8'hxx;

        prev_dbg_utx_state   = 2'bxx;
        prev_dbg_utx_clk_cnt = 32'hxxxxxxxx;
        prev_dbg_utx_bit_cnt = 3'bxxx;
        prev_dbg_utx_shreg   = 8'hxx;
        prev_dbg_utx_txd_reg = 1'bx;

        prev_dbg_urx_state   = 2'bxx;
        prev_dbg_urx_clk_cnt = 32'hxxxxxxxx;
        prev_dbg_urx_bit_cnt = 3'bxxx;
        prev_dbg_urx_shreg   = 8'hxx;
        prev_dbg_urx_data_o  = 8'hxx;
        prev_dbg_urx_valid_o = 1'bx;
    end

    // ------------------------------------------------------------
    // UART helpers
    // ------------------------------------------------------------
    task uart_send_byte;
        input [7:0] data;
        integer b;
        begin
            $display("%0t UART HOST->DUT BYTE %02x", $time, data);

            uart_rxd_i = 1'b0;
            repeat (UART_CLKS_PER_BIT) @(posedge clk);

            for (b = 0; b < 8; b = b + 1) begin
                uart_rxd_i = data[b];
                repeat (UART_CLKS_PER_BIT) @(posedge clk);
            end

            uart_rxd_i = 1'b1;
            repeat (UART_CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    task host_uart_sniff_byte;
        output [7:0] data;
        integer b;
        begin : sniff_one_byte
            data = 8'h00;
            forever begin
                @(negedge uart_txd_o);
                $display("%0t UART DUT->HOST STARTBIT", $time);

                repeat (UART_HALF_CLKS) @(posedge clk);

                if (uart_txd_o !== 1'b0) begin
                    $display("%0t HOST UART: false start, ignoring", $time);
                end else begin
                    repeat (UART_CLKS_PER_BIT) @(posedge clk);

                    for (b = 0; b < 8; b = b + 1) begin
                        data[b] = uart_txd_o;
                        if (b != 7)
                            repeat (UART_CLKS_PER_BIT) @(posedge clk);
                    end

                    repeat (UART_CLKS_PER_BIT) @(posedge clk);
                    $display("%0t UART DUT->HOST BYTE %02x", $time, data);
                    disable sniff_one_byte;
                end
            end
        end
    endtask

    task host_fifo_push;
        input [7:0] data;
        begin
            if (host_rx_count >= HOST_RX_FIFO_DEPTH) begin
                $display("%0t HOST RX FIFO OVERFLOW", $time);
                print_stall_snapshot();
                $stop;
            end

            host_rx_fifo[host_rx_wr_ptr] = data;
            host_rx_wr_ptr = (host_rx_wr_ptr + 1) % HOST_RX_FIFO_DEPTH;
            host_rx_count  = host_rx_count + 1;
            -> host_rx_fifo_event;
        end
    endtask

    task host_fifo_pop;
        output [7:0] data;
        begin
            while (host_rx_count == 0)
                @(host_rx_fifo_event);

            data = host_rx_fifo[host_rx_rd_ptr];
            host_rx_rd_ptr = (host_rx_rd_ptr + 1) % HOST_RX_FIFO_DEPTH;
            host_rx_count  = host_rx_count - 1;
        end
    endtask

    task uart_send_read_resp;
        input [7:0]  resp_type;
        input [7:0]  seq;
        input [7:0]  status;
        input [7:0]  tag;
        input [31:0] data;
        reg   [7:0]  xr;
        begin
            xr = resp_type ^ seq ^ 8'd6 ^
                 status ^ tag ^
                 data[ 7: 0] ^ data[15: 8] ^
                 data[23:16] ^ data[31:24];

            $display("%0t HOST->DUT READ_RESP type=%02x seq=%02x status=%02x tag=%02x data=%08x xor=%02x",
                     $time, resp_type, seq, status, tag, data, xr);

            uart_send_byte(UA_SOF);
            uart_send_byte(resp_type);
            uart_send_byte(seq);
            uart_send_byte(8'd6);
            uart_send_byte(status);
            uart_send_byte(tag);
            uart_send_byte(data[ 7: 0]);
            uart_send_byte(data[15: 8]);
            uart_send_byte(data[23:16]);
            uart_send_byte(data[31:24]);
            uart_send_byte(xr);
        end
    endtask

    task uart_send_write_resp;
        input [7:0] resp_type;
        input [7:0] seq;
        input [7:0] status;
        input [7:0] tag;
        reg   [7:0] xr;
        begin
            xr = resp_type ^ seq ^ 8'd2 ^ status ^ tag;

            $display("%0t HOST->DUT WRITE_RESP type=%02x seq=%02x status=%02x tag=%02x xor=%02x",
                     $time, resp_type, seq, status, tag, xr);

            uart_send_byte(UA_SOF);
            uart_send_byte(resp_type);
            uart_send_byte(seq);
            uart_send_byte(8'd2);
            uart_send_byte(status);
            uart_send_byte(tag);
            uart_send_byte(xr);
        end
    endtask

    task print_rx_payload;
        input integer plen;
        integer j;
        begin
            $write("%0t HOST RX PAYLOAD:", $time);
            for (j = 0; j < plen; j = j + 1)
                $write(" %02x", rx_payload[j]);
            $write("\n");
        end
    endtask

    task print_stall_snapshot;
        begin
            $display("STALL SNAPSHOT cycle=%0d time=%0t", cycle, $time);
            $display("  CPU: pc=%08x instr=%08x instr_valid=%b a0=%08x",
                     lab_top.sm_cpu.pc,
                     lab_top.sm_cpu.instr,
                     lab_top.sm_cpu.instr_valid,
                     lab_top.sm_cpu.rf.rf[10]);

            $display("  IC : valid=%b addr=%08x wstrb=%x wdata=%08x ready=%b rvalid=%b rdata=%08x",
                     dbg_ic_valid, dbg_ic_addr, dbg_ic_wstrb, dbg_ic_wdata,
                     dbg_ic_ready, dbg_ic_rvalid, dbg_ic_rdata);

            $display("  DC : valid=%b addr=%08x wstrb=%x wdata=%08x ready=%b rvalid=%b rdata=%08x",
                     dbg_dc_valid, dbg_dc_addr, dbg_dc_wstrb, dbg_dc_wdata,
                     dbg_dc_ready, dbg_dc_rvalid, dbg_dc_rdata);

            $display("  UART line: rxd_i=%b txd_o=%b", uart_rxd_i, uart_txd_o);

            $display("  UA_TOP: uart_state=%0d rx_state=%0d core_busy=%b",
                     dbg_ua_uart_state, dbg_ua_rx_state, dbg_ua_core_busy);

            $display("  UA_CORE_REQ: type=%02x tag=%02x addr=%08x wstrb=%x wdata=%08x req_toggle=%b",
                     dbg_cdc_req_type, dbg_cdc_req_tag, dbg_cdc_req_addr,
                     dbg_cdc_req_wstrb, dbg_cdc_req_wdata, dbg_req_toggle_core);

            $display("  UA_CORE_RESP: status=%02x data=%08x resp_toggle=%b",
                     dbg_cdc_resp_status, dbg_cdc_resp_data, dbg_resp_toggle_uart);

            $display("  UA_SYNC: req_sync1=%b req_sync2=%b req_seen=%b resp_sync1=%b resp_sync2=%b resp_seen=%b",
                     dbg_req_sync1_uart, dbg_req_sync2_uart, dbg_req_seen_uart,
                     dbg_resp_sync1_core, dbg_resp_sync2_core, dbg_resp_seen_core);

            $display("  UA_UART_REQ: type=%02x seq=%02x tag=%02x addr=%08x wstrb=%x wdata=%08x seq_ctr=%02x",
                     dbg_req_type_uart, dbg_req_seq_uart, dbg_req_tag_uart,
                     dbg_req_addr_uart, dbg_req_wstrb_uart, dbg_req_wdata_uart,
                     dbg_seq_ctr_uart);

            $display("  UA_TX: valid=%b ready=%b data=%02x total=%0d idx=%0d",
                     dbg_tx_valid_i, dbg_tx_ready_i, dbg_tx_data_i,
                     dbg_tx_total, dbg_tx_idx);

            $display("  UA_RX: valid=%b byte=%02x type=%02x seq=%02x len=%02x pl_idx=%02x xor_acc=%02x timeout=%0d",
                     dbg_rx_valid_i, dbg_rx_byte_i, dbg_rx_type_i, dbg_rx_seq_i,
                     dbg_rx_len_i, dbg_rx_pl_idx_i, dbg_rx_xor_acc_i,
                     dbg_timeout_cnt);

            $display("  UTX: state=%0d clk_cnt=%0d bit_cnt=%0d shreg=%02x txd_reg=%b",
                     dbg_utx_state, dbg_utx_clk_cnt, dbg_utx_bit_cnt,
                     dbg_utx_shreg, dbg_utx_txd_reg);

            $display("  URX: state=%0d clk_cnt=%0d bit_cnt=%0d shreg=%02x data_o=%02x valid_o=%b",
                     dbg_urx_state, dbg_urx_clk_cnt, dbg_urx_bit_cnt,
                     dbg_urx_shreg, dbg_urx_data_o, dbg_urx_valid_o);

            $display("  HOST_FIFO: count=%0d rd=%0d wr=%0d", host_rx_count, host_rx_rd_ptr, host_rx_wr_ptr);
        end
    endtask

    // ------------------------------------------------------------
    // PC-side host memory emulator
    // ------------------------------------------------------------
    task host_service_one_request;
        reg [7:0] sof;
        reg [7:0] ptype;
        reg [7:0] pseq;
        reg [7:0] plen;
        reg [7:0] pxor;
        reg [7:0] calc_xor;
        reg [31:0] addr;
        reg [31:0] wdata;
        reg [31:0] rdata;
        reg [3:0]  wstrb;
        reg [7:0]  tag;
        reg [7:0]  status;
        integer j;
        integer word_index;
        begin
            host_fifo_pop(sof);
            while (sof !== UA_SOF) begin
                $display("%0t HOST: skipping byte %02x while waiting SOF=%02x", $time, sof, UA_SOF);
                host_fifo_pop(sof);
            end

            host_fifo_pop(ptype);
            host_fifo_pop(pseq);
            host_fifo_pop(plen);

            calc_xor = ptype ^ pseq ^ plen;

            for (j = 0; j < plen; j = j + 1) begin
                host_fifo_pop(rx_payload[j]);
                calc_xor = calc_xor ^ rx_payload[j];
            end

            host_fifo_pop(pxor);

            $display("%0t HOST RX FRAME sof=%02x type=%02x seq=%02x len=%0d xor_got=%02x xor_calc=%02x xor_ok=%b",
                     $time, sof, ptype, pseq, plen, pxor, calc_xor, (pxor === calc_xor));
            print_rx_payload(plen);

            if (pxor !== calc_xor) begin
                $display("%0t HOST: BAD_XOR in request: type=%02x seq=%02x plen=%0d got=%02x exp=%02x",
                         $time, ptype, pseq, plen, pxor, calc_xor);
            end

            addr       = 32'd0;
            wdata      = 32'd0;
            rdata      = 32'd0;
            wstrb      = 4'd0;
            tag        = 8'd0;
            status     = UA_STATUS_OK;
            word_index = 0;

            case (ptype)
                UA_TYPE_IREAD_REQ,
                UA_TYPE_DREAD_REQ: begin
                    if (plen != 8'd5) begin
                        status = UA_STATUS_BAD_TYPE;
                        tag    = 8'h00;
                        rdata  = 32'h00000000;
                        $display("%0t HOST: BAD LEN for READ request, plen=%0d exp=5", $time, plen);
                    end else begin
                        addr = {rx_payload[3], rx_payload[2], rx_payload[1], rx_payload[0]};
                        tag  = rx_payload[4];

                        if (addr[31:24] != 8'h00) begin
                            status = UA_STATUS_BAD_ADDR;
                            rdata  = 32'h00000000;
                            $display("%0t HOST: BAD ADDR high byte for READ addr=%08x", $time, addr);
                        end else begin
                            word_index = addr[MEM_AW+1:2];
                            if ((word_index < 0) || (word_index >= MEM_WORDS)) begin
                                status = UA_STATUS_BAD_ADDR;
                                rdata  = 32'h00000000;
                                $display("%0t HOST: BAD ADDR range for READ addr=%08x word_index=%0d", $time, addr, word_index);
                            end else begin
                                rdata = pc_mem[word_index];
                            end
                        end
                    end

                    if (ptype == UA_TYPE_IREAD_REQ)
                        $display("%0t HOST: I READ  addr=%08x tag=%02x -> data=%08x status=%02x",
                                 $time, addr, tag, rdata, status);
                    else
                        $display("%0t HOST: D READ  addr=%08x tag=%02x -> data=%08x status=%02x",
                                 $time, addr, tag, rdata, status);

                    if (ptype == UA_TYPE_IREAD_REQ)
                        uart_send_read_resp(UA_TYPE_IREAD_RESP, pseq, status, tag, rdata);
                    else
                        uart_send_read_resp(UA_TYPE_DREAD_RESP, pseq, status, tag, rdata);
                end

                UA_TYPE_WRITE: begin
                    if (plen != 8'd10) begin
                        status = UA_STATUS_BAD_TYPE;
                        tag    = 8'h00;
                        $display("%0t HOST: BAD LEN for WRITE request, plen=%0d exp=10", $time, plen);
                    end else begin
                        addr  = {rx_payload[3], rx_payload[2], rx_payload[1], rx_payload[0]};
                        wstrb = rx_payload[4][3:0];
                        tag   = rx_payload[5];
                        wdata = {rx_payload[9], rx_payload[8], rx_payload[7], rx_payload[6]};

                        if (addr[31:24] != 8'h00) begin
                            status = UA_STATUS_BAD_ADDR;
                            $display("%0t HOST: BAD ADDR high byte for WRITE addr=%08x", $time, addr);
                        end else begin
                            word_index = addr[MEM_AW+1:2];
                            if ((word_index < 0) || (word_index >= MEM_WORDS)) begin
                                status = UA_STATUS_BAD_ADDR;
                                $display("%0t HOST: BAD ADDR range for WRITE addr=%08x word_index=%0d", $time, addr, word_index);
                            end else begin
                                if (wstrb[0]) pc_mem[word_index][ 7: 0] = wdata[ 7: 0];
                                if (wstrb[1]) pc_mem[word_index][15: 8] = wdata[15: 8];
                                if (wstrb[2]) pc_mem[word_index][23:16] = wdata[23:16];
                                if (wstrb[3]) pc_mem[word_index][31:24] = wdata[31:24];
                            end
                        end
                    end

                    $display("%0t HOST: D WRITE addr=%08x tag=%02x wstrb=%x wdata=%08x status=%02x",
                             $time, addr, tag, wstrb, wdata, status);

                    uart_send_write_resp(UA_TYPE_WRITE_RESP, pseq, status, tag);
                end

                default: begin
                    $display("%0t HOST: UNKNOWN request type=%02x seq=%02x len=%0d", $time, ptype, pseq, plen);
                    uart_send_write_resp(UA_TYPE_WRITE_RESP, pseq, UA_STATUS_BAD_TYPE, 8'h00);
                end
            endcase
        end
    endtask

    // ------------------------------------------------------------
    // Continuous DUT->HOST UART monitor
    // ------------------------------------------------------------
    initial begin
        wait(rst_n == 1'b1);
        forever begin
            host_uart_sniff_byte(host_mon_byte);
            host_fifo_push(host_mon_byte);
        end
    end

    // ------------------------------------------------------------
    // Request parser / host emulator
    // ------------------------------------------------------------
    initial begin
        wait(rst_n == 1'b1);
        forever begin
            host_service_one_request();
        end
    end

    // ------------------------------------------------------------
    // Event-based monitor
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n) begin
            // ---------------- I-cache ----------------
            if (dbg_ic_valid &&
               (!prev_dbg_ic_valid ||
                (dbg_ic_addr  !== prev_dbg_ic_addr) ||
                (dbg_ic_wstrb !== prev_dbg_ic_wstrb) ||
                (dbg_ic_wdata !== prev_dbg_ic_wdata))) begin
                $display("%0t IC_REQ_NEW   addr=%08x wstrb=%x wdata=%08x ready=%b rvalid=%b rdata=%08x",
                         $time, dbg_ic_addr, dbg_ic_wstrb, dbg_ic_wdata,
                         dbg_ic_ready, dbg_ic_rvalid, dbg_ic_rdata);
            end

            if (dbg_ic_ready && !prev_dbg_ic_ready) begin
                $display("%0t IC_READY_RISE addr=%08x", $time, dbg_ic_addr);
            end

            if (dbg_ic_rvalid && !prev_dbg_ic_rvalid) begin
                $display("%0t IC_RVALID_RISE addr=%08x rdata=%08x", $time, dbg_ic_addr, dbg_ic_rdata);
            end

            if (dbg_ic_valid && dbg_ic_ready && (!prev_dbg_ic_valid || !prev_dbg_ic_ready)) begin
                $display("%0t IC_REQ_ACCEPT addr=%08x wstrb=%x wdata=%08x",
                         $time, dbg_ic_addr, dbg_ic_wstrb, dbg_ic_wdata);
            end

            if ((dbg_ic_wstrb == 4'b0000) && dbg_ic_rvalid && !prev_dbg_ic_rvalid) begin
                $display("%0t IC_READ_DONE  addr=%08x data=%08x",
                         $time, dbg_ic_addr, dbg_ic_rdata);
            end

            // ---------------- D-cache ----------------
            if (dbg_dc_valid &&
               (!prev_dbg_dc_valid ||
                (dbg_dc_addr  !== prev_dbg_dc_addr) ||
                (dbg_dc_wstrb !== prev_dbg_dc_wstrb) ||
                (dbg_dc_wdata !== prev_dbg_dc_wdata))) begin
                $display("%0t DC_REQ_NEW   addr=%08x wstrb=%x wdata=%08x ready=%b rvalid=%b rdata=%08x",
                         $time, dbg_dc_addr, dbg_dc_wstrb, dbg_dc_wdata,
                         dbg_dc_ready, dbg_dc_rvalid, dbg_dc_rdata);
            end

            if (dbg_dc_ready && !prev_dbg_dc_ready) begin
                $display("%0t DC_READY_RISE addr=%08x", $time, dbg_dc_addr);
            end

            if (dbg_dc_rvalid && !prev_dbg_dc_rvalid) begin
                $display("%0t DC_RVALID_RISE addr=%08x rdata=%08x", $time, dbg_dc_addr, dbg_dc_rdata);
            end

            if (dbg_dc_valid && dbg_dc_ready && (!prev_dbg_dc_valid || !prev_dbg_dc_ready)) begin
                $display("%0t DC_REQ_ACCEPT addr=%08x wstrb=%x wdata=%08x",
                         $time, dbg_dc_addr, dbg_dc_wstrb, dbg_dc_wdata);
            end

            if ((dbg_dc_wstrb != 4'b0000) && dbg_dc_valid && dbg_dc_ready &&
                (!prev_dbg_dc_valid || !prev_dbg_dc_ready)) begin
                $display("%0t DC_WRITE_DONE addr=%08x wdata=%08x wstrb=%x",
                         $time, dbg_dc_addr, dbg_dc_wdata, dbg_dc_wstrb);
            end

            if ((dbg_dc_wstrb == 4'b0000) && dbg_dc_rvalid && !prev_dbg_dc_rvalid) begin
                $display("%0t DC_READ_DONE  addr=%08x data=%08x",
                         $time, dbg_dc_addr, dbg_dc_rdata);
            end

            // ---------------- CPU progression ----------------
            if ((lab_top.sm_cpu.pc !== prev_cpu_pc) ||
                (lab_top.sm_cpu.instr !== prev_cpu_instr) ||
                (lab_top.sm_cpu.instr_valid !== prev_cpu_instr_valid) ||
                (lab_top.sm_cpu.rf.rf[10] !== prev_a0)) begin
                $display("%0t CPU_STATE pc=%08x instr=%08x instr_valid=%b a0=%08x",
                         $time, lab_top.sm_cpu.pc, lab_top.sm_cpu.instr,
                         lab_top.sm_cpu.instr_valid, lab_top.sm_cpu.rf.rf[10]);
            end

            // ---------------- UA top-level state ----------------
            if ((dbg_ua_uart_state !== prev_dbg_ua_uart_state) ||
                (dbg_ua_rx_state   !== prev_dbg_ua_rx_state)   ||
                (dbg_ua_core_busy  !== prev_dbg_ua_core_busy)) begin
                $display("%0t UA_TOP uart_state=%0d rx_state=%0d core_busy=%b",
                         $time, dbg_ua_uart_state, dbg_ua_rx_state, dbg_ua_core_busy);
            end

            // ---------------- Core-side request context ----------------
            if ((dbg_cdc_req_type    !== prev_dbg_cdc_req_type)    ||
                (dbg_cdc_req_tag     !== prev_dbg_cdc_req_tag)     ||
                (dbg_cdc_req_addr    !== prev_dbg_cdc_req_addr)    ||
                (dbg_cdc_req_wdata   !== prev_dbg_cdc_req_wdata)   ||
                (dbg_cdc_req_wstrb   !== prev_dbg_cdc_req_wstrb)   ||
                (dbg_req_toggle_core !== prev_dbg_req_toggle_core)) begin
                $display("%0t UA_CORE_REQ type=%02x tag=%02x addr=%08x wstrb=%x wdata=%08x req_toggle=%b",
                         $time, dbg_cdc_req_type, dbg_cdc_req_tag, dbg_cdc_req_addr,
                         dbg_cdc_req_wstrb, dbg_cdc_req_wdata, dbg_req_toggle_core);
            end

            // ---------------- Response bundle in core domain ----------------
            if ((dbg_cdc_resp_status  !== prev_dbg_cdc_resp_status)  ||
                (dbg_cdc_resp_data    !== prev_dbg_cdc_resp_data)    ||
                (dbg_resp_toggle_uart !== prev_dbg_resp_toggle_uart)) begin
                $display("%0t UA_CORE_RESP status=%02x data=%08x resp_toggle=%b",
                         $time, dbg_cdc_resp_status, dbg_cdc_resp_data, dbg_resp_toggle_uart);
            end

            // ---------------- CDC synchronizers ----------------
            if ((dbg_req_sync1_uart  !== prev_dbg_req_sync1_uart)  ||
                (dbg_req_sync2_uart  !== prev_dbg_req_sync2_uart)  ||
                (dbg_req_seen_uart   !== prev_dbg_req_seen_uart)   ||
                (dbg_resp_sync1_core !== prev_dbg_resp_sync1_core) ||
                (dbg_resp_sync2_core !== prev_dbg_resp_sync2_core) ||
                (dbg_resp_seen_core  !== prev_dbg_resp_seen_core)) begin
                $display("%0t UA_SYNC req_sync1=%b req_sync2=%b req_seen=%b resp_sync1=%b resp_sync2=%b resp_seen=%b",
                         $time, dbg_req_sync1_uart, dbg_req_sync2_uart, dbg_req_seen_uart,
                         dbg_resp_sync1_core, dbg_resp_sync2_core, dbg_resp_seen_core);
            end

            // ---------------- UART-side request context ----------------
            if ((dbg_req_type_uart  !== prev_dbg_req_type_uart)  ||
                (dbg_req_tag_uart   !== prev_dbg_req_tag_uart)   ||
                (dbg_req_addr_uart  !== prev_dbg_req_addr_uart)  ||
                (dbg_req_wdata_uart !== prev_dbg_req_wdata_uart) ||
                (dbg_req_wstrb_uart !== prev_dbg_req_wstrb_uart) ||
                (dbg_req_seq_uart   !== prev_dbg_req_seq_uart)   ||
                (dbg_seq_ctr_uart   !== prev_dbg_seq_ctr_uart)) begin
                $display("%0t UA_UART_REQ type=%02x seq=%02x tag=%02x addr=%08x wstrb=%x wdata=%08x seq_ctr=%02x",
                         $time, dbg_req_type_uart, dbg_req_seq_uart, dbg_req_tag_uart,
                         dbg_req_addr_uart, dbg_req_wstrb_uart, dbg_req_wdata_uart,
                         dbg_seq_ctr_uart);
            end

            // ---------------- UART TX feed inside agent ----------------
            if ((dbg_tx_valid_i !== prev_dbg_tx_valid_i) ||
                (dbg_tx_data_i  !== prev_dbg_tx_data_i)  ||
                (dbg_tx_ready_i !== prev_dbg_tx_ready_i) ||
                (dbg_tx_total   !== prev_dbg_tx_total)   ||
                (dbg_tx_idx     !== prev_dbg_tx_idx)) begin
                $display("%0t UA_TX valid=%b ready=%b data=%02x total=%0d idx=%0d",
                         $time, dbg_tx_valid_i, dbg_tx_ready_i, dbg_tx_data_i,
                         dbg_tx_total, dbg_tx_idx);
            end

            // ---------------- UART RX capture inside agent ----------------
            if ((dbg_rx_valid_i   && !prev_dbg_rx_valid_i) ||
                (dbg_rx_byte_i    !== prev_dbg_rx_byte_i)  ||
                (dbg_rx_type_i    !== prev_dbg_rx_type_i)  ||
                (dbg_rx_seq_i     !== prev_dbg_rx_seq_i)   ||
                (dbg_rx_len_i     !== prev_dbg_rx_len_i)   ||
                (dbg_rx_pl_idx_i  !== prev_dbg_rx_pl_idx_i)||
                (dbg_rx_xor_acc_i !== prev_dbg_rx_xor_acc_i)) begin
                $display("%0t UA_RX valid=%b byte=%02x type=%02x seq=%02x len=%02x pl_idx=%02x xor_acc=%02x timeout=%0d",
                         $time, dbg_rx_valid_i, dbg_rx_byte_i, dbg_rx_type_i, dbg_rx_seq_i,
                         dbg_rx_len_i, dbg_rx_pl_idx_i, dbg_rx_xor_acc_i, dbg_timeout_cnt);
            end

            if ((dbg_timeout_cnt !== prev_dbg_timeout_cnt) &&
                ((dbg_timeout_cnt == 32'd0) || (dbg_timeout_cnt[11:0] == 12'd0))) begin
                $display("%0t UA_TIMEOUT_CNT %0d", $time, dbg_timeout_cnt);
            end

            // ---------------- Physical UART TX internals ----------------
            if ((dbg_utx_state   !== prev_dbg_utx_state)   ||
                (dbg_utx_bit_cnt !== prev_dbg_utx_bit_cnt) ||
                (dbg_utx_txd_reg !== prev_dbg_utx_txd_reg)) begin
                $display("%0t UTX state=%0d clk_cnt=%0d bit_cnt=%0d shreg=%02x txd_reg=%b",
                         $time, dbg_utx_state, dbg_utx_clk_cnt, dbg_utx_bit_cnt,
                         dbg_utx_shreg, dbg_utx_txd_reg);
            end

            // ---------------- Physical UART RX internals ----------------
            if ((dbg_urx_state   !== prev_dbg_urx_state)   ||
                (dbg_urx_bit_cnt !== prev_dbg_urx_bit_cnt) ||
                (dbg_urx_valid_o && !prev_dbg_urx_valid_o) ||
                (dbg_urx_data_o  !== prev_dbg_urx_data_o)) begin
                $display("%0t URX state=%0d clk_cnt=%0d bit_cnt=%0d shreg=%02x data_o=%02x valid_o=%b",
                         $time, dbg_urx_state, dbg_urx_clk_cnt, dbg_urx_bit_cnt,
                         dbg_urx_shreg, dbg_urx_data_o, dbg_urx_valid_o);
            end

            prev_dbg_ic_valid   <= dbg_ic_valid;
            prev_dbg_ic_addr    <= dbg_ic_addr;
            prev_dbg_ic_wdata   <= dbg_ic_wdata;
            prev_dbg_ic_wstrb   <= dbg_ic_wstrb;
            prev_dbg_ic_ready   <= dbg_ic_ready;
            prev_dbg_ic_rvalid  <= dbg_ic_rvalid;
            prev_dbg_ic_rdata   <= dbg_ic_rdata;

            prev_dbg_dc_valid   <= dbg_dc_valid;
            prev_dbg_dc_addr    <= dbg_dc_addr;
            prev_dbg_dc_wdata   <= dbg_dc_wdata;
            prev_dbg_dc_wstrb   <= dbg_dc_wstrb;
            prev_dbg_dc_ready   <= dbg_dc_ready;
            prev_dbg_dc_rvalid  <= dbg_dc_rvalid;
            prev_dbg_dc_rdata   <= dbg_dc_rdata;

            prev_cpu_pc          <= lab_top.sm_cpu.pc;
            prev_cpu_instr       <= lab_top.sm_cpu.instr;
            prev_cpu_instr_valid <= lab_top.sm_cpu.instr_valid;
            prev_a0              <= lab_top.sm_cpu.rf.rf[10];

            prev_dbg_ua_uart_state <= dbg_ua_uart_state;
            prev_dbg_ua_rx_state   <= dbg_ua_rx_state;
            prev_dbg_ua_core_busy  <= dbg_ua_core_busy;

            prev_dbg_cdc_req_type    <= dbg_cdc_req_type;
            prev_dbg_cdc_req_tag     <= dbg_cdc_req_tag;
            prev_dbg_cdc_req_addr    <= dbg_cdc_req_addr;
            prev_dbg_cdc_req_wdata   <= dbg_cdc_req_wdata;
            prev_dbg_cdc_req_wstrb   <= dbg_cdc_req_wstrb;
            prev_dbg_req_toggle_core <= dbg_req_toggle_core;

            prev_dbg_cdc_resp_status  <= dbg_cdc_resp_status;
            prev_dbg_cdc_resp_data    <= dbg_cdc_resp_data;
            prev_dbg_resp_toggle_uart <= dbg_resp_toggle_uart;

            prev_dbg_req_sync1_uart  <= dbg_req_sync1_uart;
            prev_dbg_req_sync2_uart  <= dbg_req_sync2_uart;
            prev_dbg_req_seen_uart   <= dbg_req_seen_uart;
            prev_dbg_resp_sync1_core <= dbg_resp_sync1_core;
            prev_dbg_resp_sync2_core <= dbg_resp_sync2_core;
            prev_dbg_resp_seen_core  <= dbg_resp_seen_core;

            prev_dbg_req_type_uart  <= dbg_req_type_uart;
            prev_dbg_req_tag_uart   <= dbg_req_tag_uart;
            prev_dbg_req_addr_uart  <= dbg_req_addr_uart;
            prev_dbg_req_wdata_uart <= dbg_req_wdata_uart;
            prev_dbg_req_wstrb_uart <= dbg_req_wstrb_uart;
            prev_dbg_req_seq_uart   <= dbg_req_seq_uart;
            prev_dbg_seq_ctr_uart   <= dbg_seq_ctr_uart;

            prev_dbg_tx_valid_i <= dbg_tx_valid_i;
            prev_dbg_tx_data_i  <= dbg_tx_data_i;
            prev_dbg_tx_ready_i <= dbg_tx_ready_i;
            prev_dbg_tx_total   <= dbg_tx_total;
            prev_dbg_tx_idx     <= dbg_tx_idx;

            prev_dbg_rx_valid_i   <= dbg_rx_valid_i;
            prev_dbg_rx_byte_i    <= dbg_rx_byte_i;
            prev_dbg_timeout_cnt  <= dbg_timeout_cnt;
            prev_dbg_rx_type_i    <= dbg_rx_type_i;
            prev_dbg_rx_seq_i     <= dbg_rx_seq_i;
            prev_dbg_rx_len_i     <= dbg_rx_len_i;
            prev_dbg_rx_pl_idx_i  <= dbg_rx_pl_idx_i;
            prev_dbg_rx_xor_acc_i <= dbg_rx_xor_acc_i;

            prev_dbg_utx_state   <= dbg_utx_state;
            prev_dbg_utx_clk_cnt <= dbg_utx_clk_cnt;
            prev_dbg_utx_bit_cnt <= dbg_utx_bit_cnt;
            prev_dbg_utx_shreg   <= dbg_utx_shreg;
            prev_dbg_utx_txd_reg <= dbg_utx_txd_reg;

            prev_dbg_urx_state   <= dbg_urx_state;
            prev_dbg_urx_clk_cnt <= dbg_urx_clk_cnt;
            prev_dbg_urx_bit_cnt <= dbg_urx_bit_cnt;
            prev_dbg_urx_shreg   <= dbg_urx_shreg;
            prev_dbg_urx_data_o  <= dbg_urx_data_o;
            prev_dbg_urx_valid_o <= dbg_urx_valid_o;
        end
    end

    // ------------------------------------------------------------
    // Original disassembler
    // ------------------------------------------------------------
    task disasmInstr;

        reg [ 6:0] cmdOp;
        reg [ 4:0] rd;
        reg [ 2:0] cmdF3;
        reg [ 4:0] rs1;
        reg [ 4:0] rs2;
        reg [ 6:0] cmdF7;
        reg [31:0] immI;
        reg signed [31:0] immB;
        reg [31:0] immU;
        reg [31:0] immS;
        reg [31:0] immJ;

    begin
        cmdOp = lab_top.sm_cpu.cmdOp;
        rd    = lab_top.sm_cpu.rd;
        cmdF3 = lab_top.sm_cpu.cmdF3;
        rs1   = lab_top.sm_cpu.rs1;
        rs2   = lab_top.sm_cpu.rs2;
        cmdF7 = lab_top.sm_cpu.cmdF7;
        immI  = lab_top.sm_cpu.immI;
        immB  = lab_top.sm_cpu.immB;
        immU  = lab_top.sm_cpu.immU;
        immS  = lab_top.sm_cpu.immS;
        immJ  = lab_top.sm_cpu.immJ;

        $write("   ");
        casez( { rs2, cmdF7, cmdF3, cmdOp } )
            default :
                $write ("new/unknown");

            { `RVF5_ANY, `RVF7_ADD, `RVF3_ADD,  `RVOP_ADD  } :
                $write ("add         $%1d, $%1d, $%1d", rd, rs1, rs2);
            { `RVF5_ANY, `RVF7_SUB, `RVF3_SUB,  `RVOP_SUB  } :
                $write ("sub         $%1d, $%1d, $%1d", rd, rs1, rs2);
            { `RVF5_ANY, `RVF7_XOR, `RVF3_XOR,  `RVOP_XOR  } :
                $write ("xor         $%1d, $%1d, $%1d", rd, rs1, rs2);
            { `RVF5_ANY, `RVF7_OR,  `RVF3_OR,   `RVOP_OR   } :
                $write ("or          $%1d, $%1d, $%1d", rd, rs1, rs2);
            { `RVF5_ANY, `RVF7_AND, `RVF3_AND,  `RVOP_AND  } :
                $write ("and         $%1d, $%1d, $%1d", rd, rs1, rs2);

            { `RVF5_ANY, `RVF7_ANY, `RVF3_ADDI, `RVOP_ADDI } :
                $write ("addi        $%1d, $%1d, %1d", rd, rs1, immI);

            { `RVF5_ANY, `RVF7_ANY, `RVF3_BEQ,  `RVOP_BNCH } :
                $write ("beq         $%1d, $%1d, %1d", rs1, rs2, immB);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_BNE,  `RVOP_BNCH } :
                $write ("bne         $%1d, $%1d, %1d", rs1, rs2, immB);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_BLT,  `RVOP_BNCH } :
                $write ("blt         $%1d, $%1d, %1d", rs1, rs2, immB);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_BGE,  `RVOP_BNCH } :
                $write ("bge         $%1d, $%1d, %1d", rs1, rs2, immB);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_BLTU, `RVOP_BNCH } :
                $write ("bltu        $%1d, $%1d, %1d", rs1, rs2, immB);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_BGEU, `RVOP_BNCH } :
                $write ("bgeu        $%1d, $%1d, %1d", rs1, rs2, immB);

            { `RVF5_ANY, `RVF7_ANY, `RVF3_ANY,  `RVOP_JAL  } :
                $write ("jal         $%1d, %1d", rd, immJ);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_JALR, `RVOP_JALR } :
                $write ("jalr        $%1d, $%1d, %1d", rd, rs1, immI);

            { `RVF5_ANY, `RVF7_ANY, `RVF3_LB,   `RVOP_LOAD } :
                $write ("lb          $%1d, %1d($%1d)", rd, immI, rs1);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_LH,   `RVOP_LOAD } :
                $write ("lh          $%1d, %1d($%1d)", rd, immI, rs1);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_LW,   `RVOP_LOAD } :
                $write ("lw          $%1d, %1d($%1d)", rd, immI, rs1);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_LBU,  `RVOP_LOAD } :
                $write ("lbu         $%1d, %1d($%1d)", rd, immI, rs1);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_LHU,  `RVOP_LOAD } :
                $write ("lhu         $%1d, %1d($%1d)", rd, immI, rs1);

            { `RVF5_ANY, `RVF7_ANY, `RVF3_SB,   `RVOP_STORE } :
                $write ("sb          $%1d, %1d($%1d)", rs2, immS, rs1);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_SH,   `RVOP_STORE } :
                $write ("sh          $%1d, %1d($%1d)", rs2, immS, rs1);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_SW,   `RVOP_STORE } :
                $write ("sw          $%1d, %1d($%1d)", rs2, immS, rs1);

            { `RVF5_ANY, `RVF7_ANY, `RVF3_ANY,  `RVOP_LUI  } :
                $write ("lui         $%1d, 0x%8h", rd, immU);
            { `RVF5_ANY, `RVF7_ANY, `RVF3_ANY,  `RVOP_AUIPC } :
                $write ("auipc       $%1d, 0x%8h", rd, immU);
        endcase
    end
    endtask

    // ------------------------------------------------------------
    // Progress watchdog
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            last_progress_cycle <= 0;
            last_pc <= 32'hxxxxxxxx;
        end else begin
            if (dbg_ic_valid || dbg_ic_rvalid ||
                dbg_dc_valid || dbg_dc_rvalid ||
                dbg_tx_valid_i || dbg_rx_valid_i ||
                (lab_top.sm_cpu.pc != last_pc)) begin
                last_progress_cycle <= cycle;
                last_pc <= lab_top.sm_cpu.pc;
            end

            if ((cycle - last_progress_cycle) > 20000) begin
                $display("STALL DETECTED at cycle=%0d time=%0t", cycle, $time);
                print_stall_snapshot();
                $stop;
            end
        end
    end

    // ------------------------------------------------------------
    // Debug / finish
    // ------------------------------------------------------------
    always @(posedge cpuClk) begin
        if (rst_n) begin
            if (lab_top.sm_cpu.instr_valid) begin
                $write("%6d  pc=%08h instr=%08h a0=0x%08h",
                       cycle,
                       lab_top.sm_cpu.pc,
                       lab_top.sm_cpu.instr,
                       lab_top.sm_cpu.rf.rf[10]);
                disasmInstr();
                $write("\n");
            end

            cycle = cycle + 1;

            if (lab_top.sm_cpu.rf.rf[10] == 32'd10) begin
                $display("SUCCESS: a0 reached 10");
                $stop;
            end

            if (cycle > `SIMULATION_CYCLES) begin
                $display("Timeout");
                print_stall_snapshot();
                $stop;
            end
        end
    end

endmodule