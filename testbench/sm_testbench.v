/*
 * schoolRISCV - UART-hosted memory simulation testbench
 * fixed version:
 *   - consistent UA_* protocol macros
 *   - consistent snake_case naming
 *   - PC-side memory model over UART
 *   - active control-protocol smoke test:
 *       SET_HOLD / SET_REGSEL / CORE_RESET / SET_CLKDIV / SET_HOLD=0
 *   - timeout counting starts only after control smoke test completes
 *   - finishes when a0 reaches 10 on the provided counter program
 */

`timescale 1 ns / 100 ps

`include "sr_cpu.vh"
`include "uart_agent_proto.vh"

// ------------------------------------------------------------------
// Fallback definitions for control protocol commands.
// If uart_agent_proto.vh already defines them, these blocks are ignored.
// ------------------------------------------------------------------
`ifndef UA_TYPE_SET_CLKDIV_REQ
    `define UA_TYPE_SET_CLKDIV_REQ   8'h10
`endif
`ifndef UA_TYPE_SET_CLKDIV_RESP
    `define UA_TYPE_SET_CLKDIV_RESP  8'h90
`endif

`ifndef UA_TYPE_SET_HOLD_REQ
    `define UA_TYPE_SET_HOLD_REQ     8'h11
`endif
`ifndef UA_TYPE_SET_HOLD_RESP
    `define UA_TYPE_SET_HOLD_RESP    8'h91
`endif

`ifndef UA_TYPE_CORE_RESET_REQ
    `define UA_TYPE_CORE_RESET_REQ   8'h12
`endif
`ifndef UA_TYPE_CORE_RESET_RESP
    `define UA_TYPE_CORE_RESET_RESP  8'h92
`endif

`ifndef UA_TYPE_SET_REGSEL_REQ
    `define UA_TYPE_SET_REGSEL_REQ   8'h13
`endif
`ifndef UA_TYPE_SET_REGSEL_RESP
    `define UA_TYPE_SET_REGSEL_RESP  8'h93
`endif

`ifndef UA_PLEN_SET_CLKDIV_REQ
    `define UA_PLEN_SET_CLKDIV_REQ   8'd1
`endif
`ifndef UA_PLEN_SET_CLKDIV_RESP
    `define UA_PLEN_SET_CLKDIV_RESP  8'd2
`endif

`ifndef UA_PLEN_SET_HOLD_REQ
    `define UA_PLEN_SET_HOLD_REQ     8'd1
`endif
`ifndef UA_PLEN_SET_HOLD_RESP
    `define UA_PLEN_SET_HOLD_RESP    8'd2
`endif

`ifndef UA_PLEN_CORE_RESET_REQ
    `define UA_PLEN_CORE_RESET_REQ   8'd0
`endif
`ifndef UA_PLEN_CORE_RESET_RESP
    `define UA_PLEN_CORE_RESET_RESP  8'd1
`endif

`ifndef UA_PLEN_SET_REGSEL_REQ
    `define UA_PLEN_SET_REGSEL_REQ   8'd1
`endif
`ifndef UA_PLEN_SET_REGSEL_RESP
    `define UA_PLEN_SET_REGSEL_RESP  8'd2
`endif

`ifndef UA_STATUS_BAD_LEN
    `define UA_STATUS_BAD_LEN        8'h03
`endif

`ifndef SIMULATION_CYCLES
    `define SIMULATION_CYCLES 20000000
`endif

module sm_testbench;

    parameter Tt = 20;

    localparam UART_CLK_HZ       = 50000000;
    localparam UART_BAUD         = 115200;
    localparam UART_CLKS_PER_BIT = UART_CLK_HZ / UART_BAUD;
    localparam UART_HALF_CLKS    = UART_CLKS_PER_BIT / 2;

    localparam MEM_WORDS = 4096;
    localparam MEM_AW    = 12;

    localparam integer HOST_RX_FIFO_DEPTH = 256;

    reg         clk;
    reg         rst_n;
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

    reg [7:0]  ctrl_resp_type;
    reg [7:0]  ctrl_resp_seq;
    reg [7:0]  ctrl_resp_len;
    reg [7:0]  ctrl_resp_p0;
    reg [7:0]  ctrl_resp_p1;
    event      ctrl_resp_event;

    integer i;
    integer cycle;
    integer last_progress_cycle;

    reg [31:0] last_pc;
    reg [31:0] pc_hold_sample;
    reg [7:0]  ctrl_seq;
    reg        run_started;

    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------
    lab_top lab_top (
        .clkIn      (clk),
        .rst_n      (rst_n),
        .clk        (cpuClk),
        .regData    (),
        .uart_rxd_i (uart_rxd_i),
        .uart_txd_o (uart_txd_o)
    );

    defparam lab_top.sm_clk_divider.bypass = 1;

`ifdef ICARUS
    // initial $dumpfile("dump.vcd");
    // initial $dumpvars(0, sm_testbench);
`endif

    // ------------------------------------------------------------
    // Handy debug aliases
    // ------------------------------------------------------------
    wire [31:0] dbg_pc          = lab_top.sm_cpu.pc;
    wire [31:0] dbg_instr       = lab_top.sm_cpu.instr;
    wire        dbg_instr_v     = lab_top.sm_cpu.instr_valid;
    wire [31:0] dbg_a0          = lab_top.sm_cpu.rf.rf[10];

    wire [2:0]  dbg_uart_state  = lab_top.dbg_uart_state;
    wire [3:0]  dbg_rx_state    = lab_top.dbg_rx_state;
    wire        dbg_core_busy   = lab_top.dbg_core_busy;

    wire        dbg_ctrl_hold      = lab_top.uart_hold;
    wire [3:0]  dbg_ctrl_clk_div   = lab_top.uart_clk_div;
    wire [4:0]  dbg_ctrl_reg_addr  = lab_top.uart_reg_addr;

    wire        dbg_ic_valid    = lab_top.ic_be_valid;
    wire [23:0] dbg_ic_addr     = lab_top.ic_be_addr;
    wire        dbg_ic_ready    = lab_top.ic_be_ready;
    wire        dbg_ic_rvalid   = lab_top.ic_be_rvalid;
    wire [31:0] dbg_ic_rdata    = lab_top.ic_be_rdata;

    wire        dbg_dc_valid    = lab_top.dc_be_valid;
    wire [23:0] dbg_dc_addr     = lab_top.dc_be_addr;
    wire [31:0] dbg_dc_wdata    = lab_top.dc_be_wdata;
    wire [3:0]  dbg_dc_wstrb    = lab_top.dc_be_wstrb;
    wire        dbg_dc_ready    = lab_top.dc_be_ready;
    wire        dbg_dc_rvalid   = lab_top.dc_be_rvalid;
    wire [31:0] dbg_dc_rdata    = lab_top.dc_be_rdata;

    // ------------------------------------------------------------
    // Clock / reset / program image
    // ------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever clk = #(Tt/2) ~clk;
    end

    initial begin
        rst_n      = 1'b0;
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

        ctrl_resp_type      = 8'h00;
        ctrl_resp_seq       = 8'h00;
        ctrl_resp_len       = 8'h00;
        ctrl_resp_p0        = 8'h00;
        ctrl_resp_p1        = 8'h00;

        pc_hold_sample      = 32'h00000000;
        ctrl_seq            = 8'h40;
        run_started         = 1'b0;
    end

    // ------------------------------------------------------------
    // UART helpers
    // ------------------------------------------------------------
    task uart_send_byte;
        input [7:0] data;
        integer b;
        begin
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
        begin : sniff_one
            data = 8'h00;
            forever begin
                @(negedge uart_txd_o);
                repeat (UART_HALF_CLKS) @(posedge clk);

                if (uart_txd_o == 1'b0) begin
                    repeat (UART_CLKS_PER_BIT) @(posedge clk);

                    for (b = 0; b < 8; b = b + 1) begin
                        data[b] = uart_txd_o;
                        if (b != 7)
                            repeat (UART_CLKS_PER_BIT) @(posedge clk);
                    end

                    repeat (UART_CLKS_PER_BIT) @(posedge clk);
                    disable sniff_one;
                end
            end
        end
    endtask

    task host_fifo_push;
        input [7:0] data;
        begin
            if (host_rx_count >= HOST_RX_FIFO_DEPTH) begin
                $display("%0t HOST RX FIFO OVERFLOW", $time);
                print_snapshot();
                $stop;
            end

            host_rx_fifo[host_rx_wr_ptr] = data;
            host_rx_wr_ptr = host_rx_wr_ptr + 1;
            if (host_rx_wr_ptr == HOST_RX_FIFO_DEPTH)
                host_rx_wr_ptr = 0;

            host_rx_count = host_rx_count + 1;
            -> host_rx_fifo_event;
        end
    endtask

    task host_fifo_pop;
        output [7:0] data;
        begin
            while (host_rx_count == 0)
                @(host_rx_fifo_event);

            data = host_rx_fifo[host_rx_rd_ptr];
            host_rx_rd_ptr = host_rx_rd_ptr + 1;
            if (host_rx_rd_ptr == HOST_RX_FIFO_DEPTH)
                host_rx_rd_ptr = 0;

            host_rx_count = host_rx_count - 1;
        end
    endtask

    // ------------------------------------------------------------
    // UART response builders for PC-side memory model
    // ------------------------------------------------------------
    task uart_send_read_resp;
        input [7:0]  resptype;
        input [7:0]  seq;
        input [7:0]  status;
        input [7:0]  tag;
        input [31:0] data;
        reg   [7:0]  xr;
        begin
            xr = resptype ^ seq ^ `UA_PLEN_READ_RESP ^ status ^ tag ^
                 data[7:0] ^ data[15:8] ^ data[23:16] ^ data[31:24];

            $display("%0t HOST RESP READ : type=%02x seq=%02x len=%0d status=%02x tag=%02x data=%08x xor=%02x",
                     $time, resptype, seq, `UA_PLEN_READ_RESP, status, tag, data, xr);

            uart_send_byte(`UA_SOF);
            uart_send_byte(resptype);
            uart_send_byte(seq);
            uart_send_byte(`UA_PLEN_READ_RESP);
            uart_send_byte(status);
            uart_send_byte(tag);
            uart_send_byte(data[7:0]);
            uart_send_byte(data[15:8]);
            uart_send_byte(data[23:16]);
            uart_send_byte(data[31:24]);
            uart_send_byte(xr);
        end
    endtask

    task uart_send_write_resp;
        input [7:0] resptype;
        input [7:0] seq;
        input [7:0] status;
        input [7:0] tag;
        reg   [7:0] xr;
        begin
            xr = resptype ^ seq ^ `UA_PLEN_WRITE_RESP ^ status ^ tag;

            $display("%0t HOST RESP WRITE: type=%02x seq=%02x len=%0d status=%02x tag=%02x xor=%02x",
                     $time, resptype, seq, `UA_PLEN_WRITE_RESP, status, tag, xr);

            uart_send_byte(`UA_SOF);
            uart_send_byte(resptype);
            uart_send_byte(seq);
            uart_send_byte(`UA_PLEN_WRITE_RESP);
            uart_send_byte(status);
            uart_send_byte(tag);
            uart_send_byte(xr);
        end
    endtask

    // ------------------------------------------------------------
    // Host -> DUT control frames
    // ------------------------------------------------------------
    task host_send_ctrl_frame;
        input [7:0] pkt_type;
        input [7:0] seq;
        input [7:0] plen;
        input [7:0] p0;
        input [7:0] p1;
        reg   [7:0] xr;
        begin
            xr = pkt_type ^ seq ^ plen;

            uart_send_byte(`UA_SOF);
            uart_send_byte(pkt_type);
            uart_send_byte(seq);
            uart_send_byte(plen);

            if (plen > 0) begin
                uart_send_byte(p0);
                xr = xr ^ p0;
            end

            if (plen > 1) begin
                uart_send_byte(p1);
                xr = xr ^ p1;
            end

            uart_send_byte(xr);
        end
    endtask

    task host_expect_ctrl_resp;
        input [7:0] exp_type;
        input [7:0] exp_seq;
        input [7:0] exp_len;
        input [7:0] exp_status;
        input [7:0] exp_p0;
        begin : wait_ctrl
            forever begin
                @(ctrl_resp_event);

                if ((ctrl_resp_type == exp_type) && (ctrl_resp_seq == exp_seq)) begin
                    if (ctrl_resp_len !== exp_len) begin
                        $display("CTRL RESP BAD LEN: got=%0d exp=%0d", ctrl_resp_len, exp_len);
                        print_snapshot();
                        $stop;
                    end

                    if (ctrl_resp_p0 !== exp_status) begin
                        $display("CTRL RESP BAD STATUS: got=%02x exp=%02x", ctrl_resp_p0, exp_status);
                        print_snapshot();
                        $stop;
                    end

                    if ((exp_len > 1) && (ctrl_resp_p1 !== exp_p0)) begin
                        $display("CTRL RESP BAD PAYLOAD[0]: got=%02x exp=%02x", ctrl_resp_p1, exp_p0);
                        print_snapshot();
                        $stop;
                    end

                    disable wait_ctrl;
                end
            end
        end
    endtask

    task tb_set_clkdiv;
        input [7:0] seq;
        input [3:0] div;
        begin
            host_send_ctrl_frame(
                `UA_TYPE_SET_CLKDIV_REQ,
                seq,
                `UA_PLEN_SET_CLKDIV_REQ,
                {4'b0000, div},
                8'h00
            );

            host_expect_ctrl_resp(
                `UA_TYPE_SET_CLKDIV_RESP,
                seq,
                `UA_PLEN_SET_CLKDIV_RESP,
                `UA_STATUS_OK,
                {4'b0000, div}
            );
        end
    endtask

    task tb_set_hold;
        input [7:0] seq;
        input       hold_val;
        begin
            host_send_ctrl_frame(
                `UA_TYPE_SET_HOLD_REQ,
                seq,
                `UA_PLEN_SET_HOLD_REQ,
                {7'b0000000, hold_val},
                8'h00
            );

            host_expect_ctrl_resp(
                `UA_TYPE_SET_HOLD_RESP,
                seq,
                `UA_PLEN_SET_HOLD_RESP,
                `UA_STATUS_OK,
                {7'b0000000, hold_val}
            );
        end
    endtask

    task tb_core_reset;
        input [7:0] seq;
        begin
            host_send_ctrl_frame(
                `UA_TYPE_CORE_RESET_REQ,
                seq,
                `UA_PLEN_CORE_RESET_REQ,
                8'h00,
                8'h00
            );

            host_expect_ctrl_resp(
                `UA_TYPE_CORE_RESET_RESP,
                seq,
                `UA_PLEN_CORE_RESET_RESP,
                `UA_STATUS_OK,
                8'h00
            );
        end
    endtask

    task tb_set_regsel;
        input [7:0] seq;
        input [4:0] regsel;
        begin
            host_send_ctrl_frame(
                `UA_TYPE_SET_REGSEL_REQ,
                seq,
                `UA_PLEN_SET_REGSEL_REQ,
                {3'b000, regsel},
                8'h00
            );

            host_expect_ctrl_resp(
                `UA_TYPE_SET_REGSEL_RESP,
                seq,
                `UA_PLEN_SET_REGSEL_RESP,
                `UA_STATUS_OK,
                {3'b000, regsel}
            );
        end
    endtask

    // ------------------------------------------------------------
    // Snapshot
    // ------------------------------------------------------------
    task print_snapshot;
        begin
            $display("SNAPSHOT cycle=%0d time=%0t run_started=%0b", cycle, $time, run_started);
            $display("  CPU : pc=%08x instr=%08x instr_valid=%0b a0=%08x",
                     dbg_pc, dbg_instr, dbg_instr_v, dbg_a0);
            $display("  UART: state=%0d rx_state=%0d core_busy=%0b rx_line=%0b tx_line=%0b",
                     dbg_uart_state, dbg_rx_state, dbg_core_busy, uart_rxd_i, uart_txd_o);
            $display("  CTRL: hold=%0b clkdiv=%0d regsel=%0d",
                     dbg_ctrl_hold, dbg_ctrl_clk_div, dbg_ctrl_reg_addr);
            $display("  IC  : valid=%0b addr=%08x ready=%0b rvalid=%0b rdata=%08x",
                     dbg_ic_valid, {8'h00, dbg_ic_addr}, dbg_ic_ready, dbg_ic_rvalid, dbg_ic_rdata);
            $display("  DC  : valid=%0b addr=%08x wstrb=%1x wdata=%08x ready=%0b rvalid=%0b rdata=%08x",
                     dbg_dc_valid, {8'h00, dbg_dc_addr}, dbg_dc_wstrb, dbg_dc_wdata,
                     dbg_dc_ready, dbg_dc_rvalid, dbg_dc_rdata);
            $display("  FIFO: count=%0d rd=%0d wr=%0d",
                     host_rx_count, host_rx_rd_ptr, host_rx_wr_ptr);
        end
    endtask

    // ------------------------------------------------------------
    // Dispatcher for all DUT -> HOST frames
    // - memory requests are serviced here
    // - control responses are latched here
    // ------------------------------------------------------------
    task host_dispatch_one_frame;
        reg [7:0]  sof;
        reg [7:0]  ptype;
        reg [7:0]  pseq;
        reg [7:0]  plen;
        reg [7:0]  pxor;
        reg [7:0]  calcxor;
        reg [31:0] addr;
        reg [31:0] wdata;
        reg [31:0] rdata;
        reg [3:0]  wstrb;
        reg [7:0]  tag;
        reg [7:0]  status;
        integer    j;
        integer    word_index;
        begin
            host_fifo_pop(sof);
            while (sof != `UA_SOF)
                host_fifo_pop(sof);

            host_fifo_pop(ptype);
            host_fifo_pop(pseq);
            host_fifo_pop(plen);

            calcxor = ptype ^ pseq ^ plen;
            for (j = 0; j < 16; j = j + 1)
                rx_payload[j] = 8'h00;

            for (j = 0; j < plen; j = j + 1) begin
                host_fifo_pop(rx_payload[j]);
                calcxor = calcxor ^ rx_payload[j];
            end

            host_fifo_pop(pxor);

            $display("%0t HOST FRAME RX  : type=%02x seq=%02x len=%0d xor_got=%02x xor_exp=%02x",
                     $time, ptype, pseq, plen, pxor, calcxor);

            if (pxor != calcxor) begin
                $display("%0t HOST BAD XOR   : type=%02x seq=%02x len=%0d got=%02x exp=%02x",
                         $time, ptype, pseq, plen, pxor, calcxor);
                print_snapshot();
                $stop;
            end

            case (ptype)
                `UA_TYPE_IREAD_REQ,
                `UA_TYPE_DREAD_REQ: begin
                    addr       = {rx_payload[3], rx_payload[2], rx_payload[1], rx_payload[0]};
                    tag        = rx_payload[4];
                    status     = `UA_STATUS_OK;
                    rdata      = 32'h00000000;
                    word_index = addr[MEM_AW+1:2];

                    if ((plen != `UA_PLEN_READ_REQ) || (addr[31:24] != 8'h00) || (word_index >= MEM_WORDS))
                        status = `UA_STATUS_BAD_ADDR;
                    else
                        rdata = pc_mem[word_index];

                    $display("%0t HOST REQ READ : type=%02x seq=%02x addr=%08x tag=%02x plen=%0d word=%0d status=%02x data=%08x",
                             $time, ptype, pseq, addr, tag, plen, word_index, status, rdata);

                    if (ptype == `UA_TYPE_IREAD_REQ)
                        uart_send_read_resp(`UA_TYPE_IREAD_RESP, pseq, status, tag, rdata);
                    else
                        uart_send_read_resp(`UA_TYPE_DREAD_RESP, pseq, status, tag, rdata);
                end

                `UA_TYPE_WRITE: begin
                    addr       = {rx_payload[3], rx_payload[2], rx_payload[1], rx_payload[0]};
                    wstrb      = rx_payload[4][3:0];
                    tag        = rx_payload[5];
                    wdata      = {rx_payload[9], rx_payload[8], rx_payload[7], rx_payload[6]};
                    status     = `UA_STATUS_OK;
                    word_index = addr[MEM_AW+1:2];

                    if ((plen != `UA_PLEN_WRITE_REQ) || (addr[31:24] != 8'h00) || (word_index >= MEM_WORDS)) begin
                        status = `UA_STATUS_BAD_ADDR;
                    end else begin
                        if (wstrb[0]) pc_mem[word_index][7:0]   = wdata[7:0];
                        if (wstrb[1]) pc_mem[word_index][15:8]  = wdata[15:8];
                        if (wstrb[2]) pc_mem[word_index][23:16] = wdata[23:16];
                        if (wstrb[3]) pc_mem[word_index][31:24] = wdata[31:24];
                    end

                    $display("%0t HOST REQ WRITE: seq=%02x addr=%08x wstrb=%1x tag=%02x data=%08x plen=%0d word=%0d status=%02x",
                             $time, pseq, addr, wstrb, tag, wdata, plen, word_index, status);

                    uart_send_write_resp(`UA_TYPE_WRITE_RESP, pseq, status, tag);
                end

                `UA_TYPE_SET_CLKDIV_RESP,
                `UA_TYPE_SET_HOLD_RESP,
                `UA_TYPE_CORE_RESET_RESP,
                `UA_TYPE_SET_REGSEL_RESP: begin
                    ctrl_resp_type = ptype;
                    ctrl_resp_seq  = pseq;
                    ctrl_resp_len  = plen;
                    ctrl_resp_p0   = (plen > 0) ? rx_payload[0] : 8'h00;
                    ctrl_resp_p1   = (plen > 1) ? rx_payload[1] : 8'h00;

                    $display("%0t HOST CTRL RESP: type=%02x seq=%02x len=%0d p0=%02x p1=%02x",
                             $time, ptype, pseq, plen, ctrl_resp_p0, ctrl_resp_p1);

                    -> ctrl_resp_event;
                end

                default: begin
                    $display("%0t HOST UNKNOWN  : type=%02x seq=%02x len=%0d",
                             $time, ptype, pseq, plen);
                    print_snapshot();
                    $stop;
                end
            endcase
        end
    endtask

    // ------------------------------------------------------------
    // Continuous DUT -> HOST UART sniffer
    // ------------------------------------------------------------
    initial begin
        wait (rst_n == 1'b1);
        forever begin
            host_uart_sniff_byte(host_mon_byte);
            host_fifo_push(host_mon_byte);
        end
    end

    // ------------------------------------------------------------
    // Continuous DUT frame dispatcher
    // ------------------------------------------------------------
    initial begin
        wait (rst_n == 1'b1);
        forever begin
            host_dispatch_one_frame();
        end
    end

    // ------------------------------------------------------------
    // Debug traces
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && dbg_ic_valid && dbg_ic_ready) begin
            $display("%0t IC REQ        : addr=%08x", $time, {8'h00, dbg_ic_addr});
        end
        if (rst_n && dbg_ic_rvalid) begin
            $display("%0t IC RESP       : data=%08x", $time, dbg_ic_rdata);
        end
    end

    always @(posedge clk) begin
        if (rst_n && dbg_dc_valid && dbg_dc_ready) begin
            $display("%0t DC REQ        : addr=%08x wstrb=%1x wdata=%08x",
                     $time, {8'h00, dbg_dc_addr}, dbg_dc_wstrb, dbg_dc_wdata);
        end
        if (rst_n && dbg_dc_rvalid) begin
            $display("%0t DC RESP       : data=%08x", $time, dbg_dc_rdata);
        end
    end

    always @(posedge clk) begin
        if (rst_n && run_started && !dbg_ctrl_hold && !dbg_core_busy &&
            (dbg_pc == 32'h00000000) && !dbg_instr_v &&
            (cycle > 1000) && ((cycle % 5000) == 0)) begin
            $display("%0t STILL IDLE    : pc=%08x instr_valid=%0b icvalid=%0b icready=%0b dcvalid=%0b dcready=%0b",
                     $time, dbg_pc, dbg_instr_v, dbg_ic_valid, dbg_ic_ready, dbg_dc_valid, dbg_dc_ready);
        end
    end

    // ------------------------------------------------------------
    // Active control-protocol smoke test
    // ------------------------------------------------------------
    initial begin : control_smoke_test
        wait (rst_n == 1'b1);
        repeat (3000) @(posedge clk);

        $display("%0t CTRL TEST: SET_HOLD=1", $time);
        tb_set_hold(ctrl_seq, 1'b1);
        ctrl_seq = ctrl_seq + 1'b1;

        if (dbg_ctrl_hold !== 1'b1) begin
            $display("CTRL TEST FAIL: hold was not set");
            print_snapshot();
            $stop;
        end

        pc_hold_sample = dbg_pc;
        repeat (3000) @(posedge clk);
        if (dbg_pc !== pc_hold_sample) begin
            $display("CTRL TEST FAIL: PC changed while hold=1, pc0=%08x pc1=%08x",
                     pc_hold_sample, dbg_pc);
            print_snapshot();
            $stop;
        end

        $display("%0t CTRL TEST: SET_REGSEL=a0(10)", $time);
        tb_set_regsel(ctrl_seq, 5'd10);
        ctrl_seq = ctrl_seq + 1'b1;

        if (dbg_ctrl_reg_addr !== 5'd10) begin
            $display("CTRL TEST FAIL: regsel mismatch");
            print_snapshot();
            $stop;
        end

        $display("%0t CTRL TEST: CORE_RESET", $time);
        tb_core_reset(ctrl_seq);
        ctrl_seq = ctrl_seq + 1'b1;

        if (dbg_ctrl_hold !== 1'b1) begin
            $display("CTRL TEST FAIL: hold was not forced by core reset");
            print_snapshot();
            $stop;
        end

        $display("%0t CTRL TEST: SET_CLKDIV=0", $time);
        tb_set_clkdiv(ctrl_seq, 4'd0);
        ctrl_seq = ctrl_seq + 1'b1;

        if (dbg_ctrl_clk_div !== 4'd0) begin
            $display("CTRL TEST FAIL: clkdiv mismatch");
            print_snapshot();
            $stop;
        end

        $display("%0t CTRL TEST: SET_HOLD=0", $time);
        tb_set_hold(ctrl_seq, 1'b0);
        ctrl_seq = ctrl_seq + 1'b1;

        if (dbg_ctrl_hold !== 1'b0) begin
            $display("CTRL TEST FAIL: hold was not cleared");
            print_snapshot();
            $stop;
        end

        $display("%0t CTRL TEST: control smoke sequence completed", $time);

        run_started         = 1'b1;
        cycle               = 0;
        last_progress_cycle = 0;
        last_pc             = dbg_pc;
    end

    // ------------------------------------------------------------
    // Progress watchdog
    // Skip stall detection while hold=1.
    // Start only after control smoke test is complete.
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            last_progress_cycle <= 0;
            last_pc             <= 32'hxxxxxxxx;
        end else if (run_started) begin
            if (dbg_ctrl_hold) begin
                last_progress_cycle <= cycle;
                last_pc             <= dbg_pc;
            end else begin
                if (dbg_ic_valid || dbg_ic_rvalid || dbg_dc_valid || dbg_dc_rvalid || (dbg_pc != last_pc)) begin
                    last_progress_cycle <= cycle;
                    last_pc             <= dbg_pc;
                end

                if ((cycle - last_progress_cycle) > 50000) begin
                    $display("STALL DETECTED at cycle=%0d time=%0t", cycle, $time);
                    print_snapshot();
                    $stop;
                end
            end
        end
    end

    // ------------------------------------------------------------
    // CPU trace / finish
    // ------------------------------------------------------------
    always @(posedge cpuClk) begin
        if (rst_n && run_started) begin
            if (dbg_instr_v) begin
                $display("%6d  pc=%08x instr=%08x a0=%08x hold=%0b",
                         cycle, dbg_pc, dbg_instr, dbg_a0, dbg_ctrl_hold);
            end

            cycle = cycle + 1;

            if (dbg_a0 == 32'd10) begin
                $display("SUCCESS: a0 reached 10");
                $stop;
            end

            if (cycle > `SIMULATION_CYCLES) begin
                $display("Timeout");
                print_snapshot();
                $stop;
            end
        end
    end

endmodule