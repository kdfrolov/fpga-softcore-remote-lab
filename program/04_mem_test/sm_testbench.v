/*
 * schoolRISCV - UART-hosted memory simulation testbench
 * modified for data-memory UART-agent test:
 *   program:
 *     li t0, 10
 *     sw t0, 0x104(x0)
 *     nop
 *     nop
 *     lw a0, 0x104(x0)
 *     nop
 *     nop
 *     li a0, 0
 *     li t0, -2
 *     sb t0, 0x100(x0)
 *     lb a0, 0x100(x0)
 *     nop
 *     nop
 *     lbu a0, 0x100(x0)
 *   loop:
 *     j loop
 */

`timescale 1 ns / 100 ps

`ifndef SIMULATION_CYCLES
    `define SIMULATION_CYCLES 2500000
`endif

module sm_testbench;

    parameter Tt = 20;

    localparam UART_CLK_HZ          = 50000000;
    localparam UART_BAUD            = 115200;
    localparam UART_CLKS_PER_BIT    = UART_CLK_HZ / UART_BAUD;
    localparam UART_HALF_CLKS       = UART_CLKS_PER_BIT / 2;

    localparam MEM_WORDS            = 4096;
    localparam MEM_AW               = 12;

    localparam [31:0] DATA_WORD_ADDR = 32'h00000104;
    localparam [31:0] DATA_BYTE_ADDR = 32'h00000100;

    localparam integer CACHE_LINE_BYTES = 32;
    localparam integer CACHE_LINE_WORDS = 8;
    localparam [31:0] CACHE_LINE_MASK   = 32'hffffffe0;

    // Self-contained RV32I decode constants
    localparam [6:0] TB_RVOP_LOAD   = 7'b0000011;
    localparam [6:0] TB_RVOP_STORE  = 7'b0100011;

    localparam [2:0] TB_RVF3_LB     = 3'b000;
    localparam [2:0] TB_RVF3_LBU    = 3'b100;
    localparam [2:0] TB_RVF3_LW     = 3'b010;
    localparam [2:0] TB_RVF3_SB     = 3'b000;
    localparam [2:0] TB_RVF3_SW     = 3'b010;

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
    integer sys_cycle;
    integer last_progress_cycle;
    reg [31:0] last_pc;

    reg [31:0] prev_cpu_pc;
    reg [31:0] prev_cpu_instr;
    reg        prev_cpu_instr_valid;
    reg [31:0] prev_a0;
    reg [31:0] prev_dbg_dm_addr;
    reg [31:0] prev_dbg_alu_result;
    reg        prev_dbg_dm_we;
    reg        prev_dbg_reg_write_ctrl;
    reg        prev_dbg_reg_write_rf;
    reg [2:0]  prev_dbg_wd_src;
    reg        prev_dbg_pc_hold;
    reg        prev_dbg_commit;
    reg        prev_dbg_load_pending;
    reg        prev_dbg_im_pending;
    reg        prev_dbg_dc_valid;
    reg        prev_dbg_dc_ready;
    reg        prev_dbg_dc_rvalid;

    reg        prev_mon_dc_valid;
    reg        prev_mon_dc_ready;
    reg        prev_mon_dc_rvalid;
    reg [31:0] prev_mon_dc_addr;
    reg [3:0]  prev_mon_dc_wstrb;
    reg [31:0] prev_mon_dc_wdata;
    reg [31:0] prev_mon_dc_rdata;

    reg        seen_sw_host;
    reg        seen_lw_host;
    reg        seen_sb_host;
    integer    seen_dread_byte_host_count;

    reg        seen_a0_after_lw;
    reg        seen_a0_after_li0;
    reg        seen_a0_after_lb;
    reg        seen_a0_after_lbu;

    reg        test_passed;

    // ------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------
    function [31:0] mem_word_at_addr;
        input [31:0] byte_addr;
        integer idx;
        begin
            if (byte_addr[31:24] != 8'h00) begin
                mem_word_at_addr = 32'hbad0_0001;
            end else if (byte_addr > ((MEM_WORDS * 4) - 4)) begin
                mem_word_at_addr = 32'hbad0_0002;
            end else begin
                idx = byte_addr[MEM_AW+1:2];
                mem_word_at_addr = pc_mem[idx];
            end
        end
    endfunction

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
    wire        dbg_ic_valid;
    wire [31:0] dbg_ic_addr;
    wire [31:0] dbg_ic_wdata;
    wire [3:0]  dbg_ic_wstrb;
    wire        dbg_ic_ready;
    wire        dbg_ic_rvalid;
    wire [31:0] dbg_ic_rdata;

    assign dbg_ic_valid  = lab_top.ic_be_valid;
    assign dbg_ic_addr   = {{(32-$bits(lab_top.ic_be_addr)){1'b0}}, lab_top.ic_be_addr};
    assign dbg_ic_wdata  = lab_top.ic_be_wdata;
    assign dbg_ic_wstrb  = lab_top.ic_be_wstrb;
    assign dbg_ic_ready  = lab_top.ic_be_ready;
    assign dbg_ic_rvalid = lab_top.ic_be_rvalid;
    assign dbg_ic_rdata  = lab_top.ic_be_rdata;

    wire        dbg_dc_valid;
    wire [31:0] dbg_dc_addr;
    wire [31:0] dbg_dc_wdata;
    wire [3:0]  dbg_dc_wstrb;
    wire        dbg_dc_ready;
    wire        dbg_dc_rvalid;
    wire [31:0] dbg_dc_rdata;

    assign dbg_dc_valid  = lab_top.dc_be_valid;
    assign dbg_dc_addr   = {{(32-$bits(lab_top.dc_be_addr)){1'b0}}, lab_top.dc_be_addr};
    assign dbg_dc_wdata  = lab_top.dc_be_wdata;
    assign dbg_dc_wstrb  = lab_top.dc_be_wstrb;
    assign dbg_dc_ready  = lab_top.dc_be_ready;
    assign dbg_dc_rvalid = lab_top.dc_be_rvalid;
    assign dbg_dc_rdata  = lab_top.dc_be_rdata;

    // ------------------------------------------------------------
    // CPU internal debug aliases
    // ------------------------------------------------------------
    wire [6:0]  dbg_cmdOp;
    wire [4:0]  dbg_rd;
    wire [2:0]  dbg_cmdF3;
    wire [4:0]  dbg_rs1;
    wire [4:0]  dbg_rs2;
    wire [6:0]  dbg_cmdF7;

    wire [31:0] dbg_immI;
    wire [31:0] dbg_immS;
    wire [31:0] dbg_immB;
    wire [31:0] dbg_immU;
    wire [31:0] dbg_immJ;

    wire [31:0] dbg_rd1;
    wire [31:0] dbg_rd2;
    wire [31:0] dbg_wd3;

    wire [31:0] dbg_aluSrc1In;
    wire [31:0] dbg_aluSrc2In;
    wire [31:0] dbg_aluResult;

    wire        dbg_regWrite_ctrl;
    wire        dbg_regWrite_rf;
    wire        dbg_dmWe;
    wire        dbg_dmSign;
    wire        dbg_op_byte;
    wire        dbg_op_half;
    wire        dbg_op_word;
    wire [2:0]  dbg_wdSrc;
    wire        dbg_aluSrc1Sel;
    wire [1:0]  dbg_aluSrc2Sel;
    wire [3:0]  dbg_aluControl;

    wire [1:0]  dbg_pcSrc1;
    wire        dbg_pcSrc2;
    wire [31:0] dbg_pcSrc1In;
    wire [31:0] dbg_pcSrc2In;
    wire [31:0] dbg_pcNext;
    wire        dbg_pc_hold;

    wire [31:0] dbg_dmAddr;
    wire [31:0] dbg_dmDataW;
    wire [31:0] dbg_dmDataR;

    wire        dbg_instr_valid;
    wire        dbg_load_pending;
    wire        dbg_im_pending;
    wire        dbg_commit;
    wire        dbg_dm_fire;

    wire        dbg_is_load;
    wire        dbg_is_store;
    wire        dbg_is_lb;
    wire        dbg_is_lbu;
    wire        dbg_is_lw;
    wire        dbg_is_sb;
    wire        dbg_is_sw;

    assign dbg_cmdOp         = lab_top.sm_cpu.cmdOp;
    assign dbg_rd            = lab_top.sm_cpu.rd;
    assign dbg_cmdF3         = lab_top.sm_cpu.cmdF3;
    assign dbg_rs1           = lab_top.sm_cpu.rs1;
    assign dbg_rs2           = lab_top.sm_cpu.rs2;
    assign dbg_cmdF7         = lab_top.sm_cpu.cmdF7;

    assign dbg_immI          = lab_top.sm_cpu.immI;
    assign dbg_immS          = lab_top.sm_cpu.immS;
    assign dbg_immB          = lab_top.sm_cpu.immB;
    assign dbg_immU          = lab_top.sm_cpu.immU;
    assign dbg_immJ          = lab_top.sm_cpu.immJ;

    assign dbg_rd1           = lab_top.sm_cpu.rd1;
    assign dbg_rd2           = lab_top.sm_cpu.rd2;
    assign dbg_wd3           = lab_top.sm_cpu.wd3;

    assign dbg_aluSrc1In     = lab_top.sm_cpu.aluSrc1In;
    assign dbg_aluSrc2In     = lab_top.sm_cpu.aluSrc2In;
    assign dbg_aluResult     = lab_top.sm_cpu.aluResult;

    assign dbg_regWrite_ctrl = lab_top.sm_cpu.regWrite_ctrl;
    assign dbg_regWrite_rf   = lab_top.sm_cpu.regWrite_rf;
    assign dbg_dmWe          = lab_top.sm_cpu.dmWe;
    assign dbg_dmSign        = lab_top.sm_cpu.dmSign;
    assign dbg_op_byte       = lab_top.sm_cpu.op_byte;
    assign dbg_op_half       = lab_top.sm_cpu.op_half;
    assign dbg_op_word       = lab_top.sm_cpu.op_word;
    assign dbg_wdSrc         = lab_top.sm_cpu.wdSrc;
    assign dbg_aluSrc1Sel    = lab_top.sm_cpu.aluSrc1;
    assign dbg_aluSrc2Sel    = lab_top.sm_cpu.aluSrc2;
    assign dbg_aluControl    = lab_top.sm_cpu.aluControl;

    assign dbg_pcSrc1        = lab_top.sm_cpu.pcSrc1;
    assign dbg_pcSrc2        = lab_top.sm_cpu.pcSrc2;
    assign dbg_pcSrc1In      = lab_top.sm_cpu.pcSrc1In;
    assign dbg_pcSrc2In      = lab_top.sm_cpu.pcSrc2In;
    assign dbg_pcNext        = lab_top.sm_cpu.pcNext;
    assign dbg_pc_hold       = lab_top.sm_cpu.pc_hold;

    assign dbg_dmAddr        = lab_top.sm_cpu.dmAddr;
    assign dbg_dmDataW       = lab_top.sm_cpu.dmDataW;
    assign dbg_dmDataR       = lab_top.sm_cpu.dmDataR;

    assign dbg_instr_valid   = lab_top.sm_cpu.instr_valid;
    assign dbg_load_pending  = lab_top.sm_cpu.load_pending;
    assign dbg_im_pending    = lab_top.sm_cpu.im_pending;
    assign dbg_commit        = lab_top.sm_cpu.commit;
    assign dbg_dm_fire       = lab_top.sm_cpu.dm_fire;

    assign dbg_is_load  = (dbg_cmdOp == TB_RVOP_LOAD);
    assign dbg_is_store = (dbg_cmdOp == TB_RVOP_STORE);
    assign dbg_is_lb    = (dbg_cmdOp == TB_RVOP_LOAD)  && (dbg_cmdF3 == TB_RVF3_LB);
    assign dbg_is_lbu   = (dbg_cmdOp == TB_RVOP_LOAD)  && (dbg_cmdF3 == TB_RVF3_LBU);
    assign dbg_is_lw    = (dbg_cmdOp == TB_RVOP_LOAD)  && (dbg_cmdF3 == TB_RVF3_LW);
    assign dbg_is_sb    = (dbg_cmdOp == TB_RVOP_STORE) && (dbg_cmdF3 == TB_RVF3_SB);
    assign dbg_is_sw    = (dbg_cmdOp == TB_RVOP_STORE) && (dbg_cmdF3 == TB_RVF3_SW);

    // ------------------------------------------------------------
    // Clock / reset
    // ------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #(Tt/2) clk = ~clk;
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
        cycle                   = 0;
        sys_cycle               = 0;
        last_progress_cycle     = 0;
        last_pc                 = 32'hxxxxxxxx;

        host_rx_wr_ptr          = 0;
        host_rx_rd_ptr          = 0;
        host_rx_count           = 0;
        host_mon_byte           = 8'h00;

        prev_cpu_pc             = 32'hxxxxxxxx;
        prev_cpu_instr          = 32'hxxxxxxxx;
        prev_cpu_instr_valid    = 1'b0;
        prev_a0                 = 32'hxxxxxxxx;
        prev_dbg_dm_addr        = 32'hxxxxxxxx;
        prev_dbg_alu_result     = 32'hxxxxxxxx;
        prev_dbg_dm_we          = 1'b0;
        prev_dbg_reg_write_ctrl = 1'b0;
        prev_dbg_reg_write_rf   = 1'b0;
        prev_dbg_wd_src         = 3'bxxx;
        prev_dbg_pc_hold        = 1'b0;
        prev_dbg_commit         = 1'b0;
        prev_dbg_load_pending   = 1'b0;
        prev_dbg_im_pending     = 1'b0;
        prev_dbg_dc_valid       = 1'b0;
        prev_dbg_dc_ready       = 1'b0;
        prev_dbg_dc_rvalid      = 1'b0;

        prev_mon_dc_valid       = 1'b0;
        prev_mon_dc_ready       = 1'b0;
        prev_mon_dc_rvalid      = 1'b0;
        prev_mon_dc_addr        = 32'hxxxxxxxx;
        prev_mon_dc_wstrb       = 4'bxxxx;
        prev_mon_dc_wdata       = 32'hxxxxxxxx;
        prev_mon_dc_rdata       = 32'hxxxxxxxx;

        seen_sw_host            = 1'b0;
        seen_lw_host            = 1'b0;
        seen_sb_host            = 1'b0;
        seen_dread_byte_host_count = 0;

        seen_a0_after_lw        = 1'b0;
        seen_a0_after_li0       = 1'b0;
        seen_a0_after_lb        = 1'b0;
        seen_a0_after_lbu       = 1'b0;

        test_passed             = 1'b0;
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

            $display("%0t HOST-DUT READRESP type%02x seq%02x status%02x tag%02x data%08x xor%02x",
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

            $display("%0t HOST-DUT WRITERESP type%02x seq%02x status%02x tag%02x xor%02x",
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
            $write("%0t HOST RX PAYLOAD", $time);
            for (j = 0; j < plen; j = j + 1)
                $write(" %02x", rx_payload[j]);
            $write("\n");
        end
    endtask

    task print_stall_snapshot;
        begin
            $display("STALL SNAPSHOT sys_cycle=%0d cpu_cycle=%0d time=%0t", sys_cycle, cycle, $time);

            $display("  CPU: pc=%08x instr=%08x instr_valid=%b a0=%08x",
                     lab_top.sm_cpu.pc,
                     lab_top.sm_cpu.instr,
                     dbg_instr_valid,
                     lab_top.sm_cpu.rf.rf[10]);

            $display("  DECODE: op=%02x f3=%x f7=%02x rd=%0d rs1=%0d rs2=%0d immI=%08x immS=%08x immB=%08x immU=%08x immJ=%08x",
                     dbg_cmdOp, dbg_cmdF3, dbg_cmdF7, dbg_rd, dbg_rs1, dbg_rs2,
                     dbg_immI, dbg_immS, dbg_immB, dbg_immU, dbg_immJ);

            $display("  REGS: rs1_val=%08x rs2_val=%08x wd3=%08x",
                     dbg_rd1, dbg_rd2, dbg_wd3);

            $display("  EXEC: aluSrc1Sel=%b aluSrc2Sel=%b aluCtrl=%x aluA=%08x aluB=%08x aluResult=%08x",
                     dbg_aluSrc1Sel, dbg_aluSrc2Sel, dbg_aluControl,
                     dbg_aluSrc1In, dbg_aluSrc2In, dbg_aluResult);

            $display("  CTRL: regWrite_ctrl=%b regWrite_rf=%b wdSrc=%0d dmWe=%b dmSign=%b op_byte=%b op_half=%b op_word=%b pc_hold=%b commit=%b load_pending=%b im_pending=%b dm_fire=%b",
                     dbg_regWrite_ctrl, dbg_regWrite_rf, dbg_wdSrc, dbg_dmWe, dbg_dmSign,
                     dbg_op_byte, dbg_op_half, dbg_op_word,
                     dbg_pc_hold, dbg_commit, dbg_load_pending, dbg_im_pending, dbg_dm_fire);

            $display("  NEXT: pcSrc1=%0d pcSrc2=%b pcSrc1In=%08x pcSrc2In=%08x pcNext=%08x",
                     dbg_pcSrc1, dbg_pcSrc2, dbg_pcSrc1In, dbg_pcSrc2In, dbg_pcNext);

            $display("  LSU: is_load=%b is_store=%b is_lb=%b is_lbu=%b is_lw=%b is_sb=%b is_sw=%b dmAddr=%08x dmDataW=%08x dmDataR=%08x",
                     dbg_is_load, dbg_is_store, dbg_is_lb, dbg_is_lbu, dbg_is_lw,
                     dbg_is_sb, dbg_is_sw, dbg_dmAddr, dbg_dmDataW, dbg_dmDataR);

            $display("  IC : valid=%b addr=%08x wstrb=%x wdata=%08x ready=%b rvalid=%b rdata=%08x",
                     dbg_ic_valid, dbg_ic_addr, dbg_ic_wstrb, dbg_ic_wdata,
                     dbg_ic_ready, dbg_ic_rvalid, dbg_ic_rdata);

            $display("  DC : valid=%b addr=%08x wstrb=%x wdata=%08x ready=%b rvalid=%b rdata=%08x",
                     dbg_dc_valid, dbg_dc_addr, dbg_dc_wstrb, dbg_dc_wdata,
                     dbg_dc_ready, dbg_dc_rvalid, dbg_dc_rdata);

            $display("  CACHE VIEW: cpu_dm_line=%08x cpu_dm_off=%0d dc_line=%08x dc_off=%0d",
                     (dbg_dmAddr & CACHE_LINE_MASK), dbg_dmAddr[3:2],
                     (dbg_dc_addr & CACHE_LINE_MASK), dbg_dc_addr[3:2]);

            $display("  LINE0: [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h00, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h00),
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h04, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h04),
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h08, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h08),
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h0c, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h0c));

            $display("  LINE1: [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h10, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h10),
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h14, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h14),
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h18, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h18),
                    (dbg_dc_addr & CACHE_LINE_MASK) + 32'h1c, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h1c));

            $display("  HOST_FIFO: count=%0d rd=%0d wr=%0d",
                     host_rx_count, host_rx_rd_ptr, host_rx_wr_ptr);

            $display("  MEM[0]=%08x MEM[1]=%08x MEM[64]=%08x MEM[65]=%08x",
                     pc_mem[0], pc_mem[1], pc_mem[64], pc_mem[65]);

            $display("  FLAGS: sw=%b lw=%b sb=%b dread_byte_cnt=%0d a0_lw=%b a0_li0=%b a0_lb=%b a0_lbu=%b",
                     seen_sw_host, seen_lw_host, seen_sb_host, seen_dread_byte_host_count,
                     seen_a0_after_lw, seen_a0_after_li0, seen_a0_after_lb, seen_a0_after_lbu);
        end
    endtask

    // ------------------------------------------------------------
    // PC-side host memory emulator
    // ------------------------------------------------------------
    task host_service_one_request;
        reg [7:0]  sof;
        reg [7:0]  ptype;
        reg [7:0]  pseq;
        reg [7:0]  plen;
        reg [7:0]  pxor;
        reg [7:0]  calc_xor;
        reg [31:0] addr;
        reg [31:0] wdata;
        reg [31:0] rdata;
        reg [3:0]  wstrb;
        reg [7:0]  tag;
        reg [7:0]  status;
        integer    j;
        integer    word_index;

        reg [31:0] req_line_base;
        reg [31:0] cpu_line_base;
        integer    req_word_off;
        integer    cpu_word_off;
        reg        same_line;
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

            if (plen > 16) begin
                $display("%0t HOST ERROR: payload too large plen=%0d (max 16), stopping", $time, plen);
                print_stall_snapshot();
                $stop;
            end

            for (j = 0; j < plen; j = j + 1) begin
                host_fifo_pop(rx_payload[j]);
                calc_xor = calc_xor ^ rx_payload[j];
            end

            host_fifo_pop(pxor);

            $display("%0t HOST RX FRAME sofa5 type%02x seq%02x len%0d xorgot%02x xorcalc%02x xorok%b",
                     $time, ptype, pseq, plen, pxor, calc_xor, (pxor === calc_xor));
            print_rx_payload(plen);

            if (pxor !== calc_xor) begin
                $display("%0t HOST ERROR: BAD_XOR type=%02x seq=%02x plen=%0d got=%02x exp=%02x",
                         $time, ptype, pseq, plen, pxor, calc_xor);
            end

            addr          = 32'd0;
            wdata         = 32'd0;
            rdata         = 32'd0;
            wstrb         = 4'd0;
            tag           = 8'd0;
            status        = UA_STATUS_OK;
            word_index    = 0;
            req_line_base = 32'd0;
            cpu_line_base = 32'd0;
            req_word_off  = 0;
            cpu_word_off  = 0;
            same_line     = 1'b0;

            case (ptype)
                UA_TYPE_IREAD_REQ,
                UA_TYPE_DREAD_REQ: begin
                    if (plen != 8'd5) begin
                        status = UA_STATUS_BAD_TYPE;
                        tag    = 8'h00;
                        rdata  = 32'h00000000;
                    end else begin
                        addr = {rx_payload[3], rx_payload[2], rx_payload[1], rx_payload[0]};
                        tag  = rx_payload[4];

                        $display("%0t HOST RAW ADDR bytes: b0=%02x b1=%02x b2=%02x b3=%02x -> addr=%08x",
                                 $time, rx_payload[0], rx_payload[1], rx_payload[2], rx_payload[3], addr);

                        req_line_base = (addr & CACHE_LINE_MASK);
                        req_word_off  = addr[3:2];
                        cpu_line_base = (dbg_dmAddr & CACHE_LINE_MASK);
                        cpu_word_off  = dbg_dmAddr[3:2];
                        same_line     = (req_line_base == cpu_line_base);

                        if (ptype == UA_TYPE_DREAD_REQ) begin
                            $display("%0t HOST DREAD CACHE uart_addr=%08x uart_line=%08x uart_word_off=%0d cpu_dmAddr=%08x cpu_line=%08x cpu_word_off=%0d same_line=%0d",
                                     $time, addr, req_line_base, req_word_off, dbg_dmAddr, cpu_line_base, cpu_word_off, same_line);
                            $display("%0t HOST DREAD LINE0 [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                                    $time,
                                    req_line_base + 32'h00, mem_word_at_addr(req_line_base + 32'h00),
                                    req_line_base + 32'h04, mem_word_at_addr(req_line_base + 32'h04),
                                    req_line_base + 32'h08, mem_word_at_addr(req_line_base + 32'h08),
                                    req_line_base + 32'h0c, mem_word_at_addr(req_line_base + 32'h0c));

                            $display("%0t HOST DREAD LINE1 [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                                    $time,
                                    req_line_base + 32'h10, mem_word_at_addr(req_line_base + 32'h10),
                                    req_line_base + 32'h14, mem_word_at_addr(req_line_base + 32'h14),
                                    req_line_base + 32'h18, mem_word_at_addr(req_line_base + 32'h18),
                                    req_line_base + 32'h1c, mem_word_at_addr(req_line_base + 32'h1c));
                        end

                        if (addr[31:24] != 8'h00) begin
                            status = UA_STATUS_BAD_ADDR;
                            rdata  = 32'h00000000;
                        end else if (addr > ((MEM_WORDS * 4) - 4)) begin
                            $display("%0t HOST ERROR: addr=%08x out of range (max=%08x)",
                                     $time, addr, (MEM_WORDS * 4) - 4);
                            status = UA_STATUS_BAD_ADDR;
                            rdata  = 32'h00000000;
                        end else begin
                            word_index = addr[MEM_AW+1:2];
                            rdata = pc_mem[word_index];

                            if (ptype == UA_TYPE_DREAD_REQ)
                                $display("%0t HOST DREAD DETAIL: addr=%08x word_index=%0d mem=%08x",
                                         $time, addr, word_index, rdata);
                        end
                    end

                    if (ptype == UA_TYPE_IREAD_REQ)
                        $display("%0t HOST I READ  addr%08x tag%02x - data%08x status%02x",
                                 $time, addr, tag, rdata, status);
                    else
                        $display("%0t HOST D READ  addr%08x tag%02x - data%08x status%02x",
                                 $time, addr, tag, rdata, status);

                    if ((ptype == UA_TYPE_DREAD_REQ) && (status == UA_STATUS_OK)) begin
                        if ((addr == DATA_WORD_ADDR) && (rdata == 32'h0000000a)) begin
                            if (!seen_lw_host)
                                $display("%0t CHECK HOST: observed DREAD addr=%08x -> 0000000a", $time, addr);
                            seen_lw_host = 1'b1;
                        end

                        if ((addr == DATA_BYTE_ADDR) && (rdata[7:0] == 8'hfe)) begin
                            seen_dread_byte_host_count = seen_dread_byte_host_count + 1;
                            $display("%0t CHECK HOST: observed DREAD addr=%08x -> %08x (count=%0d)",
                                     $time, addr, rdata, seen_dread_byte_host_count);
                        end

                        if ((dbg_dmAddr !== 32'hxxxxxxxx) &&
                            ((dbg_dmAddr & CACHE_LINE_MASK) != (addr & CACHE_LINE_MASK))) begin
                            $display("%0t HOST ERROR: LINE ADDR MISMATCH! CPU dmAddr=%08x line=%08x off=%0d but UART req addr=%08x line=%08x off=%0d",
                                     $time,
                                     dbg_dmAddr, (dbg_dmAddr & CACHE_LINE_MASK), dbg_dmAddr[3:2],
                                     addr,      (addr      & CACHE_LINE_MASK), addr[3:2]);
                            print_stall_snapshot();
                            $stop;
                        end else if ((dbg_dmAddr !== 32'hxxxxxxxx) && (dbg_dmAddr != addr)) begin
                            $display("%0t HOST NOTE: line-aligned DREAD, CPU dmAddr=%08x but UART req addr=%08x (same line, different word offset)",
                                     $time, dbg_dmAddr, addr);
                        end
                    end

                    if (ptype == UA_TYPE_IREAD_REQ)
                        uart_send_read_resp(UA_TYPE_IREAD_RESP, pseq, status, tag, rdata);
                    else
                        uart_send_read_resp(UA_TYPE_DREAD_RESP, pseq, status, tag, rdata);
                end

                UA_TYPE_WRITE: begin
                    if (plen != 8'd10) begin
                        status = UA_STATUS_BAD_TYPE;
                        tag    = 8'h00;
                    end else begin
                        addr  = {rx_payload[3], rx_payload[2], rx_payload[1], rx_payload[0]};
                        wstrb = rx_payload[4][3:0];
                        tag   = rx_payload[5];
                        wdata = {rx_payload[9], rx_payload[8], rx_payload[7], rx_payload[6]};

                        $display("%0t HOST RAW WRITE ADDR bytes: b0=%02x b1=%02x b2=%02x b3=%02x -> addr=%08x wstrb=%x wdata=%08x",
                                 $time, rx_payload[0], rx_payload[1], rx_payload[2], rx_payload[3],
                                 addr, wstrb, wdata);

                        $display("%0t HOST WRITE CACHE addr=%08x line=%08x word_off=%0d",
                                 $time, addr, (addr & CACHE_LINE_MASK), addr[3:2]);

                        if (addr[31:24] != 8'h00) begin
                            status = UA_STATUS_BAD_ADDR;
                        end else if (addr > ((MEM_WORDS * 4) - 4)) begin
                            $display("%0t HOST ERROR: write addr=%08x out of range (max=%08x)",
                                     $time, addr, (MEM_WORDS * 4) - 4);
                            status = UA_STATUS_BAD_ADDR;
                        end else begin
                            word_index = addr[MEM_AW+1:2];
                            if (wstrb[0]) pc_mem[word_index][ 7: 0] = wdata[ 7: 0];
                            if (wstrb[1]) pc_mem[word_index][15: 8] = wdata[15: 8];
                            if (wstrb[2]) pc_mem[word_index][23:16] = wdata[23:16];
                            if (wstrb[3]) pc_mem[word_index][31:24] = wdata[31:24];

                            $display("%0t HOST WRITE DETAIL: addr=%08x word_index=%0d mem_after=%08x",
                                     $time, addr, word_index, pc_mem[word_index]);
                            $display("%0t HOST WRITE LINE0 [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                                    $time,
                                    (addr & CACHE_LINE_MASK) + 32'h00, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h00),
                                    (addr & CACHE_LINE_MASK) + 32'h04, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h04),
                                    (addr & CACHE_LINE_MASK) + 32'h08, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h08),
                                    (addr & CACHE_LINE_MASK) + 32'h0c, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h0c));

                            $display("%0t HOST WRITE LINE1 [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                                    $time,
                                    (addr & CACHE_LINE_MASK) + 32'h10, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h10),
                                    (addr & CACHE_LINE_MASK) + 32'h14, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h14),
                                    (addr & CACHE_LINE_MASK) + 32'h18, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h18),
                                    (addr & CACHE_LINE_MASK) + 32'h1c, mem_word_at_addr((addr & CACHE_LINE_MASK) + 32'h1c));
                        end
                    end

                    $display("%0t HOST D WRITE addr%08x tag%02x wstrb%x wdata%08x status%02x",
                             $time, addr, tag, wstrb, wdata, status);

                    if (status == UA_STATUS_OK) begin
                        if ((addr == DATA_WORD_ADDR) &&
                            (wstrb == 4'hf) &&
                            (pc_mem[DATA_WORD_ADDR[MEM_AW+1:2]] == 32'h0000000a)) begin
                            if (!seen_sw_host)
                                $display("%0t CHECK HOST: observed SW to addr=%08x, memory now %08x",
                                         $time, addr, pc_mem[DATA_WORD_ADDR[MEM_AW+1:2]]);
                            seen_sw_host = 1'b1;
                        end

                        if ((addr == DATA_BYTE_ADDR) &&
                            (wstrb == 4'b0001) &&
                            (pc_mem[DATA_BYTE_ADDR[MEM_AW+1:2]][7:0] == 8'hfe)) begin
                            if (!seen_sb_host)
                                $display("%0t CHECK HOST: observed SB to addr=%08x, memory now %08x",
                                         $time, addr, pc_mem[DATA_BYTE_ADDR[MEM_AW+1:2]]);
                            seen_sb_host = 1'b1;
                        end
                    end

                    uart_send_write_resp(UA_TYPE_WRITE_RESP, pseq, status, tag);
                end

                default: begin
                    $display("%0t HOST: UNKNOWN request type=%02x seq=%02x len=%0d",
                             $time, ptype, pseq, plen);
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
    // D-cache backend monitors (quiet mode: log only on changes)
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            prev_mon_dc_valid  <= 1'b0;
            prev_mon_dc_ready  <= 1'b0;
            prev_mon_dc_rvalid <= 1'b0;
            prev_mon_dc_addr   <= 32'hxxxxxxxx;
            prev_mon_dc_wstrb  <= 4'bxxxx;
            prev_mon_dc_wdata  <= 32'hxxxxxxxx;
            prev_mon_dc_rdata  <= 32'hxxxxxxxx;
        end else begin
            if (dbg_dc_valid &&
                (!prev_mon_dc_valid ||
                 (dbg_dc_addr   != prev_mon_dc_addr)  ||
                 (dbg_dc_wstrb  != prev_mon_dc_wstrb) ||
                 (dbg_dc_wdata  != prev_mon_dc_wdata) ||
                 (dbg_dc_ready  != prev_mon_dc_ready) ||
                 (dbg_dc_rvalid != prev_mon_dc_rvalid) ||
                 (dbg_dc_rdata  != prev_mon_dc_rdata))) begin

                $display("%0t DBG DCREQ valid%b addr%08x wstrb%x wdata%08x ready%b rvalid%b rdata%08x",
                         $time, dbg_dc_valid, dbg_dc_addr, dbg_dc_wstrb, dbg_dc_wdata,
                         dbg_dc_ready, dbg_dc_rvalid, dbg_dc_rdata);

                $display("%0t DBG DCCACHE cpu_dmAddr=%08x cpu_line=%08x cpu_off=%0d | dcaddr=%08x dc_line=%08x dc_off=%0d same_line=%0d loadpending=%0b dmfire=%0b",
                         $time,
                         dbg_dmAddr, (dbg_dmAddr & CACHE_LINE_MASK), dbg_dmAddr[3:2],
                         dbg_dc_addr, (dbg_dc_addr & CACHE_LINE_MASK), dbg_dc_addr[3:2],
                         ((dbg_dmAddr & CACHE_LINE_MASK) == (dbg_dc_addr & CACHE_LINE_MASK)),
                         dbg_load_pending, dbg_dm_fire);

                $display("%0t DBG DCLINE0 [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                        $time,
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h00, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h00),
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h04, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h04),
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h08, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h08),
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h0c, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h0c));

                $display("%0t DBG DCLINE1 [%08x]=%08x [%08x]=%08x [%08x]=%08x [%08x]=%08x",
                        $time,
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h10, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h10),
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h14, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h14),
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h18, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h18),
                        (dbg_dc_addr & CACHE_LINE_MASK) + 32'h1c, mem_word_at_addr((dbg_dc_addr & CACHE_LINE_MASK) + 32'h1c));
            end

            if (dbg_dc_rvalid &&
                (!prev_mon_dc_rvalid ||
                 (dbg_dc_addr  != prev_mon_dc_addr) ||
                 (dbg_dc_rdata != prev_mon_dc_rdata))) begin
                $display("%0t DBG DCRSP rdata%08x addr%08x line%08x off%0d",
                         $time, dbg_dc_rdata, dbg_dc_addr, (dbg_dc_addr & CACHE_LINE_MASK), dbg_dc_addr[3:2]);
            end

            if (dbg_dc_valid && (dbg_dc_wstrb != 4'b0000) && dbg_dc_ready && !prev_mon_dc_ready) begin
                $display("%0t DBG DCWRITEDONE addr%08x wdata%08x wstrb%x",
                         $time, dbg_dc_addr, dbg_dc_wdata, dbg_dc_wstrb);
            end

            if (dbg_dc_valid && (dbg_dc_wstrb == 4'b0000) && dbg_dc_rvalid && !prev_mon_dc_rvalid) begin
                $display("%0t DBG DCREADDONE addr%08x rdata%08x",
                         $time, dbg_dc_addr, dbg_dc_rdata);
            end

            prev_mon_dc_valid  <= dbg_dc_valid;
            prev_mon_dc_ready  <= dbg_dc_ready;
            prev_mon_dc_rvalid <= dbg_dc_rvalid;
            prev_mon_dc_addr   <= dbg_dc_addr;
            prev_mon_dc_wstrb  <= dbg_dc_wstrb;
            prev_mon_dc_wdata  <= dbg_dc_wdata;
            prev_mon_dc_rdata  <= dbg_dc_rdata;
        end
    end

    // ------------------------------------------------------------
    // System progress watchdog
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            sys_cycle           <= 0;
            last_progress_cycle <= 0;
            last_pc             <= 32'hxxxxxxxx;
        end else begin
            sys_cycle <= sys_cycle + 1;

            if (dbg_ic_valid || dbg_ic_rvalid ||
                dbg_dc_valid || dbg_dc_rvalid ||
                (lab_top.sm_cpu.pc != last_pc)) begin
                last_progress_cycle <= sys_cycle;
                last_pc <= lab_top.sm_cpu.pc;
            end

            if ((sys_cycle - last_progress_cycle) > 200000) begin
                $display("STALL DETECTED at sys_cycle=%0d time=%0t", sys_cycle, $time);
                print_stall_snapshot();
                $stop;
            end
        end
    end

    // ------------------------------------------------------------
    // CPU trace + test checks
    // ------------------------------------------------------------
    always @(posedge cpuClk) begin
        if (rst_n) begin
            if ((lab_top.sm_cpu.pc        !== prev_cpu_pc) ||
                (lab_top.sm_cpu.instr     !== prev_cpu_instr) ||
                (dbg_instr_valid          !== prev_cpu_instr_valid) ||
                (lab_top.sm_cpu.rf.rf[10] !== prev_a0) ||
                (dbg_dmAddr               !== prev_dbg_dm_addr) ||
                (dbg_aluResult            !== prev_dbg_alu_result) ||
                (dbg_dmWe                 !== prev_dbg_dm_we) ||
                (dbg_regWrite_ctrl        !== prev_dbg_reg_write_ctrl) ||
                (dbg_regWrite_rf          !== prev_dbg_reg_write_rf) ||
                (dbg_wdSrc                !== prev_dbg_wd_src) ||
                (dbg_pc_hold              !== prev_dbg_pc_hold) ||
                (dbg_commit               !== prev_dbg_commit) ||
                (dbg_load_pending         !== prev_dbg_load_pending) ||
                (dbg_im_pending           !== prev_dbg_im_pending) ||
                (dbg_dc_valid             !== prev_dbg_dc_valid) ||
                (dbg_dc_ready             !== prev_dbg_dc_ready) ||
                (dbg_dc_rvalid            !== prev_dbg_dc_rvalid)) begin

                $display("%6d pc%08h instr%08h instrvalid%b a0%08h",
                         cycle,
                         lab_top.sm_cpu.pc,
                         lab_top.sm_cpu.instr,
                         dbg_instr_valid,
                         lab_top.sm_cpu.rf.rf[10]);

                $display("      DEC op%02x f3%x f7%02x rd%0d rs1%0d rs2%0d rs1v%08x rs2v%08x",
                         dbg_cmdOp, dbg_cmdF3, dbg_cmdF7, dbg_rd, dbg_rs1, dbg_rs2,
                         dbg_rd1, dbg_rd2);

                $display("      IMM I%08x S%08x B%08x U%08x J%08x",
                         dbg_immI, dbg_immS, dbg_immB, dbg_immU, dbg_immJ);

                $display("      EXE aluA%08x aluB%08x aluRes%08x aluSrc1Sel%b aluSrc2Sel%b aluCtrl%x",
                         dbg_aluSrc1In, dbg_aluSrc2In, dbg_aluResult,
                         dbg_aluSrc1Sel, dbg_aluSrc2Sel, dbg_aluControl);

                $display("      LSU load%b store%b lb%b lbu%b lw%b sb%b sw%b dmWe%b dmSign%b opB%b opH%b opW%b",
                         dbg_is_load, dbg_is_store, dbg_is_lb, dbg_is_lbu, dbg_is_lw,
                         dbg_is_sb, dbg_is_sw, dbg_dmWe, dbg_dmSign, dbg_op_byte, dbg_op_half, dbg_op_word);

                $display("      MEM dmAddr%08x dmDataW%08x dmDataR%08x dmfire%b dcvalid%b dcready%b dcrvalid%b dcaddr%08x dcwstrb%x dcwdata%08x dcrdata%08x",
                         dbg_dmAddr, dbg_dmDataW, dbg_dmDataR, dbg_dm_fire,
                         dbg_dc_valid, dbg_dc_ready, dbg_dc_rvalid,
                         dbg_dc_addr, dbg_dc_wstrb, dbg_dc_wdata, dbg_dc_rdata);

                $display("      WB regWritectrl%b regWriterf%b wdSrc%0d wd3%08x pchold%b commit%b loadpending%b impending%b pcSrc1%0d pcSrc2%b pcSrc1In%08x pcSrc2In%08x pcNext%08x",
                         dbg_regWrite_ctrl, dbg_regWrite_rf, dbg_wdSrc, dbg_wd3,
                         dbg_pc_hold, dbg_commit, dbg_load_pending, dbg_im_pending,
                         dbg_pcSrc1, dbg_pcSrc2, dbg_pcSrc1In, dbg_pcSrc2In, dbg_pcNext);
            end

            if (!seen_a0_after_lw && (lab_top.sm_cpu.rf.rf[10] == 32'h0000000a)) begin
                seen_a0_after_lw <= 1'b1;
                $display("%0t CHECK CPU: a0 after lw = 0000000a", $time);
            end else if (seen_a0_after_lw && !seen_a0_after_li0 &&
                         (lab_top.sm_cpu.rf.rf[10] == 32'h00000000)) begin
                seen_a0_after_li0 <= 1'b1;
                $display("%0t CHECK CPU: a0 after li 0 = 00000000", $time);
            end else if (seen_a0_after_li0 && !seen_a0_after_lb &&
                         (lab_top.sm_cpu.rf.rf[10] == 32'hfffffffe)) begin
                seen_a0_after_lb <= 1'b1;
                $display("%0t CHECK CPU: a0 after lb = fffffffe", $time);
            end else if (seen_a0_after_lb && !seen_a0_after_lbu &&
                         (lab_top.sm_cpu.rf.rf[10] == 32'h000000fe)) begin
                seen_a0_after_lbu <= 1'b1;
                $display("%0t CHECK CPU: a0 after lbu = 000000fe", $time);
            end

            if (!test_passed &&
                seen_sw_host &&
                seen_lw_host &&
                seen_sb_host &&
                seen_a0_after_lw &&
                seen_a0_after_li0 &&
                seen_a0_after_lb &&
                seen_a0_after_lbu) begin
                test_passed = 1'b1;
                $display("");
                $display("TEST PASS");
                $display("  mem[0x104] = %08x", pc_mem[DATA_WORD_ADDR[MEM_AW+1:2]]);
                $display("  mem[0x100] = %08x", pc_mem[DATA_BYTE_ADDR[MEM_AW+1:2]]);
                $display("  DREAD byte addr count = %0d (informational only)", seen_dread_byte_host_count);
                $display("");
                $stop;
            end

            prev_cpu_pc             <= lab_top.sm_cpu.pc;
            prev_cpu_instr          <= lab_top.sm_cpu.instr;
            prev_cpu_instr_valid    <= dbg_instr_valid;
            prev_a0                 <= lab_top.sm_cpu.rf.rf[10];
            prev_dbg_dm_addr        <= dbg_dmAddr;
            prev_dbg_alu_result     <= dbg_aluResult;
            prev_dbg_dm_we          <= dbg_dmWe;
            prev_dbg_reg_write_ctrl <= dbg_regWrite_ctrl;
            prev_dbg_reg_write_rf   <= dbg_regWrite_rf;
            prev_dbg_wd_src         <= dbg_wdSrc;
            prev_dbg_pc_hold        <= dbg_pc_hold;
            prev_dbg_commit         <= dbg_commit;
            prev_dbg_load_pending   <= dbg_load_pending;
            prev_dbg_im_pending     <= dbg_im_pending;
            prev_dbg_dc_valid       <= dbg_dc_valid;
            prev_dbg_dc_ready       <= dbg_dc_ready;
            prev_dbg_dc_rvalid      <= dbg_dc_rvalid;

            cycle <= cycle + 1;

            if (cycle > `SIMULATION_CYCLES) begin
                $display("Timeout");
                print_stall_snapshot();
                $stop;
            end
        end
    end

endmodule