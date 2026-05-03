`include "uart_agent_proto.vh"

module uart_mem_agent_2clk
#(
    parameter UART_CLK_HZ       = 50000000,
    parameter UART_BAUD         = 115200,
    parameter UART_TIMEOUT_CLKS = 5000000
)(
    input             rst_n,

    // -----------------------------
    // Stable UART clock domain
    // -----------------------------
    input             uart_clk,
    output            uart_txd_o,
    input             uart_rxd_i,

    // -----------------------------
    // Variable core clock domain
    // -----------------------------
    input             core_clk,

    // I-side request channel (core domain)
    input             ic_valid_i,
    input  [31:0]     ic_addr_i,
    output            ic_ready_o,
    output reg        ic_rvalid_o,
    output reg [31:0] ic_rdata_o,

    // D-side request channel (core domain)
    input             dc_valid_i,
    input  [31:0]     dc_addr_i,
    input  [31:0]     dc_wdata_i,
    input  [3:0]      dc_wstrb_i,
    output            dc_ready_o,
    output reg        dc_rvalid_o,
    output reg [31:0] dc_rdata_o,

    // Debug
    output [2:0]      dbg_uart_state_o,
    output [3:0]      dbg_rx_state_o,
    output            dbg_core_busy_o
);

    // ==========================================================
    // Core-domain request context
    // ==========================================================
    reg        core_busy;

    reg [7:0]  cdc_req_type;
    reg [7:0]  cdc_req_tag;
    reg [31:0] cdc_req_addr;
    reg [31:0] cdc_req_wdata;
    reg [3:0]  cdc_req_wstrb;
    reg        req_toggle_core;

    // Active transaction context used to retire response
    reg [7:0]  active_req_type;
    reg [7:0]  active_req_tag;
    reg [31:0] active_req_addr;
    reg [31:0] active_req_wdata;
    reg [3:0]  active_req_wstrb;

    // One-entry pending buffer in core domain
    reg        pend_valid;
    reg [7:0]  pend_req_type;
    reg [7:0]  pend_req_tag;
    reg [31:0] pend_req_addr;
    reg [31:0] pend_req_wdata;
    reg [3:0]  pend_req_wstrb;

    // ----------------------------------------------------------
    // NEW: "accepted once" tracking for level-valid sources
    // Prevent the same still-asserted request from being
    // accepted multiple times.
    // ----------------------------------------------------------
    reg        dc_seen_valid;
    reg [31:0] dc_seen_addr;
    reg [31:0] dc_seen_wdata;
    reg [3:0]  dc_seen_wstrb;

    reg        ic_seen_valid;
    reg [31:0] ic_seen_addr;

    wire dc_same_seen;
    wire ic_same_seen;

    assign dc_same_seen =
        dc_valid_i &&
        dc_seen_valid &&
        (dc_addr_i  == dc_seen_addr) &&
        (dc_wdata_i == dc_seen_wdata) &&
        (dc_wstrb_i == dc_seen_wstrb);

    assign ic_same_seen =
        ic_valid_i &&
        ic_seen_valid &&
        (ic_addr_i == ic_seen_addr);

    // ==========================================================
    // UART-domain CDC response bundle
    // ==========================================================
    reg [7:0]  cdc_resp_status;
    reg [31:0] cdc_resp_data;
    reg        resp_toggle_uart;

    // ==========================================================
    // Toggle synchronizers
    // ==========================================================
    reg req_sync1_uart,  req_sync2_uart,  req_seen_uart;
    reg resp_sync1_core, resp_sync2_core, resp_seen_core;

    assign dbg_core_busy_o = core_busy;

    // ==========================================================
    // Core-side ready
    // Allow one in-flight request plus one queued request,
    // but do not re-accept the exact same still-held request.
    // D-side keeps priority over I-side.
    // ==========================================================
    assign dc_ready_o = !pend_valid && !dc_same_seen;
    assign ic_ready_o = !pend_valid && !dc_valid_i && !ic_same_seen;

    // ==========================================================
    // Core domain logic
    // ==========================================================
    always @(posedge core_clk or negedge rst_n) begin
        if (!rst_n) begin
            core_busy         <= 1'b0;

            cdc_req_type      <= 8'd0;
            cdc_req_tag       <= 8'd0;
            cdc_req_addr      <= 32'd0;
            cdc_req_wdata     <= 32'd0;
            cdc_req_wstrb     <= 4'd0;
            req_toggle_core   <= 1'b0;

            active_req_type   <= 8'd0;
            active_req_tag    <= 8'd0;
            active_req_addr   <= 32'd0;
            active_req_wdata  <= 32'd0;
            active_req_wstrb  <= 4'd0;

            pend_valid        <= 1'b0;
            pend_req_type     <= 8'd0;
            pend_req_tag      <= 8'd0;
            pend_req_addr     <= 32'd0;
            pend_req_wdata    <= 32'd0;
            pend_req_wstrb    <= 4'd0;

            dc_seen_valid     <= 1'b0;
            dc_seen_addr      <= 32'd0;
            dc_seen_wdata     <= 32'd0;
            dc_seen_wstrb     <= 4'd0;

            ic_seen_valid     <= 1'b0;
            ic_seen_addr      <= 32'd0;

            resp_sync1_core   <= 1'b0;
            resp_sync2_core   <= 1'b0;
            resp_seen_core    <= 1'b0;

            ic_rvalid_o       <= 1'b0;
            ic_rdata_o        <= 32'd0;
            dc_rvalid_o       <= 1'b0;
            dc_rdata_o        <= 32'd0;
        end else begin
            ic_rvalid_o <= 1'b0;
            dc_rvalid_o <= 1'b0;

            resp_sync1_core <= resp_toggle_uart;
            resp_sync2_core <= resp_sync1_core;

            // Release "seen" when valid drops, or when payload changes
            // while valid stays high (back-to-back request case).
            if (!dc_valid_i) begin
                dc_seen_valid <= 1'b0;
            end else if (dc_seen_valid &&
                         ((dc_addr_i  != dc_seen_addr)  ||
                          (dc_wdata_i != dc_seen_wdata) ||
                          (dc_wstrb_i != dc_seen_wstrb))) begin
                dc_seen_valid <= 1'b0;
            end

            if (!ic_valid_i) begin
                ic_seen_valid <= 1'b0;
            end else if (ic_seen_valid &&
                         (ic_addr_i != ic_seen_addr)) begin
                ic_seen_valid <= 1'b0;
            end

            // ------------------------------------------
            // Response arrival from UART domain
            // Retire active request and, if possible,
            // immediately launch the next one.
            // ------------------------------------------
            if (resp_sync2_core != resp_seen_core) begin
                resp_seen_core <= resp_sync2_core;

                case (active_req_type)
                    `UA_TYPE_IREAD_REQ: begin
                        ic_rvalid_o <= 1'b1;
                        if (cdc_resp_status == `UA_STATUS_OK)
                            ic_rdata_o <= cdc_resp_data;
                        else
                            ic_rdata_o <= 32'h00000013;
                    end

                    `UA_TYPE_DREAD_REQ: begin
                        dc_rvalid_o <= 1'b1;
                        if (cdc_resp_status == `UA_STATUS_OK)
                            dc_rdata_o <= cdc_resp_data;
                        else
                            dc_rdata_o <= 32'hDEADBEEF;
                    end

                    `UA_TYPE_WRITE: begin
                    end

                    default: begin
                    end
                endcase

                // Launch next request immediately if one is pending.
                if (pend_valid) begin
                    cdc_req_type    <= pend_req_type;
                    cdc_req_tag     <= pend_req_tag;
                    cdc_req_addr    <= pend_req_addr;
                    cdc_req_wdata   <= pend_req_wdata;
                    cdc_req_wstrb   <= pend_req_wstrb;
                    req_toggle_core <= ~req_toggle_core;

                    active_req_type  <= pend_req_type;
                    active_req_tag   <= pend_req_tag;
                    active_req_addr  <= pend_req_addr;
                    active_req_wdata <= pend_req_wdata;
                    active_req_wstrb <= pend_req_wstrb;

                    pend_valid <= 1'b0;
                    core_busy  <= 1'b1;

                end else if (dc_valid_i && !dc_same_seen) begin
                    cdc_req_addr    <= dc_addr_i;
                    cdc_req_wdata   <= dc_wdata_i;
                    cdc_req_wstrb   <= dc_wstrb_i;
                    cdc_req_tag     <= `UA_TAG_DCACHE;
                    cdc_req_type    <= (|dc_wstrb_i) ? `UA_TYPE_WRITE : `UA_TYPE_DREAD_REQ;
                    req_toggle_core <= ~req_toggle_core;

                    active_req_addr  <= dc_addr_i;
                    active_req_wdata <= dc_wdata_i;
                    active_req_wstrb <= dc_wstrb_i;
                    active_req_tag   <= `UA_TAG_DCACHE;
                    active_req_type  <= (|dc_wstrb_i) ? `UA_TYPE_WRITE : `UA_TYPE_DREAD_REQ;

                    dc_seen_valid <= 1'b1;
                    dc_seen_addr  <= dc_addr_i;
                    dc_seen_wdata <= dc_wdata_i;
                    dc_seen_wstrb <= dc_wstrb_i;

                    core_busy <= 1'b1;

                end else if (ic_valid_i && !ic_same_seen) begin
                    cdc_req_addr    <= ic_addr_i;
                    cdc_req_wdata   <= 32'd0;
                    cdc_req_wstrb   <= 4'd0;
                    cdc_req_tag     <= `UA_TAG_ICACHE;
                    cdc_req_type    <= `UA_TYPE_IREAD_REQ;
                    req_toggle_core <= ~req_toggle_core;

                    active_req_addr  <= ic_addr_i;
                    active_req_wdata <= 32'd0;
                    active_req_wstrb <= 4'd0;
                    active_req_tag   <= `UA_TAG_ICACHE;
                    active_req_type  <= `UA_TYPE_IREAD_REQ;

                    ic_seen_valid <= 1'b1;
                    ic_seen_addr  <= ic_addr_i;

                    core_busy <= 1'b1;

                end else begin
                    core_busy <= 1'b0;
                end

            // ------------------------------------------
            // No response this cycle
            // ------------------------------------------
            end else begin
                if (!core_busy) begin
                    // Idle: launch directly
                    if (dc_valid_i && !dc_same_seen) begin
                        core_busy       <= 1'b1;

                        cdc_req_addr    <= dc_addr_i;
                        cdc_req_wdata   <= dc_wdata_i;
                        cdc_req_wstrb   <= dc_wstrb_i;
                        cdc_req_tag     <= `UA_TAG_DCACHE;
                        cdc_req_type    <= (|dc_wstrb_i) ? `UA_TYPE_WRITE : `UA_TYPE_DREAD_REQ;
                        req_toggle_core <= ~req_toggle_core;

                        active_req_addr  <= dc_addr_i;
                        active_req_wdata <= dc_wdata_i;
                        active_req_wstrb <= dc_wstrb_i;
                        active_req_tag   <= `UA_TAG_DCACHE;
                        active_req_type  <= (|dc_wstrb_i) ? `UA_TYPE_WRITE : `UA_TYPE_DREAD_REQ;

                        dc_seen_valid <= 1'b1;
                        dc_seen_addr  <= dc_addr_i;
                        dc_seen_wdata <= dc_wdata_i;
                        dc_seen_wstrb <= dc_wstrb_i;

                    end else if (ic_valid_i && !ic_same_seen) begin
                        core_busy       <= 1'b1;

                        cdc_req_addr    <= ic_addr_i;
                        cdc_req_wdata   <= 32'd0;
                        cdc_req_wstrb   <= 4'd0;
                        cdc_req_tag     <= `UA_TAG_ICACHE;
                        cdc_req_type    <= `UA_TYPE_IREAD_REQ;
                        req_toggle_core <= ~req_toggle_core;

                        active_req_addr  <= ic_addr_i;
                        active_req_wdata <= 32'd0;
                        active_req_wstrb <= 4'd0;
                        active_req_tag   <= `UA_TAG_ICACHE;
                        active_req_type  <= `UA_TYPE_IREAD_REQ;

                        ic_seen_valid <= 1'b1;
                        ic_seen_addr  <= ic_addr_i;
                    end

                end else begin
                    // Busy: queue one request if pending slot is free
                    if (!pend_valid) begin
                        if (dc_valid_i && !dc_same_seen) begin
                            pend_valid     <= 1'b1;
                            pend_req_addr  <= dc_addr_i;
                            pend_req_wdata <= dc_wdata_i;
                            pend_req_wstrb <= dc_wstrb_i;
                            pend_req_tag   <= `UA_TAG_DCACHE;
                            pend_req_type  <= (|dc_wstrb_i) ? `UA_TYPE_WRITE : `UA_TYPE_DREAD_REQ;

                            dc_seen_valid <= 1'b1;
                            dc_seen_addr  <= dc_addr_i;
                            dc_seen_wdata <= dc_wdata_i;
                            dc_seen_wstrb <= dc_wstrb_i;

                        end else if (ic_valid_i && !ic_same_seen) begin
                            pend_valid     <= 1'b1;
                            pend_req_addr  <= ic_addr_i;
                            pend_req_wdata <= 32'd0;
                            pend_req_wstrb <= 4'd0;
                            pend_req_tag   <= `UA_TAG_ICACHE;
                            pend_req_type  <= `UA_TYPE_IREAD_REQ;

                            ic_seen_valid <= 1'b1;
                            ic_seen_addr  <= ic_addr_i;
                        end
                    end
                end
            end
        end
    end

    // ==========================================================
    // UART TX/RX
    // ==========================================================
    reg        tx_valid;
    reg [7:0]  tx_data;
    wire       tx_ready;

    wire [7:0] rx_byte;
    wire       rx_valid;

    uart_tx #(
        .CLK_HZ (UART_CLK_HZ),
        .BAUD   (UART_BAUD)
    ) u_tx (
        .clk     (uart_clk),
        .rst_n   (rst_n),
        .data_i  (tx_data),
        .valid_i (tx_valid),
        .ready_o (tx_ready),
        .txd_o   (uart_txd_o)
    );

    uart_rx #(
        .CLK_HZ (UART_CLK_HZ),
        .BAUD   (UART_BAUD)
    ) u_rx (
        .clk     (uart_clk),
        .rst_n   (rst_n),
        .rxd_i   (uart_rxd_i),
        .data_o  (rx_byte),
        .valid_o (rx_valid)
    );

    // ==========================================================
    // UART domain state
    // ==========================================================
    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_BUILD_XOR = 3'd1,
        ST_TX        = 3'd2,
        ST_WAIT_RESP = 3'd3;

    localparam [3:0]
        RX_SOF      = 4'd0,
        RX_TYPE     = 4'd1,
        RX_SEQ      = 4'd2,
        RX_LEN      = 4'd3,
        RX_PAYLOAD  = 4'd4,
        RX_XOR      = 4'd5;

    reg [2:0] uart_state;
    reg [3:0] rx_state;

    assign dbg_uart_state_o = uart_state;
    assign dbg_rx_state_o   = rx_state;

    reg [7:0]  req_type_uart;
    reg [7:0]  req_tag_uart;
    reg [31:0] req_addr_uart;
    reg [31:0] req_wdata_uart;
    reg [3:0]  req_wstrb_uart;
    reg [7:0]  req_seq_uart;
    reg [7:0]  seq_ctr_uart;

    reg [7:0] txbuf [0:14];
    reg [4:0] tx_total;
    reg [4:0] tx_idx;

    reg [31:0] timeout_cnt;

    reg [7:0] rx_type;
    reg [7:0] rx_seq;
    reg [7:0] rx_len;
    reg [7:0] rx_pl_idx;
    reg [7:0] rx_xor_acc;
    reg [7:0] rx_pl [0:9];

    reg [7:0] expected_resp_type;

    reg [7:0] xor_checksum;
    integer xor_i;

    wire [7:0]  resp_status = rx_pl[0];
    wire [31:0] resp_data   = {rx_pl[5], rx_pl[4], rx_pl[3], rx_pl[2]};

    always @(*) begin
        case (req_type_uart)
            `UA_TYPE_IREAD_REQ: expected_resp_type = `UA_TYPE_IREAD_RESP;
            `UA_TYPE_DREAD_REQ: expected_resp_type = `UA_TYPE_DREAD_RESP;
            `UA_TYPE_WRITE:     expected_resp_type = `UA_TYPE_WRITE_RESP;
            default:            expected_resp_type = 8'hFF;
        endcase
    end

    always @(*) begin
        xor_checksum = 8'h00;
        if (tx_total > 5'd2) begin
            for (xor_i = 1; xor_i < 14; xor_i = xor_i + 1) begin
                if (xor_i <= (tx_total - 2))
                    xor_checksum = xor_checksum ^ txbuf[xor_i];
            end
        end
    end

    // ==========================================================
    // UART domain FSM
    // ==========================================================
    always @(posedge uart_clk or negedge rst_n) begin
        if (!rst_n) begin
            req_sync1_uart <= 1'b0;
            req_sync2_uart <= 1'b0;
            req_seen_uart  <= 1'b0;

            uart_state <= ST_IDLE;
            rx_state   <= RX_SOF;

            req_type_uart  <= 8'd0;
            req_tag_uart   <= 8'd0;
            req_addr_uart  <= 32'd0;
            req_wdata_uart <= 32'd0;
            req_wstrb_uart <= 4'd0;
            req_seq_uart   <= 8'd0;
            seq_ctr_uart   <= 8'd0;

            tx_total <= 5'd0;
            tx_idx   <= 5'd0;
            tx_valid <= 1'b0;
            tx_data  <= 8'd0;

            timeout_cnt <= 32'd0;

            rx_type    <= 8'd0;
            rx_seq     <= 8'd0;
            rx_len     <= 8'd0;
            rx_pl_idx  <= 8'd0;
            rx_xor_acc <= 8'd0;

            cdc_resp_status  <= 8'd0;
            cdc_resp_data    <= 32'd0;
            resp_toggle_uart <= 1'b0;

            txbuf[0]  <= 8'd0; txbuf[1]  <= 8'd0; txbuf[2]  <= 8'd0;
            txbuf[3]  <= 8'd0; txbuf[4]  <= 8'd0; txbuf[5]  <= 8'd0;
            txbuf[6]  <= 8'd0; txbuf[7]  <= 8'd0; txbuf[8]  <= 8'd0;
            txbuf[9]  <= 8'd0; txbuf[10] <= 8'd0; txbuf[11] <= 8'd0;
            txbuf[12] <= 8'd0; txbuf[13] <= 8'd0; txbuf[14] <= 8'd0;

            rx_pl[0] <= 8'd0; rx_pl[1] <= 8'd0; rx_pl[2] <= 8'd0;
            rx_pl[3] <= 8'd0; rx_pl[4] <= 8'd0; rx_pl[5] <= 8'd0;
            rx_pl[6] <= 8'd0; rx_pl[7] <= 8'd0; rx_pl[8] <= 8'd0;
            rx_pl[9] <= 8'd0;
        end else begin
            tx_valid <= 1'b0;

            req_sync1_uart <= req_toggle_core;
            req_sync2_uart <= req_sync1_uart;

            case (uart_state)
                ST_IDLE: begin
                    timeout_cnt <= 32'd0;
                    rx_state    <= RX_SOF;
                    rx_pl_idx   <= 8'd0;
                    rx_xor_acc  <= 8'd0;

                    if (req_sync2_uart != req_seen_uart) begin
                        req_seen_uart <= req_sync2_uart;

                        req_type_uart  <= cdc_req_type;
                        req_tag_uart   <= cdc_req_tag;
                        req_addr_uart  <= cdc_req_addr;
                        req_wdata_uart <= cdc_req_wdata;
                        req_wstrb_uart <= cdc_req_wstrb;
                        req_seq_uart   <= seq_ctr_uart;
                        seq_ctr_uart   <= seq_ctr_uart + 1'b1;

                        txbuf[0] <= `UA_SOF;
                        txbuf[1] <= cdc_req_type;
                        txbuf[2] <= seq_ctr_uart;

                        txbuf[4] <= cdc_req_addr[7:0];
                        txbuf[5] <= cdc_req_addr[15:8];
                        txbuf[6] <= cdc_req_addr[23:16];
                        txbuf[7] <= cdc_req_addr[31:24];

                        if (cdc_req_type == `UA_TYPE_WRITE) begin
                            txbuf[3]  <= `UA_PLEN_WRITE_REQ;
                            txbuf[8]  <= {4'b0000, cdc_req_wstrb};
                            txbuf[9]  <= cdc_req_tag;
                            txbuf[10] <= cdc_req_wdata[7:0];
                            txbuf[11] <= cdc_req_wdata[15:8];
                            txbuf[12] <= cdc_req_wdata[23:16];
                            txbuf[13] <= cdc_req_wdata[31:24];
                            tx_total  <= 5'd15;
                        end else begin
                            txbuf[3] <= `UA_PLEN_READ_REQ;
                            txbuf[8] <= cdc_req_tag;
                            tx_total <= 5'd10;
                        end

                        uart_state <= ST_BUILD_XOR;
                    end
                end

                ST_BUILD_XOR: begin
                    txbuf[tx_total - 1] <= xor_checksum;
                    tx_idx     <= 5'd0;
                    uart_state <= ST_TX;
                end

                ST_TX: begin
                    if (tx_ready && !tx_valid) begin
                        tx_data  <= txbuf[tx_idx];
                        tx_valid <= 1'b1;

                        if (tx_idx == tx_total - 1) begin
                            timeout_cnt <= 32'd0;
                            rx_state    <= RX_SOF;
                            rx_pl_idx   <= 8'd0;
                            rx_xor_acc  <= 8'd0;
                            uart_state  <= ST_WAIT_RESP;
                        end else begin
                            tx_idx <= tx_idx + 1'b1;
                        end
                    end
                end

                ST_WAIT_RESP: begin
                    timeout_cnt <= timeout_cnt + 1'b1;

                    if (timeout_cnt >= UART_TIMEOUT_CLKS) begin
                        cdc_resp_status <= `UA_STATUS_TIMEOUT;
                        if (req_type_uart == `UA_TYPE_IREAD_REQ)
                            cdc_resp_data <= 32'h00000013;
                        else if (req_type_uart == `UA_TYPE_DREAD_REQ)
                            cdc_resp_data <= 32'hDEADBEEF;
                        else
                            cdc_resp_data <= 32'd0;

                        resp_toggle_uart <= ~resp_toggle_uart;
                        uart_state       <= ST_IDLE;

                    end else if (rx_valid) begin
                        case (rx_state)
                            RX_SOF: begin
                                if (rx_byte == `UA_SOF) begin
                                    rx_xor_acc <= 8'd0;
                                    rx_state   <= RX_TYPE;
                                end
                            end

                            RX_TYPE: begin
                                rx_type    <= rx_byte;
                                rx_xor_acc <= rx_byte;
                                rx_state   <= RX_SEQ;
                            end

                            RX_SEQ: begin
                                rx_seq     <= rx_byte;
                                rx_xor_acc <= rx_xor_acc ^ rx_byte;
                                rx_state   <= RX_LEN;
                            end

                            RX_LEN: begin
                                rx_len     <= rx_byte;
                                rx_xor_acc <= rx_xor_acc ^ rx_byte;
                                rx_pl_idx  <= 8'd0;

                                if (rx_byte == 8'd0)
                                    rx_state <= RX_XOR;
                                else if (rx_byte <= 8'd10)
                                    rx_state <= RX_PAYLOAD;
                                else
                                    rx_state <= RX_SOF;
                            end

                            RX_PAYLOAD: begin
                                rx_pl[rx_pl_idx] <= rx_byte;
                                rx_xor_acc       <= rx_xor_acc ^ rx_byte;

                                if (rx_pl_idx == rx_len - 1)
                                    rx_state <= RX_XOR;
                                else
                                    rx_pl_idx <= rx_pl_idx + 1'b1;
                            end

                            RX_XOR: begin
                                if ((rx_xor_acc == rx_byte) &&
                                    (rx_seq     == req_seq_uart) &&
                                    (rx_type    == expected_resp_type)) begin

                                    if ((req_type_uart == `UA_TYPE_IREAD_REQ ||
                                         req_type_uart == `UA_TYPE_DREAD_REQ) &&
                                        (rx_len == `UA_PLEN_READ_RESP)) begin
                                        cdc_resp_status  <= resp_status;
                                        cdc_resp_data    <= resp_data;
                                        resp_toggle_uart <= ~resp_toggle_uart;
                                        uart_state       <= ST_IDLE;

                                    end else if ((req_type_uart == `UA_TYPE_WRITE) &&
                                                 (rx_len == `UA_PLEN_WRITE_RESP)) begin
                                        cdc_resp_status  <= resp_status;
                                        cdc_resp_data    <= 32'd0;
                                        resp_toggle_uart <= ~resp_toggle_uart;
                                        uart_state       <= ST_IDLE;

                                    end else begin
                                        rx_state <= RX_SOF;
                                    end
                                end else begin
                                    rx_state <= RX_SOF;
                                end
                            end

                            default: begin
                                rx_state <= RX_SOF;
                            end
                        endcase
                    end
                end

                default: begin
                    uart_state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule