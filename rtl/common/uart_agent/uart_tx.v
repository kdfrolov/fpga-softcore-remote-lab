module uart_tx
#(
    parameter CLK_HZ = 50000000,
    parameter BAUD   = 115200
)(
    input            clk,
    input            rst_n,
    input      [7:0] data_i,
    input            valid_i,
    output           ready_o,
    output           txd_o
);
    localparam [31:0] CLKS_PER_BIT = CLK_HZ / BAUD;

    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [31:0] clk_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  shreg;
    reg        txd_reg;

    assign ready_o = (state == S_IDLE);
    assign txd_o   = txd_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 32'd0;
            bit_cnt <= 3'd0;
            shreg   <= 8'd0;
            txd_reg <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    txd_reg <= 1'b1;
                    if (valid_i) begin
                        shreg   <= data_i;
                        clk_cnt <= 32'd0;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    txd_reg <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;
                        bit_cnt <= 3'd0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    txd_reg <= shreg[0];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;
                        shreg   <= {1'b0, shreg[7:1]};
                        if (bit_cnt == 3'd7)
                            state <= S_STOP;
                        else
                            bit_cnt <= bit_cnt + 1'b1;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                S_STOP: begin
                    txd_reg <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule