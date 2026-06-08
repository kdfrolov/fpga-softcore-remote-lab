module uart_rx
#(
    parameter CLK_HZ = 50000000,
    parameter BAUD   = 115200
)(
    input            clk,
    input            rst_n,
    input            rxd_i,
    output reg [7:0] data_o,
    output reg       valid_o
);
    localparam [31:0] CLKS_PER_BIT  = CLK_HZ / BAUD;
    localparam [31:0] HALF_BIT_CLKS = CLKS_PER_BIT / 2;

    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [31:0] clk_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  shreg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 32'd0;
            bit_cnt <= 3'd0;
            shreg   <= 8'd0;
            data_o  <= 8'd0;
            valid_o <= 1'b0;
        end else begin
            valid_o <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (!rxd_i) begin
                        clk_cnt <= HALF_BIT_CLKS - 1;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    if (clk_cnt == 32'd0) begin
                        if (!rxd_i) begin
                            clk_cnt <= CLKS_PER_BIT - 1;
                            bit_cnt <= 3'd0;
                            state   <= S_DATA;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt - 1'b1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == 32'd0) begin
                        shreg   <= {rxd_i, shreg[7:1]};
                        clk_cnt <= CLKS_PER_BIT - 1;
                        if (bit_cnt == 3'd7)
                            state <= S_STOP;
                        else
                            bit_cnt <= bit_cnt + 1'b1;
                    end else begin
                        clk_cnt <= clk_cnt - 1'b1;
                    end
                end

                S_STOP: begin
                    if (clk_cnt == 32'd0) begin
                        state <= S_IDLE;
                        if (rxd_i) begin
                            data_o  <= shreg;
                            valid_o <= 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt - 1'b1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule