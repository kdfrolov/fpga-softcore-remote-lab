/*

 */ 

//lab hardware top level module
module lab_top
(
    input           clkIn,
    input           rst_n,
    input   [ 3:0 ] clkDivide,
    input           clkEnable,
    output          clk,
    input   [ 4:0 ] regAddr,
    output  [31:0 ] regData
);
    //metastability input filters
    wire    [ 3:0 ] divide;
    wire            enable;
    wire    [ 4:0 ] addr;

    sm_debouncer #(.SIZE(4)) f0(clkIn, clkDivide, divide);
    sm_debouncer #(.SIZE(1)) f1(clkIn, clkEnable, enable);
    sm_debouncer #(.SIZE(5)) f2(clkIn, regAddr,   addr  );

    //cores
    //clock divider
    sm_clk_divider sm_clk_divider
    (
        .clkIn      ( clkIn     ),
        .rst_n      ( rst_n     ),
        .divide     ( divide    ),
        .enable     ( enable    ),
        .clkOut     ( clk       )
    );

    // data memory
    wire [31:0] data_addr;
    wire [31:0] write_data;
    wire [31:0] read_data;
    wire        we;
    wire        sign;
    wire        op_byte;
    wire        op_half;
    wire        op_word;

    sr_ram #(
        .DEPTH       ( 1024       )
    ) ram (
        .clk         ( clk        ),
        .data_addr   ( data_addr  ),
        .write_data  ( write_data ),
        .we          ( we         ),
        .sign        ( sign       ),
        .op_byte     ( op_byte    ),
        .op_half     ( op_half    ),
        .op_word     ( op_word    ),
        .read_data   ( read_data  )
    );

    //instruction memory
    wire    [31:0]  imAddr;
    wire    [31:0]  imData;
    sm_rom #(.SIZE (1024)) reset_rom (imAddr, imData);

    sr_cpu sm_cpu (
        .clk        ( clk        ),
        .rst_n      ( rst_n      ),
        .regAddr    ( addr       ),
        .regData    ( regData    ),
        .imAddr     ( imAddr     ),
        .imData     ( imData     ),

        .dmAddr     ( data_addr  ),
        .dmDataW    ( write_data ),
        .dmWe       ( we         ),
        .op_byte    ( op_byte    ),
        .op_half    ( op_half    ),
        .op_word    ( op_word    ),
        .dmSign     ( sign       ),
        .dmDataR    ( read_data  )
    );

endmodule

//metastability input debouncer module
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

//tunable clock divider
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
