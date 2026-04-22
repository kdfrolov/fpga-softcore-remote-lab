module mux #(
	parameter WIDTH_MUX = 2,
	parameter WIDTH_IN  = 2
)(
	input  [WIDTH_MUX -1:0] mux_select,
	input  [WIDTH_MUX-1:0][WIDTH_IN-1:0] mux_input,
	output [WIDTH_IN-1:0]               mux_output
);

generate
	genvar i;
	for (i=0, i<WIDTH_MUX, i++)
	begin
		if (mux_select[i])
			mux_output = mux_input[i];
	end
	
endgenerate

always_comb begin
	mux_output = mux_input[];
end

endmodule

