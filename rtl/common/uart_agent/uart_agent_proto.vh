`ifndef UART_AGENT_PROTO_VH
`define UART_AGENT_PROTO_VH

// -------------------------------------------------------
// Frame
// -------------------------------------------------------
`define UA_SOF                 8'hA5

// -------------------------------------------------------
// Packet types: FPGA -> PC (memory requests)
// -------------------------------------------------------
`define UA_TYPE_IREAD_REQ      8'h01
`define UA_TYPE_DREAD_REQ      8'h02
`define UA_TYPE_WRITE          8'h03

// -------------------------------------------------------
// Packet types: PC -> FPGA (memory responses)
// -------------------------------------------------------
`define UA_TYPE_IREAD_RESP     8'h81
`define UA_TYPE_DREAD_RESP     8'h82
`define UA_TYPE_WRITE_RESP     8'h83

// -------------------------------------------------------
// Packet types: PC -> FPGA (control requests)
// -------------------------------------------------------
`define UA_TYPE_SET_CLKDIV_REQ 8'h10
`define UA_TYPE_SET_HOLD_REQ   8'h11
`define UA_TYPE_CORE_RESET_REQ 8'h12
`define UA_TYPE_SET_REGSEL_REQ 8'h13

// -------------------------------------------------------
// Packet types: FPGA -> PC (control responses)
// -------------------------------------------------------
`define UA_TYPE_SET_CLKDIV_RESP 8'h90
`define UA_TYPE_SET_HOLD_RESP   8'h91
`define UA_TYPE_CORE_RESET_RESP 8'h92
`define UA_TYPE_SET_REGSEL_RESP 8'h93

// -------------------------------------------------------
// Status codes
// -------------------------------------------------------
`define UA_STATUS_OK           8'h00
`define UA_STATUS_BAD_XOR      8'h01
`define UA_STATUS_BAD_TYPE     8'h02
`define UA_STATUS_BAD_LEN      8'h03
`define UA_STATUS_BAD_ADDR     8'h04
`define UA_STATUS_BUSY         8'h06
`define UA_STATUS_TIMEOUT      8'h07

// -------------------------------------------------------
// Source tags
// -------------------------------------------------------
`define UA_TAG_ICACHE          8'h00
`define UA_TAG_DCACHE          8'h01

// -------------------------------------------------------
// Payload lengths: memory path
// -------------------------------------------------------
`define UA_PLEN_READ_REQ       8'd5   // ADDR(4) + TAG(1)
`define UA_PLEN_WRITE_REQ      8'd10  // ADDR(4) + WSTRB(1) + TAG(1) + DATA(4)
`define UA_PLEN_READ_RESP      8'd6   // STATUS(1) + TAG(1) + DATA(4)
`define UA_PLEN_WRITE_RESP     8'd2   // STATUS(1) + TAG(1)

// -------------------------------------------------------
// Payload lengths: control path
// -------------------------------------------------------
`define UA_PLEN_SET_CLKDIV_REQ 8'd1   // DIV(1)
`define UA_PLEN_SET_HOLD_REQ   8'd1   // HOLD(1)
`define UA_PLEN_CORE_RESET_REQ 8'd0   // no payload
`define UA_PLEN_SET_REGSEL_REQ 8'd1   // REGSEL(1)

`define UA_PLEN_SET_CLKDIV_RESP 8'd2  // STATUS(1) + DIV(1)
`define UA_PLEN_SET_HOLD_RESP   8'd2  // STATUS(1) + HOLD(1)
`define UA_PLEN_CORE_RESET_RESP 8'd1  // STATUS(1)
`define UA_PLEN_SET_REGSEL_RESP 8'd2  // STATUS(1) + REGSEL(1)

// -------------------------------------------------------
// Full frame lengths: memory path
// -------------------------------------------------------
`define UA_FLEN_READ_REQ       8'd10
`define UA_FLEN_WRITE_REQ      8'd15
`define UA_FLEN_READ_RESP      8'd11
`define UA_FLEN_WRITE_RESP     8'd7

`endif