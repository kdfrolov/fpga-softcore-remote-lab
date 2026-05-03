`ifndef UART_AGENT_PROTO_VH
`define UART_AGENT_PROTO_VH

// -------------------------------------------------------
// Frame
// -------------------------------------------------------
`define UA_SOF             8'hA5

// -------------------------------------------------------
// Packet types: FPGA -> PC
// -------------------------------------------------------
`define UA_TYPE_IREAD_REQ  8'h01
`define UA_TYPE_DREAD_REQ  8'h02
`define UA_TYPE_WRITE      8'h03

// -------------------------------------------------------
// Packet types: PC -> FPGA
// -------------------------------------------------------
`define UA_TYPE_IREAD_RESP 8'h81
`define UA_TYPE_DREAD_RESP 8'h82
`define UA_TYPE_WRITE_RESP 8'h83

// -------------------------------------------------------
// Status codes
// -------------------------------------------------------
`define UA_STATUS_OK       8'h00
`define UA_STATUS_BAD_XOR  8'h01
`define UA_STATUS_BAD_TYPE 8'h02
`define UA_STATUS_BAD_ADDR 8'h04
`define UA_STATUS_BUSY     8'h06
`define UA_STATUS_TIMEOUT  8'h07

// -------------------------------------------------------
// Source tags
// -------------------------------------------------------
`define UA_TAG_ICACHE      8'h00
`define UA_TAG_DCACHE      8'h01

// -------------------------------------------------------
// Payload lengths
// -------------------------------------------------------
`define UA_PLEN_READ_REQ   8'd5   // ADDR(4) + TAG(1)
`define UA_PLEN_WRITE_REQ  8'd10  // ADDR(4) + WSTRB(1) + TAG(1) + DATA(4)
`define UA_PLEN_READ_RESP  8'd6   // STATUS(1) + TAG(1) + DATA(4)
`define UA_PLEN_WRITE_RESP 8'd2   // STATUS(1) + TAG(1)

// -------------------------------------------------------
// Full frame lengths
// -------------------------------------------------------
`define UA_FLEN_READ_REQ   8'd10  // 4 + 5 + 1
`define UA_FLEN_WRITE_REQ  8'd15  // 4 + 10 + 1
`define UA_FLEN_READ_RESP  8'd11  // 4 + 6 + 1
`define UA_FLEN_WRITE_RESP 8'd7   // 4 + 2 + 1

`endif