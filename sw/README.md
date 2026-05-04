# schoolRISCV PC Host — UART Memory Agent

Python-side memory agent for the schoolRISCV FPGA project.  
Connects to the FPGA via a physical UART, answers IREAD / DREAD / WRITE
requests from the hardware `uart_mem_agent_2clk` module.

## Protocol summary

| Direction     | Frame structure                                      |
|---------------|------------------------------------------------------|
| FPGA → PC     | `A5 TYPE SEQ LEN [PAYLOAD x LEN] XOR`               |
| PC → FPGA     | `A5 TYPE SEQ LEN [PAYLOAD x LEN] XOR`               |

XOR = TYPE ^ SEQ ^ LEN ^ payload bytes

| Request type  | Code | Payload (LEN bytes)                              |
|---------------|------|--------------------------------------------------|
| IREAD_REQ     | 0x01 | ADDR[4LE] TAG[1]   (5 bytes)                     |
| DREAD_REQ     | 0x02 | ADDR[4LE] TAG[1]   (5 bytes)                     |
| WRITE_REQ     | 0x03 | ADDR[4LE] WSTRB[1] TAG[1] DATA[4LE]  (10 bytes) |

| Response type | Code | Payload (LEN bytes)                              |
|---------------|------|--------------------------------------------------|
| IREAD_RESP    | 0x81 | STATUS[1] TAG[1] DATA[4LE]   (6 bytes)          |
| DREAD_RESP    | 0x82 | STATUS[1] TAG[1] DATA[4LE]   (6 bytes)          |
| WRITE_RESP    | 0x83 | STATUS[1] TAG[1]              (2 bytes)          |

Status codes: `OK=0x00 BAD_XOR=0x01 BAD_TYPE=0x02 BAD_ADDR=0x04`

## Quick start

```bash
pip install pyserial
pip install pytest        # for tests only

# Load program.hex and serve on COM3 at 115200 baud
python main.py --port COM3 --baud 115200 --hex data/program.hex

# Or on Linux
python main.py --port /dev/ttyUSB0 --baud 115200 --hex data/program.hex

# Run tests (no hardware needed)
pytest
```

## Project structure

```text
sw/
├── main.py              Entry point (CLI)
├── pyproject.toml
├── app/
│   ├── protocol.py      Frame codec + constants
│   ├── memory_model.py  Byte-addressable LE memory + WSTRB writes
│   ├── agent.py         Request handler loop
│   ├── serial_port.py   pyserial ByteStream adapter
│   └── mock_stream.py   In-memory stream for tests
├── tests/
│   ├── test_protocol.py
│   ├── test_memory_model.py
│   └── test_agent.py
└── data/
    └── program.hex      (place your hex image here)
```

## Future extensions (not yet implemented)

- CPU clock stop / start command
- Clock divider control
- Register file / PC read-back monitoring