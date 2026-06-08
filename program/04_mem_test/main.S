            .text

  start:    li t0, 10     # t0 = 10
            sw t0, 0x104  # mem[0x104] = 10
            nop
            nop
            lw a0, 0x104  # a0 = mem[0x104]
            nop
            nop
            li a0, 0      # a0 = 0

            li t0, -2     # t0 = -2
            sb t0, 0x100  # mem[0x100] = 0xFE
            lb a0, 0x100  # a0 = signext(mem[0x100])
            nop
            nop
            lbu a0, 0x100 # a0 = zeroext(mem[0x100])
loop:       j loop
