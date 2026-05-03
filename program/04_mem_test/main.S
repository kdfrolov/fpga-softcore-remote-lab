  .text
  li t0, 10
  sw t0, 0x104(zero)
  nop
  nop
  lw a0, 0x104(zero)
  nop
  nop
  li a0, 0

  li t0, -2
  sb t0, 0x100(zero)
  lb a0, 0x100(zero)
  nop
  nop
  lbu a0, 0x100(zero)
loop:
  j loop