.data
    n: .word 7
    
.text
.globl __start

FUNCTION:
    addi x6, x0, 2
    blt x10, x6, final # n < 2 go to final
    
    slli x7, x10, 2    # x7 = 4*n
    addi sp, sp, -8
    sw x1, 4(sp)
    sw x7, 0(sp)
    srli x10, x10, 1    # n = floor(n/2)
    jal x1, FUNCTION
    
    slli x7, x10, 3    # x7 = 8*T(floor(n/2))
    lw x10, 0(sp)     # x10 = 4*n
    lw x1, 4(sp)
    addi sp, sp, 8
    add x10, x10, x7  # x10 = x10 + x7
    jalr x0, 0(x1)    # return 8*T(floor(n/2)) + 4*n

final:
    addi x10, x0, 7   # x10 = 7
    jalr x1, 0(x1)    # return 7

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall