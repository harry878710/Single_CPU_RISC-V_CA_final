.data
    n: .word 7
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1

    addi x9, x0, 2
    blt x10, x9, FINAL # if(n>=2)jump to RECUR

    slli x19, x10, 2 # 4n
    addi sp, sp, -8
    sw x19, 4(sp) # tmp store 4n
    sw x1, 0(sp) # tmp store address
    srli x10, x10, 1 # n/2
    jal x1, FUNCTION # Compute T(n/2)
    
    slli x19, x5, 3 # 8*T(n/2)
    lw x1, 0(sp)
    lw x5, 4(sp) # +4n
    addi sp, sp, 8
    add x5, x5, x19 # 8*T(n/2)+4n
    jalr x0, 0(x1)
FINAL:
    addi t0, x0, 7    # x5 = 7
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