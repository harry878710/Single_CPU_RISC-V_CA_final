// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    reg    [31:0] rd_data     ;              //
    //---------------------------------------//

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Opcode variables
    parameter OP_AUIPC = 7'b0010111;
    parameter OP_JAL   = 7'b1101111;
    parameter OP_JALR  = 7'b1100111;
    parameter OP_B     = 7'b1100011;
    parameter OP_LW    = 7'b0000011;
    parameter OP_SW    = 7'b0100011;
    parameter OP_I     = 7'b0010011;
    parameter OP_R     = 7'b0110011;

    // State variables
    parameter FETCH = 2'b00;
    parameter AWAIT = 2'b01;
    parameter WRITE = 2'b10;

    // Wires for alu
    wire               alu_valid         ;
    wire        [ 2:0] alu_mode          ;
    wire signed [31:0] alu_in_A, alu_in_B;
    wire signed [32:0] alu_out           ;

    // Wires for mdu
    wire               mdu_valid         ;
    wire               mdu_mode          ;
    wire               mdu_ready         ;
    wire signed [31:0] mdu_in_A, mdu_in_B;
    wire signed [63:0] mdu_out           ;

    // Wires/Regs for instuction
    reg  [31:0] imm   ;
    wire [ 6:0] opcode;
    wire [ 2:0] funct3;
    wire [ 6:0] funct7;

    // Wires for control signals
    wire is_slti      ;
    wire is_sub, is_md;
    wire is_PC, is_rs2; // PC or rs1_data as alu_in_A | rs2_data or imm as alu_in_B
    wire j_en, b_en   ; // jump enable | branch enable

    // Reg for state
    reg  [ 1:0] state, next_state;

    alu alu0(
        .clk(clk),
        .rst_n(rst_n),
        .valid(alu_valid),
        .mode(alu_mode),
        .in_A(alu_in_A),
        .in_B(alu_in_B),
        .out(alu_out));

    mulDiv mdu0(
        .clk(clk),
        .rst_n(rst_n),
        .valid(mdu_valid),
        .ready(mdu_ready),
        .mode(mdu_mode),
        .in_A(mdu_in_A),
        .in_B(mdu_in_B),
        .out(mdu_out));

    // Instruction decode
    assign mem_addr_I = PC;
    assign opcode = mem_rdata_I[ 6: 0];
    assign rd     = mem_rdata_I[11: 7];
    assign funct3 = mem_rdata_I[14:12];
    assign rs1    = mem_rdata_I[19:15];
    assign rs2    = mem_rdata_I[24:20];
    assign funct7 = mem_rdata_I[31:25];

    // Immediate decode
    always @(*) begin
        case (opcode)
            OP_AUIPC: imm = {{12{mem_rdata_I[31]}}, mem_rdata_I[31:12]};
            OP_JAL  : imm = {{11{mem_rdata_I[31]}}, mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0};
            OP_B    : imm = {{19{mem_rdata_I[31]}}, mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8], 1'b0};
            OP_SW   : imm = {{20{mem_rdata_I[31]}}, mem_rdata_I[31:25], mem_rdata_I[11:7]};
            default : imm = {{20{mem_rdata_I[31]}}, mem_rdata_I[31:20]};
        endcase
    end

    // Control signals
    assign is_slti = opcode == OP_I & funct3[1];
    assign is_sub  = opcode == OP_R & funct7[5];
    assign is_md   = opcode == OP_R & funct7[0];
    assign is_PC   = opcode[2] & (opcode[3] | opcode[4]);
    assign is_rs2  = opcode == OP_B | opcode == OP_R;
    assign j_en    = opcode[2] & opcode[6];
    assign b_en    = opcode == OP_B & ((!funct3[2] & (funct3[0] ^ alu_out[31:0] == 32'd0)) | (funct3[2] & (funct3[0] ^ alu_out[31])));

    // State trasition
    always @(*) begin
        case (state)
            FETCH  : next_state = (mem_addr_I == 32'b0)? FETCH : (is_md)? AWAIT : WRITE;
            AWAIT  : next_state = (mdu_ready)? WRITE : AWAIT;
            WRITE  : next_state = FETCH;
            default: next_state = FETCH;
        endcase
    end

    // ALU control
    assign alu_valid   = state == FETCH & !is_md;
    assign alu_mode[2] = opcode == OP_I & funct3[0]; // shift or not
    assign alu_mode[1] = funct3[2]; // shift left or right
    assign alu_mode[0] = opcode == OP_B | is_slti | is_sub; // sub or not
    assign alu_in_A    = (is_PC)? PC : rs1_data;
    assign alu_in_B    = (is_rs2)? rs2_data : imm;

    // MDU control
    assign mdu_valid = state == FETCH & is_md;
    assign mdu_mode  = funct3[2];
    assign mdu_in_A  = rs1_data;
    assign mdu_in_B  = rs2_data;

    // PC control
    assign PC_nxt = (j_en)? alu_out[31:0] : (b_en)? PC + imm : PC + 32'd4;

    // Register write control
    assign regWrite = (state == WRITE & opcode != OP_SW & opcode != OP_B);
    always @(*) begin
        if (j_en) rd_data = PC + 32'd4;
        else if (opcode == OP_LW) rd_data = mem_rdata_D;
        else if (is_slti) rd_data = {31'd0, alu_out[31]};
        else rd_data = (is_md)? mdu_out[31:0] : alu_out[31:0];
    end

    // Memory write control
    assign mem_wen_D   = (state == WRITE && opcode == OP_SW)? 1'b1 : 1'b0;
    assign mem_addr_D  = alu_out[31:0];
    assign mem_wdata_D = rs2_data;

    // Sequential
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= FETCH;
        end
        else begin
            PC <= (next_state == FETCH)? PC_nxt : PC;
            state <= next_state;
        end
    end
endmodule


module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule


module alu(clk, rst_n, valid, mode, in_A, in_B, out);

    // Ports
    input                clk, rst_n;
    input                valid     ;
    input         [ 2:0] mode      ; // mode: 000:add | 001:sub | 100:shift left | 110:shift right
    input  signed [31:0] in_A, in_B;
    output signed [32:0] out       ;

    // Output logic
    assign out = (mode[2])? (mode[1])? in_A >> in_B : in_A << in_B : (mode[0])? in_A - in_B : in_A + in_B;

endmodule


module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);

    // Todo: your HW3
    // Definition of ports
    input         clk, rst_n;
    input         valid, mode; // mode: 0: mulu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

// Definition of states
    parameter IDLE = 2'b00;
    parameter MUL  = 2'b01;
    parameter DIV  = 2'b10;
    parameter OUT  = 2'b11;

// Wire and reg
    reg  [ 1:0] state, n_state;
    reg  [ 4:0] cnt;
    reg  [63:0] shreg;
    reg  [31:0] alu_in;

// my reg & wire
    wire [32:0] front_shreg;

// assignments
    assign ready = (cnt == 1'b0 && state==OUT) ? 1'b1 : 1'b0;
    assign out = shreg;
    assign front_shreg = (state==MUL) ? shreg[63:32] + alu_in : shreg[62:31] - alu_in;

// Combinational always block
// FSM
    always @(*) begin
        case(state)
            IDLE: n_state = (valid==1'b1) ? ((mode==1'b0) ? MUL : DIV) : IDLE;
            MUL : n_state = (cnt < 5'd31) ? MUL : OUT;
            DIV : n_state = (cnt < 5'd31) ? DIV : OUT;
            OUT : n_state = IDLE;
            default: n_state = IDLE;
        endcase
    end

// Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cnt <= 0;
            alu_in <= 0;
            shreg <= 0;
        end
        else begin
            state <= n_state;
            case(state)
                IDLE:begin
                    alu_in <= in_B;
                    shreg <= in_A;
                end
                MUL:begin
                    shreg <= (shreg[0]==1'b0) ? shreg>>1 : {front_shreg , shreg[31:0]} >> 1;
                    cnt <= (cnt < 5'd31) ? cnt + 1'b1 : 1'b0;
                end
                DIV:begin
                    shreg <= (shreg[62:31] < alu_in) ? shreg<<1 : {front_shreg[31:0] , shreg[30:0] , 1'b1};
                    cnt <= (cnt < 5'd31) ? cnt + 1'b1 : 1'b0;
                end
            endcase
        end
    end

endmodule
