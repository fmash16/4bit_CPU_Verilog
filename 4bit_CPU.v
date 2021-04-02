// A 4 bit cpu for EEE415

// Instruction Set

`define ADD_A_B     4'b0000
`define SUB_A_B     4'b0001
`define XCHG_B_A    4'b0010
`define RCL_A       4'b0011
`define OUT_A       4'b0100
`define INC_A       4'b0101
`define MOV_B_ADDR  4'b0110
`define MOV_B_BYTE  4'b0111
`define JMP_ADDR    4'b1000
`define PUSH_B      4'b1001
`define POP_B       4'b1010
`define NOT_A       4'b1011
`define CALL_ADDR   4'b1100
`define RET         4'b1101
`define TEST_A_B    4'b1110
`define HLT         4'b1111

// Cycles

`define T1 4'b0000
`define T2 4'b0001
`define T3 4'b0010
`define T4 4'b0011
`define T5 4'b0100
`define T6 4'b0101
`define T7 4'b0110
`define T8 4'b0111


module programCounter (clk, Cp, Ep, Ci, addrIn, count);
    input clk, Cp, Ep;
    input Ci;
    input [3:0] addrIn;
    output wire [3:0] count;

    reg [3:0] counter = 4'b0;

    always @ (posedge clk) begin
        if (Cp)
            counter = counter+1;// increment the counter
        if (Ci)
            counter = addrIn;// jump to the input address
        // $display("Ci = %b, Ep = %b, count = %b", Ci, Ep, counter);
    end
    assign count = Ep ? counter : count;
endmodule


module memory (clk, Laddr, Eram, WE, ramIn, Edata, addrIn, opcodeOut, dataOut, Esp, spOp);
    input clk;
    // MAR
    input Laddr;
    input [3:0] addrIn;
    // RAM
    input Eram, WE;
    input [3:0] ramIn;
    output [3:0] opcodeOut;
    // MDR
    input Edata;
    output [3:0] dataOut;
    // SP
    input Esp;
    input [1:0] spOp;

    reg [7:0] mem [0:15];   //This is the storage
    reg [3:0] address_buf = 4'b0;  //address buffer
    wire [3:0] sp;          //Stack pointer

    stackPtr stackPtr (
		.spOp(spOp), 
		.Esp(Esp), 
		.out(sp)
    );

    initial begin
        // Load in the program/initial memory state into the
        // memory module
        $readmemh("program.hex", mem);
    end

    always @ (posedge clk) begin
        if (Laddr==1) begin
            if (WE==1)
                mem[address_buf] = addrIn+1;
            //MAR: load instruction of next target address from PC into buffer
            address_buf = addrIn;
        end
        if (WE==1) begin
            //RAM: write to [addrIn] address the data coming into ram
            mem[address_buf] = ramIn;
            // if (Laddr==1)
            //     mem[address_buf] = addrIn+1;
        end
        if (Esp==1) begin
            //SP: load instruction from stack
            address_buf = sp;
        end
        // $display("Laddr=%b, Eram=%b, addr_buf=%b, mem_addr_buf= %b, opcodeOut=%b, addrIn=%b, Esp=%b, spOp=%b, sp=%b", Laddr, Eram, address_buf, mem[address_buf], opcodeOut, addrIn, Esp,spOp, sp);
    end

    assign opcodeOut = Eram? mem[address_buf][7:4] : 'bZ;  //if ram enabled, out opcode to IR
    assign dataOut = Edata? mem[address_buf][3:0] : 'bZ;   //if data enabled,out data to bus
endmodule


module stackPtr (spOp, Esp, out);
    input [1:0] spOp;
    input Esp;
    output [3:0] out;

    reg [3:0] pointer;

    initial begin
        pointer = 4'b1111;
    end

    always @ (spOp) begin
        case (spOp)
            2'b00: pointer = 4'b1111;       // CLEAR
            2'b01: ;                        // HOLD
            2'b10: pointer = pointer - 1;   // PUSH 
            2'b11: pointer = pointer + 1;   // POP 
        endcase
    end
    assign out = pointer ;
endmodule

module iReg (clk, Li, opcodeIn, decoderOut);
    input clk, Li, Ei;
    input [3:0] opcodeIn;
    output [3:0] decoderOut;

    reg [3:0] instruction;   // Register to store the opcode instruction

    always @ (posedge clk) begin
        if (Li)
            instruction = opcodeIn; //If Li, load instruction to IR
        //$display("Li = %b, instruction = %b", Li, instruction);
    end

    assign decoderOut = instruction; //if instruction enable,out to control unit
endmodule


module decoder (cycle, insIn, ctrlOut);
    input [3:0] insIn, cycle;
    output reg [19:0] ctrlOut;

    always @ (cycle) begin
        //$display("instruction = %b", insIn);
        case (cycle)
            `T1:    ctrlOut = 20'b01010000000000000010;
            `T2:    ctrlOut = 20'b10000101000000000010;    
            default:
                case (insIn)
                    `ADD_A_B:
                        ctrlOut = (cycle == `T3) ? 20'b00000000000010000010 :   // aluOp = 001
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ealu,La
                                  20'b10;
                    `SUB_A_B:
                        ctrlOut = (cycle == `T3) ? 20'b00000000000100000010 :   // aluOp = 010
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ealu,La
                                  20'b10;
                    `XCHG_B_A:
                        ctrlOut = (cycle == `T3) ? 20'b00000000000010000010 :   // aluOp = 001
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ea,La
                                  (cycle == `T5) ? 20'b00000000000100000010 :   // HLT
                                  (cycle == `T6) ? 20'b00000000000001100010 :   // HLT
                                  (cycle == `T7) ? 20'b00000000000100000010 :   // HLT
                                  (cycle == `T8) ? 20'b00000000100001000010 :   // HLT
                                  20'b10;
                    `RCL_A:
                        ctrlOut = (cycle == `T3) ? 20'b00000000001010000010 :   // aluOp = 101
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ea,La
                                  20'b10;
                    `OUT_A:
                        ctrlOut = (cycle == `T3) ? 20'b00000000010000000011 :   // Ea, Lo
                                  20'b10;
                    `INC_A:
                        ctrlOut = (cycle == `T3) ? 20'b00000000000110000010 :   // aluOp = 011
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ealu,La
                                  20'b10;
                    `MOV_B_ADDR:
                        ctrlOut = (cycle == `T3) ? 20'b00010000000000000010 :   // Ei, Lm
                                  (cycle == `T4) ? 20'b00000010100000000010 :   // Ed, Lb
                                  20'b10;
                    `MOV_B_BYTE:
                        ctrlOut = (cycle == `T3) ? 20'b00000010000000100010 :   // Ei, Lb
                        20'b10;
                    `JMP_ADDR:
                        ctrlOut = (cycle == `T3) ? 20'b00100000000000000010 :   // Ei, Ci
                        20'b10;
                    `PUSH_B:
                        ctrlOut = (cycle == `T3) ? 20'b00000000000000001010 :   // Esp, Lm
                                  (cycle == `T4) ? 20'b00001000000000010100 :   // WE, Eb, decSp
                        20'b10;
                    `POP_B:
                        ctrlOut = (cycle == `T3) ? 20'b00010000000000001110 :   // Esp, Lm | increment?
                                  (cycle == `T4) ? 20'b00000010000000100010 :   // Ed, Lb
                        20'b10;
                    `NOT_A:
                        ctrlOut = (cycle == `T3) ? 20'b00000000001000000010 :   // aluOp = 100
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ea,La
                        20'b10;
                    `CALL_ADDR:
                        ctrlOut = (cycle == `T3) ? 20'b00100010000000000010 :   // Ed, Ci
                                  (cycle == `T4) ? 20'b00000000000000001010 :   // Esp, Lm
                                  (cycle == `T5) ? 20'b00011000000000000100 :   // Ed, Ci
                                  
                        20'b10;
                    `RET:
                        ctrlOut = (cycle == `T3) ? 20'b00000000000000001110 :   // incSp
                                  (cycle == `T4) ? 20'b00100010000000000010 :   // Esp, Lm
                        20'b10;
                    `TEST_A_B:
                        ctrlOut = (cycle == `T3) ? 20'b00000000001100000010 :   // aluOp = 110
                                  (cycle == `T4) ? 20'b00000000100001000010 :   // Ea,La
                        20'b10;
                    `HLT:
                        ctrlOut = 20'b0 ;   
                    default:
                        ctrlOut = (cycle == `T2) ? 20'b10 :   // aluOp = 001
                        20'b10;
                endcase
            endcase
    end
endmodule


module controller (clk, insIn, ctrlOut);
    // 1  2  3  4  5  6  7  8  9  10  11 12,13  14  15 16 17   18   19
    // Cp Ep Ci Lm WE CE Ed Li Ei La  Ea   Op  Ealu Lb Eb Esp decSp Lo
    input clk;
    input [3:0] insIn;
    output wire [19:0] ctrlOut;

    wire [19:0] buffer;
    reg [3:0] cycle = 4'b0;

    decoder decode(cycle, insIn, buffer);

    always @ (posedge clk) begin
        if (buffer != 20'b0)
            cycle = cycle + 1;
        if (buffer == 19'b10)
            cycle = 0;
        // $display("ctrlOut=%b, insIn = %b, cycle=%d", buffer, insIn, cycle);
    end

    assign ctrlOut = buffer;
endmodule


module register (clk, La, Ea, data, toAlu);
    input clk;
    input La, Ea;
    inout [3:0] data;
    output [3:0] toAlu;

    reg [3:0] aReg;

    initial
        aReg = 0;

    always @ (posedge clk) begin
        if (La)
            aReg <= data;
    end

    assign toAlu = aReg;
    assign data = Ea? aReg : 'bZ;
endmodule

module flagReg (flagOut, zFlag, sFlag, cFlag);
    input [2:0] flagOut;
    output zFlag, sFlag, cFlag;

    assign cFlag = flagOut[0];
    assign zFlag = flagOut[1];
    assign sFlag = flagOut[2];

    // always @ (*)
    //     $display("carry = %b, zero = %b", cFlag, zFlag);
endmodule


module ALU (clk, Ealu, carryIn, aluOp, Ain, Bin, dataOut, flagOut);
    input clk, Ealu, carryIn;
    input [2:0] aluOp;
    input [3:0] Ain, Bin;
    output wire [3:0] dataOut;
    output reg [2:0] flagOut;

    reg [4:0] buffer = 4'b0;

    initial
        flagOut=3'b0;

    always @ (posedge clk) begin
        case (aluOp)
            3'b000:
                ;
            3'b001:
                buffer = Ain+Bin;       // ADD
            3'b010:
                buffer = Ain+(~Bin)+1;  // SUB
            3'b011:
                buffer = Ain+1;         // INC
            3'b100:
                buffer = ~Ain;          // NOT
            3'b101: begin
                buffer = Ain;           // RCL
                buffer = {Ain, carryIn};
            end
            3'b110:
                buffer = Ain ^ Bin;      // TEST
            default:
                buffer = 4'bx;
        endcase
        // $display("aluOp=%b, buffer = %b, carryIn=%b, flagOut=%b", aluOp,  buffer, carryIn, flagOut);
        flagOut[0] = buffer[4] + carryIn;                              // cFlag
        flagOut[1] = ~(buffer[0] | buffer[1] | buffer[2] | buffer[3]); // zFlag
    end

    assign dataOut = Ealu? buffer[3:0] : 'bZ;
endmodule


module out (clk, Lo, dataIn, dataOut);
    input clk;
    input Lo;
    input [3:0] dataIn;
    output wire [3:0] dataOut;

    assign dataOut = Lo ? dataIn : 'bZ;
endmodule


module cpu (clk, out);
    input clk;
    output [3:0] out;

    wire [3:0] bus, addrbus ;
    wire Cp, Ep, Ci, Lm, WE, CE, Ed, Li, Ei, La, Ea, Ealu, Lb, Eb, Esp, Lo;
    wire zFlag, cFlag, sFlag;
    wire [1:0] spOp;
    wire [2:0] aluOp;
    wire [3:0] iReg2controller, a2alu, b2alu;
    wire [2:0] alu2flag;

    programCounter programCounter(
        .clk(clk), 
        .Cp(Cp), 
        .Ep(Ep), 
        .Ci(Ci),
        .addrIn(bus),
        .count(addrbus)
    );

    memory memory (
        .clk(clk), 
		.Laddr(Lm), 
		.Eram(CE), 
		.WE(WE), 
		.ramIn(bus), 
		.Edata(Ed), 
		.addrIn(addrbus), 
		.opcodeOut(bus), 
		.dataOut(bus),
        .Esp(Esp),
        .spOp(spOp)
    );

    iReg iReg (
        .clk(clk), 
		.Li(Li), 
		.opcodeIn(bus), 
		.decoderOut(iReg2controller)
    ); 

    controller controller (
        .clk(clk), 
		.insIn(iReg2controller),
		.ctrlOut({Cp, Ep, Ci, Lm, WE, CE, Ed, Li, La, Ea, aluOp, Ealu, Lb, Eb, Esp, spOp, Lo})
    );


    register accumulator (
        .clk(clk), 
		.La(La), 
		.Ea(Ea), 
		.data(bus), 
		.toAlu(a2alu)
    );

    register BReg (
        .clk(clk), 
		.La(Lb), 
        .Ea(Eb),
		.data(bus), 
		.toAlu(b2alu)
    );

    flagReg flagReg (
        .flagOut(alu2flag), 
		.zFlag(zFlag), 
		.sFlag(sFlag), 
		.cFlag(cFlag)
    );

    ALU ALU (
        .clk(clk), 
		.Ealu(Ealu), 
        .carryIn(cFlag),
		.aluOp(aluOp), 
		.Ain(a2alu), 
		.Bin(b2alu), 
		.dataOut(bus), 
		.flagOut(alu2flag)
    );

    out cpu_out (
        .clk(clk), 
		.Lo(Lo), 
		.dataIn(bus), 
		.dataOut(out)
    );
endmodule
