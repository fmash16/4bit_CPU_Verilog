`include "../mashCPU.v"

module alu_tb;
    reg Ealu;
    reg [3:0] Ain,Bin;
    reg [2:0] aluOp;
    wire [3:0] dataOut;
    wire [2:0] flagOut;

    reg clk = 0;

    always #2 clk = ~clk;

    ALU alu(clk, Ealu, aluOp, Ain, Bin, dataOut, flagOut);

    initial begin
        Ealu = 0;
        aluOp = 3'b000;
        #2 Ain = 4'b1001;
        #2 Bin = 4'b0011;
        #2 Ealu = 1;
        #20 $finish;
    end

    initial begin
        $monitor("time = %2d, Ealu=%d, aluOp=%b, A = %b, B = %b, SUM = %b, flags = %b", $time, Ealu, aluOp, Ain, Bin, dataOut, flagOut);
    end

endmodule
