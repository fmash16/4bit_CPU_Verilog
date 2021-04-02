`include "../4bit_CPU.v"

`timescale 1s / 1s

module pc_tb;
    reg Cp, Ep;
    reg [3:0] Ci;
    wire [3:0] count;

    reg clk;

    initial begin
        clk=0;
        forever #1 clk = ~clk;
    end

    programCounter pc(clk, Cp, Ep, Ci, count);

    initial begin
        Ep=0;
        Cp=0;
        Ci=4'b0;
        #2 Cp=1;
        #2 Ep=1;
        #10 Ep=0;
        #2 Ci=4'b0001;
        #2 Ep=1;
        #6 Ci=4'bxxxx;
        #2 Cp=1;
        #20 $finish;
    end


    initial begin
        $monitor("time = %2d, Cp=%b, Ep=%b, Ci=%b, count=%b", $time, Cp, Ep, Ci, count);
    end

endmodule
