`include "4bit_CPU.v"

`timescale 1s / 1s

module pc_tb;
    wire [3:0] out;
    reg clk;

    initial begin
        clk=0;
        forever #1 clk = ~clk;
    end

    cpu cpu (clk, out);

    initial begin
        #120 $finish;
    end

    initial begin
        $dumpfile("machine.vcd");
        $dumpvars(0, cpu);
        $monitor("time = %2d, out = %d", $time, out);
    end

endmodule
