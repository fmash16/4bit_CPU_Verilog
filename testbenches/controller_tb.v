`include "../mashCPU.v"

`timescale 1s / 1s

module pc_tb;
    reg [3:0] insIn; 
    reg [3:0] ins; 
    reg [3:0] cyc; 

    wire [3:0] cycle;
    wire [19:0] ctrlOut;
    wire [3:0] out, bus;
    reg clk;

    initial begin
        clk=0;
        forever #1 clk = ~clk;
    end

    // controller controller (clk, insIn, ctrlOut, cycle);
    // decoder decode (cyc, ins, out);
    cpu cpu (clk, out);

    initial begin
        // cyc=0;
        // insIn = 4'b0000;
        #100 $finish;
    end

    initial begin
        // $monitor("time = %2d, cycle = %b, insIn = %b, ctrlOut = %b", $time,cyc, ins, out);
        // $monitor("time = %2d, ctrlOut = %b, cycle = %b, insIn = %b", $time, ctrlOut, cycle, insIn);
        $monitor("time = %2d, out = %d, bus = %b", $time, out, bus);

    end

    // always @ (posedge clk)
        // $display("=====================================================================");

endmodule
