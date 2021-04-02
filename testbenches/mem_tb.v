`include "../mashCPU.v"

`timescale 1s / 1s

module pc_tb;
    reg clk, Laddr, Eram, WE, Edata, Esp;
    reg [1:0] spOp;
    reg [3:0] opcodeOut, dataOut, ramIn, addrIn;
    
    initial begin
        clk=0;
        forever #1 clk = ~clk;
    end

    memory memory (clk, Laddr, Eram, WE, ramIn, Edata, addrIn, opcodeOut, dataOut, Esp, spOp);
    
    initial begin
        Laddr=0;Eram=0;ramIn=0;Esp=0;WE=0;
        #1 addrIn = 4'b0001; Laddr = 1;
        #1 Laddr = 0;addrIn = 4'bZ;Eram=1;
        // #1 Eram=1;
        #1 Edata = 0;Eram=0;
        #1 WE=0;
        #2 ramIn=4'b0111;
        #1 WE=0; Laddr=1;
        #1 Laddr=0;Edata=1;Eram=1;
        #1 Laddr=0;Edata=0;Eram=0;

        #1 Esp=1;spOp=2'b10;
        #1 Esp=0;spOp=2'b01;WE=1;
        #1 ramIn = 4'b1111;Edata=1;
        #1 WE=0;
        #1 
        #1 Edata=0;Eram=0;

        // #1 Esp=1;spOp=2'b10;
        // #1 Esp=0;spOp=2'b01;WE=1;
        // #1 ramIn = 4'b1110;
        // #1 WE=0;
        // #1 addrIn=4'b1111; Laddr=1;
        // #1 addrIn=4'b0000; Laddr=0;
        // #1 Edata=1;Eram=1;

        #20 $finish;
    end

    initial begin
        $monitor("time = %2d, Laddr=%b , Eram=%b , WE=%b , ramIn=%b , Edata=%b , addrIn=%b , opcodeOut=%b , dataOut=%b , Esp=%b , spOp=%b", $time, Laddr, Eram, WE, ramIn, Edata, addrIn, opcodeOut, dataOut, Esp, spOp);

    end

endmodule
