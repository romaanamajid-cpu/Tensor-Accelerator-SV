module blinker_tb;
    logic clk = 0;
    logic rst_n = 0;
    logic led;

    blinker dut (.clk(clk), .rst_n(rst_n), .led(led));

    always #5 clk = ~clk;

    initial begin
        $dumpfile("sim/m0_smoke_test/blinker.vcd");
        $dumpvars(0, blinker_tb);

        rst_n = 0;
        #12 rst_n = 1;
        #100 $finish;
    end
endmodule
