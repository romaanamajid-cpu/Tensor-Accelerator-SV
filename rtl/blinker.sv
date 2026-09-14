module blinker (
    input  logic clk,
    input  logic rst_n,
    output logic led
);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) led <= 1'b0;
        else        led <= ~led;
    end
endmodule
