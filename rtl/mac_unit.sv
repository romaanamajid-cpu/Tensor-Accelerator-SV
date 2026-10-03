module mac_unit #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          clear_acc,
    input  logic                          valid_in,
    input  logic signed [DATA_WIDTH-1:0]  a,
    input  logic signed [DATA_WIDTH-1:0]  b,
    output logic signed [ACC_WIDTH-1:0]   acc_out,
    output logic                          valid_out
);

    logic signed [2*DATA_WIDTH-1:0] product;

    // Combinational multiply
    assign product = a * b;

    // Registered accumulate
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_out   <= '0;
            valid_out <= 1'b0;
        end else if (clear_acc) begin
            acc_out   <= '0;
            valid_out <= 1'b0;
        end else if (valid_in) begin
            acc_out   <= acc_out + $signed({{(ACC_WIDTH-2*DATA_WIDTH){product[2*DATA_WIDTH-1]}}, product});
            valid_out <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end

endmodule