// mac_unit.sv
// Multiply-accumulate unit: acc_out += a * b, one product per valid cycle.
//
// Parameters
//   DATA_WIDTH : width of the signed operands a and b (legal: >= 2)
//   ACC_WIDTH  : width of the signed accumulator (legal: >= 2*DATA_WIDTH, so
//                one full product always fits). The accumulator wraps around
//                if a long sum outgrows ACC_WIDTH; it never saturates.
//
// Illegal parameter values stop the simulation at time 0 with a $fatal
// message instead of silently building a broken unit.
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

    // ------------------------------------------------------------------
    // Parameter legality checks (run once, at time 0)
    // ------------------------------------------------------------------
    initial begin
        if (DATA_WIDTH < 2)
            $fatal(1, "mac_unit: DATA_WIDTH (%0d) must be >= 2", DATA_WIDTH);
        if (ACC_WIDTH < 2*DATA_WIDTH)
            $fatal(1, "mac_unit: ACC_WIDTH (%0d) must be >= 2*DATA_WIDTH (%0d)",
                   ACC_WIDTH, 2*DATA_WIDTH);
    end

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