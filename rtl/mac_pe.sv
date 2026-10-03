// mac_pe.sv
// Processing element (PE) for the output-stationary MAC array.
//
// Wraps one mac_unit and adds the "systolic plumbing": the A operand is
// registered and forwarded to the right, the B operand is registered and
// forwarded downward. The accumulator stays put inside the PE (output
// stationary), so after a full matrix product acc_out holds C[i][j].
//
// Operand valids travel with their data. The MAC fires only when both the
// A and B operand arriving this cycle are valid.
module mac_pe #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          clear_acc,

    // A stream (flows left -> right)
    input  logic signed [DATA_WIDTH-1:0]  a_in,
    input  logic                          a_valid_in,
    output logic signed [DATA_WIDTH-1:0]  a_out,
    output logic                          a_valid_out,

    // B stream (flows top -> bottom)
    input  logic signed [DATA_WIDTH-1:0]  b_in,
    input  logic                          b_valid_in,
    output logic signed [DATA_WIDTH-1:0]  b_out,
    output logic                          b_valid_out,

    // Stationary accumulator result
    output logic signed [ACC_WIDTH-1:0]   acc_out,
    output logic                          acc_valid_out
);

    // One register stage of forwarding per PE -> neighbours see our operands
    // exactly one cycle after we do.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_out       <= '0;
            a_valid_out <= 1'b0;
            b_out       <= '0;
            b_valid_out <= 1'b0;
        end else begin
            a_out       <= a_in;
            a_valid_out <= a_valid_in;
            b_out       <= b_in;
            b_valid_out <= b_valid_in;
        end
    end

    // The verified M1/M2 MAC unit, reused unchanged.
    mac_unit #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_mac (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear_acc(clear_acc),
        .valid_in (a_valid_in & b_valid_in),
        .a        (a_in),
        .b        (b_in),
        .acc_out  (acc_out),
        .valid_out(acc_valid_out)
    );

endmodule