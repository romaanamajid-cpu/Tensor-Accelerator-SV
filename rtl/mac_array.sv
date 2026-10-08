// mac_array.sv
// ROWS x COLS output-stationary systolic MAC array.
//
//   A operands enter on the left edge (one lane per row)   and flow right.
//   B operands enter on the top edge  (one lane per column) and flow down.
//   PE(i,j) accumulates C[i][j] = sum_k A[i][k] * B[k][j] in place.
//
// This module does NOT skew its inputs. The driver must delay row i by i
// cycles and column j by j cycles so that matching A and B elements reach
// each PE in the same cycle (done by the testbench now, the M4 engine later).
module mac_array #(
    parameter ROWS       = 4,
    parameter COLS       = 4,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                                         clk,
    input  logic                                         rst_n,
    input  logic                                         clear_acc,   // broadcast to every PE

    // Left-edge A lanes, one per row
    input  logic signed [ROWS-1:0][DATA_WIDTH-1:0]       a_in,
    input  logic        [ROWS-1:0]                       a_valid_in,

    // Top-edge B lanes, one per column
    input  logic signed [COLS-1:0][DATA_WIDTH-1:0]       b_in,
    input  logic        [COLS-1:0]                       b_valid_in,

    // Stationary results, c_out[i][j] = C[i][j]
    output logic signed [ROWS-1:0][COLS-1:0][ACC_WIDTH-1:0] c_out,
    output logic        [ROWS-1:0][COLS-1:0]                c_valid_out
);

    // ------------------------------------------------------------------
    // Parameter legality checks (run once, at time 0). The width rules
    // (DATA_WIDTH, ACC_WIDTH) are checked inside mac_unit.
    // ------------------------------------------------------------------
    initial begin
        if (ROWS < 1)
            $fatal(1, "mac_array: ROWS (%0d) must be >= 1", ROWS);
        if (COLS < 1)
            $fatal(1, "mac_array: COLS (%0d) must be >= 1", COLS);
    end

    // Operand wires between PEs. Index [i][j] is the value ENTERING PE(i,j);
    // the extra column / row (index COLS / ROWS) catches what leaves the
    // right and bottom edges, which nothing uses.
    logic signed [DATA_WIDTH-1:0] a_wire   [ROWS][COLS+1];
    logic                         a_v_wire [ROWS][COLS+1];
    logic signed [DATA_WIDTH-1:0] b_wire   [ROWS+1][COLS];
    logic                         b_v_wire [ROWS+1][COLS];

    // Connect the array edges to the module ports
    generate
        for (genvar i = 0; i < ROWS; i++) begin : g_left_edge
            assign a_wire  [i][0] = a_in[i];
            assign a_v_wire[i][0] = a_valid_in[i];
        end
        for (genvar j = 0; j < COLS; j++) begin : g_top_edge
            assign b_wire  [0][j] = b_in[j];
            assign b_v_wire[0][j] = b_valid_in[j];
        end
    endgenerate

    // The grid of PEs
    generate
        for (genvar i = 0; i < ROWS; i++) begin : g_row
            for (genvar j = 0; j < COLS; j++) begin : g_col
                mac_pe #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ACC_WIDTH (ACC_WIDTH)
                ) u_pe (
                    .clk          (clk),
                    .rst_n        (rst_n),
                    .clear_acc    (clear_acc),

                    .a_in         (a_wire  [i][j]),
                    .a_valid_in   (a_v_wire[i][j]),
                    .a_out        (a_wire  [i][j+1]),
                    .a_valid_out  (a_v_wire[i][j+1]),

                    .b_in         (b_wire  [i][j]),
                    .b_valid_in   (b_v_wire[i][j]),
                    .b_out        (b_wire  [i+1][j]),
                    .b_valid_out  (b_v_wire[i+1][j]),

                    .acc_out      (c_out      [i][j]),
                    .acc_valid_out(c_valid_out[i][j])
                );
            end
        end
    endgenerate

endmodule