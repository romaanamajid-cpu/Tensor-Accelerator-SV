// skew_buffer.sv
// Per-lane delay line used to skew the operand streams of a systolic array.
//
// Lane i is delayed by exactly i clock cycles (lane 0 is a plain wire, lane 1
// has one register stage, lane 2 has two, ...). Each lane carries its valid
// bit through the same registers as its data, so bubbles and idle lanes are
// delayed together with the data they belong to.
//
// Used twice by the matrix engine:
//   - A side: LANES = ROWS, delays row i of A by i cycles
//   - B side: LANES = COLS, delays column j of B by j cycles
// so that A[i][k] and B[k][j] arrive at PE(i,j) in the same cycle.
module skew_buffer #(
    parameter LANES      = 4,
    parameter DATA_WIDTH = 8
)(
    input  logic                                    clk,
    input  logic                                    rst_n,

    input  logic signed [LANES-1:0][DATA_WIDTH-1:0] in_data,
    input  logic        [LANES-1:0]                 in_valid,

    output logic signed [LANES-1:0][DATA_WIDTH-1:0] out_data,
    output logic        [LANES-1:0]                 out_valid
);

    // ------------------------------------------------------------------
    // Parameter legality checks (run once, at time 0)
    // ------------------------------------------------------------------
    initial begin
        if (LANES < 1)
            $fatal(1, "skew_buffer: LANES (%0d) must be >= 1", LANES);
        if (DATA_WIDTH < 1)
            $fatal(1, "skew_buffer: DATA_WIDTH (%0d) must be >= 1", DATA_WIDTH);
    end

    generate
        for (genvar i = 0; i < LANES; i++) begin : g_lane
            if (i == 0) begin : g_no_delay
                // Lane 0 is not delayed at all
                assign out_data [i] = in_data [i];
                assign out_valid[i] = in_valid[i];
            end else begin : g_delay
                // i register stages: stage 0 samples the input, stage i-1
                // drives the output.
                logic signed [DATA_WIDTH-1:0] data_pipe  [i];
                logic                         valid_pipe [i];

                always_ff @(posedge clk or negedge rst_n) begin
                    if (!rst_n) begin
                        for (int s = 0; s < i; s++) begin
                            data_pipe [s] <= '0;
                            valid_pipe[s] <= 1'b0;
                        end
                    end else begin
                        data_pipe [0] <= in_data [i];
                        valid_pipe[0] <= in_valid[i];
                        for (int s = 1; s < i; s++) begin
                            data_pipe [s] <= data_pipe [s-1];
                            valid_pipe[s] <= valid_pipe[s-1];
                        end
                    end
                end

                assign out_data [i] = data_pipe [i-1];
                assign out_valid[i] = valid_pipe[i-1];
            end
        end
    endgenerate

endmodule