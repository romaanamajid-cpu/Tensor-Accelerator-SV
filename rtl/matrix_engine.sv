// matrix_engine.sv
// Matrix engine: computes C (+)= A x B for one ROWS x COLS tile.
//
// It wraps the verified mac_array and takes over the work that the M3
// testbench used to do by hand: skewing the operands. The caller just streams
// one "k-slice" per cycle (a column of A and the matching row of B) and the
// engine delays row i of A by i cycles and column j of B by j cycles so that
// A[i][k] and B[k][j] meet at PE(i,j) in the same cycle.
//
// Protocol (one tile = one run)
//   1. Pulse `start` for one cycle while `busy` is low. `tile_m` / `tile_n`
//      (number of active rows / columns) are captured on that edge.
//        accumulate = 0 : every PE accumulator is cleared by this start
//        accumulate = 1 : accumulators keep their value (K-chunking)
//   2. From the NEXT cycle on, drive the stream. A cycle with in_valid = 1
//      carries slice k: in_a[i] = A[i][k], in_b[j] = B[k][j]. Cycles with
//      in_valid = 0 are bubbles (stalls) and are fine anywhere in the stream.
//      The final slice must also set in_last. Exactly one in_last per run.
//   3. `done` pulses for one cycle, ROWS+COLS-1 cycles after the in_last
//      slice. In that cycle c_out holds the finished tile. c_out stays stable
//      until the next start or the next accepted slice.
//
// Inputs are only accepted while the engine is streaming: a start while busy
// and slices while idle (or after in_last) are ignored.
module matrix_engine #(
    parameter ROWS       = 4,
    parameter COLS       = 4,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                                            clk,
    input  logic                                            rst_n,

    // Control
    input  logic                                            start,
    input  logic                                            accumulate,
    input  logic [$clog2(ROWS+1)-1:0]                       tile_m,   // active rows
    input  logic [$clog2(COLS+1)-1:0]                       tile_n,   // active columns
    output logic                                            busy,
    output logic                                            done,

    // Operand stream (one k-slice per valid cycle)
    input  logic                                            in_valid,
    input  logic                                            in_last,
    input  logic signed [ROWS-1:0][DATA_WIDTH-1:0]          in_a,
    input  logic signed [COLS-1:0][DATA_WIDTH-1:0]          in_b,

    // Result tile, c_out[i][j] = C[i][j]
    output logic signed [ROWS-1:0][COLS-1:0][ACC_WIDTH-1:0] c_out
);

    // ------------------------------------------------------------------
    // Parameter legality checks (run once, at time 0). The width rules
    // (DATA_WIDTH, ACC_WIDTH) are checked inside mac_unit.
    // ------------------------------------------------------------------
    initial begin
        if (ROWS < 1)
            $fatal(1, "matrix_engine: ROWS (%0d) must be >= 1", ROWS);
        if (COLS < 1)
            $fatal(1, "matrix_engine: COLS (%0d) must be >= 1", COLS);
    end

    // The last slice enters the array at cycle T. Its operands reach the far
    // corner PE(ROWS-1, COLS-1) at T + (ROWS-1) + (COLS-1), and the finished
    // accumulator is visible one cycle later: T + ROWS + COLS - 1.
    localparam DRAIN_CYCLES = ROWS + COLS - 1;

    // ------------------------------------------------------------------
    // Control: IDLE -> STREAM -> DRAIN -> IDLE
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {S_IDLE, S_STREAM, S_DRAIN} state_t;
    state_t state;

    logic [$clog2(ROWS+1)-1:0] tile_m_r;
    logic [$clog2(COLS+1)-1:0] tile_n_r;
    logic [DRAIN_CYCLES-1:0]   last_pipe;   // in_last token travelling to the corner

    logic start_ok;      // a start that is actually accepted
    logic accept;        // a slice that is actually accepted
    logic accept_last;   // ... and it is the final one
    logic clear_acc;

    assign start_ok    = start && (state == S_IDLE);
    assign accept      = in_valid && (state == S_STREAM);
    assign accept_last = accept && in_last;
    assign clear_acc   = start_ok && !accumulate;
    assign busy        = (state != S_IDLE);
    assign done        = last_pipe[DRAIN_CYCLES-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            tile_m_r  <= '0;
            tile_n_r  <= '0;
            last_pipe <= '0;
        end else begin
            last_pipe <= (last_pipe << 1) | accept_last;

            case (state)
                S_IDLE: if (start_ok) begin
                    state    <= S_STREAM;
                    tile_m_r <= tile_m;
                    tile_n_r <= tile_n;
                end
                S_STREAM: if (accept_last) state <= S_DRAIN;
                S_DRAIN:  if (done)        state <= S_IDLE;
                default:                   state <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Runtime self-checks (simulation only, hidden from synthesis). Each one is
    // a rule that holds on every cycle, even when the caller misbehaves, so a
    // violation means a bug in this module, never a bad input - except the
    // last check, which guards the one input rule the engine relies on.
    // ------------------------------------------------------------------
`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (state > S_DRAIN)
                $fatal(1, "matrix_engine: illegal state %0d", state);
            // At most one in_last token travels to the corner at a time
            if ((last_pipe & (last_pipe - 1'b1)) != '0)
                $fatal(1, "matrix_engine: more than one in_last token in flight (%b)", last_pipe);
            // ... and it exists exactly while the engine is draining
            if (state != S_DRAIN && last_pipe != '0)
                $fatal(1, "matrix_engine: in_last token in flight outside DRAIN (state %0d)", state);
            if (state == S_DRAIN && last_pipe == '0)
                $fatal(1, "matrix_engine: DRAIN without an in_last token in flight");
            // Slices are only taken while streaming
            if (accept && state != S_STREAM)
                $fatal(1, "matrix_engine: slice accepted outside STREAM (state %0d)", state);
            // A start that is taken must describe a tile that exists
            if (start_ok && (tile_m > ROWS || tile_n > COLS))
                $fatal(1, "matrix_engine: tile_m=%0d / tile_n=%0d exceed ROWS=%0d / COLS=%0d at start",
                       tile_m, tile_n, ROWS, COLS);
        end
    end
`endif

    // ------------------------------------------------------------------
    // Partial tiles: lanes beyond the tile are marked invalid, so the PEs
    // on them never fire (their data is ignored, whatever it is).
    // ------------------------------------------------------------------
    logic [ROWS-1:0] a_lane_valid;
    logic [COLS-1:0] b_lane_valid;

    always_comb begin
        for (int i = 0; i < ROWS; i++) a_lane_valid[i] = accept && (i < tile_m_r);
        for (int j = 0; j < COLS; j++) b_lane_valid[j] = accept && (j < tile_n_r);
    end

    // ------------------------------------------------------------------
    // Skew: row i of A delayed i cycles, column j of B delayed j cycles
    // ------------------------------------------------------------------
    logic signed [ROWS-1:0][DATA_WIDTH-1:0] a_skewed;
    logic        [ROWS-1:0]                 a_skewed_valid;
    logic signed [COLS-1:0][DATA_WIDTH-1:0] b_skewed;
    logic        [COLS-1:0]                 b_skewed_valid;

    skew_buffer #(
        .LANES     (ROWS),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_skew_a (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_data  (in_a),
        .in_valid (a_lane_valid),
        .out_data (a_skewed),
        .out_valid(a_skewed_valid)
    );

    skew_buffer #(
        .LANES     (COLS),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_skew_b (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_data  (in_b),
        .in_valid (b_lane_valid),
        .out_data (b_skewed),
        .out_valid(b_skewed_valid)
    );

    // ------------------------------------------------------------------
    // The verified M3 array. Its per-PE valid outputs are not needed here:
    // `done` already tells the caller when the whole tile is final.
    // ------------------------------------------------------------------
    mac_array #(
        .ROWS      (ROWS),
        .COLS      (COLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_array (
        .clk        (clk),
        .rst_n      (rst_n),
        .clear_acc  (clear_acc),
        .a_in       (a_skewed),
        .a_valid_in (a_skewed_valid),
        .b_in       (b_skewed),
        .b_valid_in (b_skewed_valid),
        .c_out      (c_out),
        .c_valid_out()
    );

endmodule