`timescale 1ns/1ps

// matrix_engine_tb.sv
// Self-checking testbench for the matrix engine (M4).
//
// Unlike the M3 array testbench, this one does NOT skew anything: it streams
// plain k-slices (column k of A, row k of B) and relies on the engine to skew.
//
// Golden model: exp_c[i][j] accumulates sum_k A[i][k]*B[k][j] over the ACTIVE
// rows/columns, and is zeroed whenever a run starts with accumulate = 0 (or
// the engine is reset).
//
// What is checked on every run (run_tile):
//   - busy is low before start and high from the cycle after start
//   - done is low until exactly ROWS+COLS-1 cycles after the in_last slice,
//     high for exactly one cycle, then low again
//   - in the done cycle, every PE accumulator equals the golden model, and
//     it is still the same one cycle later (result is stable after done)
// Every cycle (monitor): done only while busy, never two cycles in a row.
//
// Protocol abuse that the engine must ignore (junk is always non-zero):
//   - slices (even with in_last) while idle
//   - a second start while streaming, or in the done cycle
//   - slices (even with in_last) after the real in_last slice
//   - in_last on a cycle without in_valid (every bubble carries in_last = 1)
//   - reset in the middle of a stream and in the middle of the drain
//
// M7: the testbench is width-aware. Operands span the full signed range of
// DATA_WIDTH (OP_MIN..OP_MAX) and the golden model wraps every sum to
// ACC_WIDTH bits (wrap_acc), exactly like the hardware accumulator.
//
// Accumulator wrap-around at engine level: the testbench deposits chosen
// values straight into every PE accumulator (white-box, seed_accumulators)
// while the engine is idle. Seeds sit just below the positive limit, just
// above the negative limit, near zero or anywhere in the range, so a short
// run is enough to push sums through the wrap point at ANY width, including
// the default 8 / 32. Directed wrap tests plus a seeded random regression.
//
// ROWS / COLS / DATA_WIDTH / ACC_WIDTH are module parameters so other sizes
// and widths can be run from the command line, e.g.
//   iverilog -Pmatrix_engine_tb.ROWS=3 -Pmatrix_engine_tb.COLS=5 \
//            -Pmatrix_engine_tb.DATA_WIDTH=6 -Pmatrix_engine_tb.ACC_WIDTH=12 ...
module matrix_engine_tb;

    parameter ROWS       = 4;
    parameter COLS       = 4;
    parameter DATA_WIDTH = 8;
    parameter ACC_WIDTH  = 32;
    parameter K_MAX      = 8;    // longest inner dimension we drive (keep >= 6)

    localparam DRAIN = ROWS + COLS - 1;   // expected done latency after in_last

    // Operand range follows DATA_WIDTH (signed): -2^(DW-1) .. 2^(DW-1)-1
    localparam int OP_MIN = -(1 << (DATA_WIDTH - 1));
    localparam int OP_MAX =  (1 << (DATA_WIDTH - 1)) - 1;

    // Protocol-abuse modes for run_tile()
    localparam int ABUSE_NONE         = 0;
    localparam int ABUSE_START_STREAM = 1;   // second start in the middle of the stream
    localparam int ABUSE_START_DONE   = 2;   // second start in the done cycle
    localparam int ABUSE_SLICE_DRAIN  = 3;   // valid slices (with in_last) after the real last slice

    // Non-zero junk on every lane/cycle that is not a valid slice
    localparam logic signed [DATA_WIDTH-1:0] JUNK_A = 8'sh55;
    localparam logic signed [DATA_WIDTH-1:0] JUNK_B = 8'shAA;

    logic clk;
    logic rst_n;
    logic start;
    logic accumulate;
    logic [$clog2(ROWS+1)-1:0] tile_m;
    logic [$clog2(COLS+1)-1:0] tile_n;
    logic busy;
    logic done;
    logic in_valid;
    logic in_last;
    logic signed [ROWS-1:0][DATA_WIDTH-1:0]          in_a;
    logic signed [COLS-1:0][DATA_WIDTH-1:0]          in_b;
    logic signed [ROWS-1:0][COLS-1:0][ACC_WIDTH-1:0] c_out;

    // Flat view of c_out so we can index it with loop variables
    logic [ROWS*COLS*ACC_WIDTH-1:0] c_flat;
    assign c_flat = c_out;

    matrix_engine #(
        .ROWS      (ROWS),
        .COLS      (COLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .accumulate(accumulate),
        .tile_m    (tile_m),
        .tile_n    (tile_n),
        .busy      (busy),
        .done      (done),
        .in_valid  (in_valid),
        .in_last   (in_last),
        .in_a      (in_a),
        .in_b      (in_b),
        .c_out     (c_out)
    );

    // Clock: 10ns period
    always #5 clk = ~clk;

    // Watchdog so a hung run can never spin forever
    initial begin
        #200_000_000;
        $display("TIMEOUT: testbench did not finish");
        $finish;
    end

    // ------------------------------------------------------------------
    // Golden-model state and counters
    // ------------------------------------------------------------------
    int A_mat [ROWS][K_MAX];
    int B_mat [K_MAX][COLS];
    longint exp_c [ROWS][COLS];

    int total_checks = 0;
    int total_fails  = 0;

    // True when the previous run_tile() deliberately stopped in its first
    // idle cycle, so that the next start lands in the earliest legal cycle.
    bit prev_tight = 1'b0;

    // Functional coverage (manual counters - no covergroups in this Icarus)
    int cov_k1             = 0;  // K = 1
    int cov_kmax           = 0;  // K = K_MAX
    int cov_stall_lead     = 0;  // bubbles before the first slice
    int cov_stall_mid      = 0;  // bubbles inside the stream
    int cov_partial_rows   = 0;  // fewer than ROWS active rows
    int cov_partial_cols   = 0;  // fewer than COLS active columns
    int cov_empty_tile     = 0;  // tile_m or tile_n = 0
    int cov_accum_no_clear = 0;  // run accumulated on top of the previous result
    int cov_extreme_ops    = 0;  // run used OP_MIN or OP_MAX operands
    int cov_neg_result     = 0;  // a PE was checked with a negative value
    int cov_pos_result     = 0;  // a PE was checked with a positive value
    int cov_back_to_back   = 0;  // start in the earliest legal cycle
    int cov_abuse_start_s  = 0;  // start while streaming
    int cov_abuse_start_d  = 0;  // start in the done cycle
    int cov_abuse_slice_d  = 0;  // slices after in_last
    int cov_idle_junk      = 0;  // slices while idle
    int cov_reset_stream   = 0;  // reset mid-stream
    int cov_reset_drain    = 0;  // reset mid-drain
    int cov_seeded         = 0;  // runs started from deposited accumulator values
    int cov_wrap_pos       = 0;  // a sum wrapped past the positive limit
    int cov_wrap_neg       = 0;  // a sum wrapped past the negative limit
    int bins_missed        = 0;  // coverage bins that were not hit

    integer rng_seed;

    function int rand_range(input int lo, input int hi);
        rand_range = lo + ($unsigned($random(rng_seed)) % (hi - lo + 1));
    endfunction

    // Wraps a value to ACC_WIDTH bits (signed), like the hardware accumulator
    function longint wrap_acc(input longint v);
        logic signed [63:0] t;
        t = v;
        if (ACC_WIDTH < 64) begin
            t = t <<< (64 - ACC_WIDTH);
            t = t >>> (64 - ACC_WIDTH);
        end
        wrap_acc = t;
    endfunction

    // Folds any integer into the operand range, so fixed test patterns stay
    // legal at every DATA_WIDTH (values already in range are unchanged)
    function int fit_op(input int v);
        longint r, t;
        r = longint'(OP_MAX) - OP_MIN + 1;
        t = (longint'(v) - OP_MIN) % r;
        if (t < 0) t += r;
        fit_op = int'(t + OP_MIN);
    endfunction

    // Operand with a bias towards the values that break things: the range
    // limits, -1, 0 and 1 (about 5 in 8), otherwise anywhere in the range
    function int rand_op();
        case (rand_range(0, 7))
            0:       rand_op = OP_MIN;
            1:       rand_op = OP_MAX;
            2:       rand_op = -1;
            3:       rand_op = 0;
            4:       rand_op = 1;
            default: rand_op = rand_range(OP_MIN, OP_MAX);
        endcase
    endfunction

    function int imin(input int a, input int b);
        imin = (a < b) ? a : b;
    endfunction

    function int imax(input int a, input int b);
        imax = (a > b) ? a : b;
    endfunction

    // ------------------------------------------------------------------
    // Cycle monitor: protocol rules that must hold on every single cycle
    // ------------------------------------------------------------------
    logic prev_done = 1'b0;
    always @(posedge clk) begin
        if (rst_n) begin
            total_checks++;
            if (done && !busy) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: done asserted while not busy", $time);
            end
            if (done && prev_done) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: done held for more than one cycle", $time);
            end
        end
        prev_done <= done;
    end

    // ------------------------------------------------------------------
    // Stream scheduling: which k is on the wire in slot s? Returns -1 for a
    // bubble. stall_len bubbles are inserted before element stall_at
    // (stall_at < 0 means no stall).
    // ------------------------------------------------------------------
    function int slot_to_k(input int s, input int k_len,
                           input int stall_at, input int stall_len);
        int k;
        k = s;
        if (stall_at >= 0 && s >= stall_at) begin
            if (s < stall_at + stall_len) k = -1;
            else                          k = s - stall_len;
        end
        if (k >= k_len) k = -1;
        slot_to_k = k;
    endfunction

    // ------------------------------------------------------------------
    // Small helpers
    // ------------------------------------------------------------------

    // Stream inputs idle: no valid slice, junk data, and in_last = 1 on
    // purpose - the engine must only honour in_last together with in_valid.
    task automatic drive_idle();
        in_valid = 1'b0;
        in_last  = 1'b1;
        for (int i = 0; i < ROWS; i++) in_a[i] = JUNK_A;
        for (int j = 0; j < COLS; j++) in_b[j] = JUNK_B;
    endtask

    // Valid-looking slice of random junk, with in_last set. Used to attack
    // the engine at times when it must not accept anything.
    task automatic drive_junk_slice();
        in_valid = 1'b1;
        in_last  = 1'b1;
        for (int i = 0; i < ROWS; i++) in_a[i] = rand_range(OP_MIN, OP_MAX);
        for (int j = 0; j < COLS; j++) in_b[j] = rand_range(OP_MIN, OP_MAX);
    endtask

    // Settings the engine must have captured at start; changed afterwards to
    // prove they are not re-read.
    task automatic scramble_config(input bit accumulate_i);
        start      = 1'b0;
        accumulate = ~accumulate_i;
        tile_m     = '1;
        tile_n     = '1;
    endtask

    // Golden model: add one product to PE (i,j), wrapping to ACC_WIDTH bits
    // and noting every time the sum really did wrap
    task automatic acc_add(input int i, input int j, input longint prod);
        longint raw, w;
        raw = exp_c[i][j] + prod;
        w   = wrap_acc(raw);
        if (ACC_WIDTH < 64 && w != raw) begin
            if (raw > 0) cov_wrap_pos++;
            else         cov_wrap_neg++;
        end
        exp_c[i][j] = w;
    endtask

    task automatic clear_golden();
        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;
    endtask

    // busy / done must both be low right now
    task automatic expect_idle(input string label);
        total_checks += 2;
        if (busy !== 1'b0) begin
            total_fails++;
            $display("FAIL [%s]: busy = %b, expected 0", label, busy);
        end
        if (done !== 1'b0) begin
            total_fails++;
            $display("FAIL [%s]: done = %b, expected 0", label, done);
        end
    endtask

    // Compares every PE's accumulator against the golden model
    task automatic check_all(input string label, input bit verbose);
        int fails_here = 0;
        logic signed [ACC_WIDTH-1:0] got;
        for (int i = 0; i < ROWS; i++) begin
            for (int j = 0; j < COLS; j++) begin
                got = c_flat[(i*COLS + j)*ACC_WIDTH +: ACC_WIDTH];
                total_checks++;
                if (got !== exp_c[i][j]) begin
                    fails_here++;
                    total_fails++;
                    $display("FAIL [%s]: C[%0d][%0d] = %0d, expected = %0d",
                             label, i, j, got, exp_c[i][j]);
                end
                if (exp_c[i][j] > 0) cov_pos_result++;
                if (exp_c[i][j] < 0) cov_neg_result++;
            end
        end
        if (verbose && fails_here == 0)
            $display("PASS [%s]: all %0d PEs match", label, ROWS*COLS);
    endtask

    task automatic sample_coverage(input int k_len, input int m_act, input int n_act,
                                   input bit accumulate_i,
                                   input int stall_at, input int abuse);
        bit extreme = 0;
        if (k_len == 1)     cov_k1++;
        if (k_len == K_MAX) cov_kmax++;
        if (stall_at == 0)  cov_stall_lead++;
        if (stall_at > 0)   cov_stall_mid++;
        if (m_act < ROWS)   cov_partial_rows++;
        if (n_act < COLS)   cov_partial_cols++;
        if (m_act == 0 || n_act == 0) cov_empty_tile++;
        if (accumulate_i)   cov_accum_no_clear++;
        if (prev_tight)     cov_back_to_back++;
        if (abuse == ABUSE_START_STREAM) cov_abuse_start_s++;
        if (abuse == ABUSE_START_DONE)   cov_abuse_start_d++;
        if (abuse == ABUSE_SLICE_DRAIN)  cov_abuse_slice_d++;
        for (int i = 0; i < m_act; i++)
            for (int k = 0; k < k_len; k++)
                if (A_mat[i][k] == OP_MIN || A_mat[i][k] == OP_MAX) extreme = 1;
        for (int k = 0; k < k_len; k++)
            for (int j = 0; j < n_act; j++)
                if (B_mat[k][j] == OP_MIN || B_mat[k][j] == OP_MAX) extreme = 1;
        if (extreme) cov_extreme_ops++;
    endtask

    // ------------------------------------------------------------------
    // One complete tile run: start, stream the k-slices, wait for done,
    // check timing and results.
    //   m_act / n_act : active rows / columns (partial tiles). The inactive
    //                   lanes still carry live matrix data, to prove the
    //                   engine masks them.
    //   stall_at/len  : optional bubble inside the stream (all lanes).
    //   abuse         : ABUSE_* mode (see top of file)
    //   tight_end     : stop in the first idle cycle, so that the next
    //                   run_tile() starts in the earliest legal cycle
    // ------------------------------------------------------------------
    task automatic run_tile(input string label, input int k_len,
                            input int m_act, input int n_act,
                            input bit accumulate_i,
                            input int stall_at, input int stall_len,
                            input int abuse, input bit tight_end,
                            input bit verbose);
        int n_slots, k, d, abuse_slot;
        bit exp_done, exp_busy;

        sample_coverage(k_len, m_act, n_act, accumulate_i, stall_at, abuse);

        // Engine must be idle before we start
        total_checks++;
        if (busy !== 1'b0) begin
            total_fails++;
            $display("FAIL [%s]: busy = %b before start, expected 0", label, busy);
        end

        // --- start pulse (one cycle) ---
        start      = 1'b1;
        accumulate = accumulate_i;
        tile_m     = m_act;
        tile_n     = n_act;
        @(posedge clk);
        #1;
        scramble_config(accumulate_i);

        if (!accumulate_i) clear_golden();

        // --- stream the slices (one per cycle, bubbles as scheduled) ---
        n_slots    = k_len + ((stall_at >= 0 && stall_at <= k_len - 1) ? stall_len : 0);
        abuse_slot = n_slots / 2;
        for (int s = 0; s < n_slots; s++) begin
            k = slot_to_k(s, k_len, stall_at, stall_len);
            if (k >= 0) begin
                in_valid = 1'b1;
                in_last  = (k == k_len - 1);
                for (int i = 0; i < ROWS; i++) in_a[i] = A_mat[i][k];
                for (int j = 0; j < COLS; j++) in_b[j] = B_mat[k][j];
            end else begin
                drive_idle();
            end
            if (abuse == ABUSE_START_STREAM && s == abuse_slot) begin
                // A second start that would clear everything and shrink the
                // tile, if the engine were foolish enough to take it
                start = 1'b1; accumulate = 1'b0; tile_m = 1; tile_n = 1;
            end
            total_checks++;
            if (busy !== 1'b1) begin
                total_fails++;
                $display("FAIL [%s]: busy = %b while streaming slot %0d, expected 1", label, busy, s);
            end
            @(posedge clk);
            #1;
            scramble_config(accumulate_i);
        end
        drive_idle();

        // Golden model: accumulate this product on the active PEs only
        for (int i = 0; i < m_act; i++)
            for (int j = 0; j < n_act; j++)
                for (int kk = 0; kk < k_len; kk++)
                    acc_add(i, j, longint'(A_mat[i][kk]) * B_mat[kk][j]);

        // --- wait for done. We are now in cycle T+1, T = cycle of in_last ---
        for (d = 1; d <= DRAIN + 1; d++) begin
            drive_idle();
            if (abuse == ABUSE_SLICE_DRAIN && d <= DRAIN) drive_junk_slice();
            if (abuse == ABUSE_START_DONE  && d == DRAIN) begin
                start = 1'b1; accumulate = 1'b0; tile_m = 1; tile_n = 1;
            end

            exp_done = (d == DRAIN);
            exp_busy = (d <= DRAIN);
            total_checks += 2;
            if (done !== exp_done) begin
                total_fails++;
                $display("FAIL [%s]: done = %b at %0d cycles after in_last, expected %b (latency should be %0d)",
                         label, done, d, exp_done, DRAIN);
            end
            if (busy !== exp_busy) begin
                total_fails++;
                $display("FAIL [%s]: busy = %b at %0d cycles after in_last, expected %b",
                         label, busy, d, exp_busy);
            end
            if (d == DRAIN)     check_all(label, verbose);
            if (d == DRAIN + 1) check_all($sformatf("%s-stable", label), 1'b0);

            // In tight mode we leave in the first idle cycle without
            // advancing, so the next start lands right there.
            if (!(tight_end && d == DRAIN + 1)) begin
                @(posedge clk);
                #1;
                scramble_config(accumulate_i);
            end
        end
        drive_idle();
        prev_tight = tight_end;
    endtask

    // Junk valid slices (random data, random in_last) while the engine is
    // idle. Nothing may change: no busy, no done, accumulators untouched.
    task automatic idle_junk(input string label, input int num_cycles);
        prev_tight = 1'b0;
        cov_idle_junk++;
        for (int c = 0; c < num_cycles; c++) begin
            drive_junk_slice();
            in_last = (rand_range(0, 1) == 1);
            total_checks++;
            if (busy !== 1'b0) begin
                total_fails++;
                $display("FAIL [%s]: busy = %b during idle junk, expected 0", label, busy);
            end
            @(posedge clk);
            #1;
        end
        drive_idle();
        // Let anything that was wrongly accepted travel through the array
        for (int d = 1; d <= DRAIN + 1; d++) begin
            expect_idle(label);
            @(posedge clk);
            #1;
        end
        check_all(label, 1'b1);
    endtask

    // Reset in the middle of a run. phase 0: mid-stream, phase 1: mid-drain
    // (the in_last token is still travelling). Reset must return the engine
    // to idle, zero every accumulator, and leave nothing behind that could
    // produce a late done or corrupt the next run.
    task automatic reset_mid_run(input string label, input int phase);
        prev_tight = 1'b0;
        fill_pattern(9);

        // start, accumulating on top of whatever is in the PEs
        start = 1'b1; accumulate = 1'b1; tile_m = ROWS; tile_n = COLS;
        @(posedge clk);
        #1;
        scramble_config(1'b1);

        if (phase == 0) begin
            for (int s = 0; s < imin(3, K_MAX); s++) begin
                in_valid = 1'b1;
                in_last  = 1'b0;
                for (int i = 0; i < ROWS; i++) in_a[i] = A_mat[i][s];
                for (int j = 0; j < COLS; j++) in_b[j] = B_mat[s][j];
                @(posedge clk);
                #1;
            end
            cov_reset_stream++;
        end else begin
            for (int s = 0; s < 2; s++) begin
                in_valid = 1'b1;
                in_last  = (s == 1);
                for (int i = 0; i < ROWS; i++) in_a[i] = A_mat[i][s];
                for (int j = 0; j < COLS; j++) in_b[j] = B_mat[s][j];
                @(posedge clk);
                #1;
            end
            drive_idle();
            for (int w = 0; w < DRAIN / 2; w++) begin
                @(posedge clk);
                #1;
            end
            cov_reset_drain++;
        end

        // Asynchronous reset with the stream still "live"
        rst_n = 1'b0;
        #1;
        expect_idle({label, "-at-reset"});
        clear_golden();
        check_all({label, "-zeroed"}, 1'b1);
        @(posedge clk);
        #1;
        rst_n = 1'b1;
        drive_idle();

        // No zombie done / busy afterwards
        for (int w = 0; w < DRAIN + 2; w++) begin
            expect_idle({label, "-after"});
            @(posedge clk);
            #1;
        end
        check_all({label, "-still-zero"}, 1'b0);
    endtask

    // ------------------------------------------------------------------
    // Matrix fillers
    // ------------------------------------------------------------------
    task automatic fill_const(input int a_val, input int b_val);
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++) A_mat[i][k] = a_val;
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = b_val;
    endtask

    // Deterministic, asymmetric, mixed-sign pattern (a transposed or
    // mis-wired engine cannot accidentally match)
    task automatic fill_pattern(input int salt);
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++)
                A_mat[i][k] = fit_op(((i*5 + k*3 + salt) % 17) - 8);
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++)
                B_mat[k][j] = fit_op(((k*7 + j*2 + salt*3) % 15) - 7);
    endtask

    task automatic fill_random();
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++) A_mat[i][k] = rand_range(OP_MIN, OP_MAX);
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = rand_range(OP_MIN, OP_MAX);
    endtask

    // Like fill_random, but every operand comes from rand_op (limits, -1, 0, 1
    // or anywhere in the range)
    task automatic fill_mixed();
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++) A_mat[i][k] = rand_op();
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = rand_op();
    endtask

    // ------------------------------------------------------------------
    // White-box accumulator seeding (accumulator wrap-around tests)
    // The generate block hands every PE its own always block; ->seed_now makes
    // them all write seed_flat into the PE accumulators. Call it only while
    // the engine is idle (not tight after a run), a short time after a clock edge.
    // ------------------------------------------------------------------
    logic [ROWS*COLS*ACC_WIDTH-1:0] seed_flat;
    event seed_now;

    generate
        for (genvar gi = 0; gi < ROWS; gi++) begin : g_seed_row
            for (genvar gj = 0; gj < COLS; gj++) begin : g_seed_col
                always @(seed_now)
                    dut.u_array.g_row[gi].g_col[gj].u_pe.u_mac.acc_out =
                        seed_flat[(gi*COLS + gj)*ACC_WIDTH +: ACC_WIDTH];
            end
        end
    endgenerate

    // mode 0: just below the positive limit, spread so that some PEs wrap
    //         during a short run and some do not
    // mode 1: just above the negative limit, same idea
    // mode 2: random mix - near either limit, near zero, or anywhere
    // The golden model is updated with the same values, then the PEs are
    // compared with it (proves the deposit landed).
    task automatic seed_accumulators(input string label, input int mode);
        longint lim_pos, lim_neg, unit_p, v;
        int idx, pick;
        lim_pos = (longint'(1) << (ACC_WIDTH - 1)) - 1;
        lim_neg = -(longint'(1) << (ACC_WIDTH - 1));
        unit_p  = (longint'(OP_MAX) * OP_MAX) / 2;
        if (unit_p < 1) unit_p = 1;
        for (int i = 0; i < ROWS; i++) begin
            for (int j = 0; j < COLS; j++) begin
                idx  = i*COLS + j;
                pick = (mode == 2) ? rand_range(0, 3) : mode;
                case (pick)
                    0: v = lim_pos - (mode == 2 ? rand_range(0, 40) : (idx % 11)) * unit_p;
                    1: v = lim_neg + (mode == 2 ? rand_range(0, 40) : (idx % 11)) * unit_p;
                    2: v = rand_range(-3, 3);
                    default: begin
                        v = {$random(rng_seed), $random(rng_seed)};
                    end
                endcase
                exp_c[i][j] = wrap_acc(v);
                seed_flat[(i*COLS + j)*ACC_WIDTH +: ACC_WIDTH] = exp_c[i][j];
            end
        end
        -> seed_now;
        #1;
        cov_seeded++;
        check_all({label, "-seeded"}, 1'b0);
    endtask

    // Constrained-random regression: random K, partial tiles, bubbles,
    // accumulate-or-clear, protocol abuse, back-to-back starts and idle junk.
    // Fixed seed -> a failure at run N reproduces at run N every time.
    task automatic random_regression(input int num_runs, input int init_seed);
        int k_len, m_act, n_act, stall_at, stall_len, abuse;
        bit acc, tight;
        rng_seed = init_seed;
        for (int r = 0; r < num_runs; r++) begin
            if (rand_range(0, 5) == 0)
                idle_junk($sformatf("random-%0d-idle-junk", r), rand_range(1, 4));

            k_len = rand_range(1, K_MAX);
            m_act = ROWS;
            n_act = COLS;
            if (rand_range(0, 9) >= 7) begin   // ~30% partial tiles
                m_act = rand_range(1, ROWS);
                n_act = rand_range(1, COLS);
            end
            stall_at  = -1;
            stall_len = 0;
            if (rand_range(0, 3) == 0) begin   // ~25% with a stall
                stall_at  = rand_range(0, k_len - 1);
                stall_len = rand_range(1, 3);
            end
            acc   = (rand_range(0, 3) == 0);    // ~25% accumulate
            abuse = (rand_range(0, 3) == 0) ? rand_range(1, 3) : ABUSE_NONE;  // ~25% abuse
            tight = (rand_range(0, 3) == 0);    // ~25% next start is immediate
            fill_random();
            run_tile($sformatf("random-%0d", r), k_len, m_act, n_act,
                     acc, stall_at, stall_len, abuse, tight, 1'b0);
        end
        // leave the engine in a normal idle cycle
        if (prev_tight) begin
            @(posedge clk);
            #1;
            prev_tight = 1'b0;
        end
    endtask

    // Wrap-around regression: every run starts from deposited accumulator
    // values and accumulates on top of them, with limit-biased operands.
    // Inactive PEs (partial tiles) must keep their seeds untouched.
    task automatic wrap_regression(input int num_runs, input int init_seed);
        int k_len, m_act, n_act, stall_at, stall_len, abuse;
        rng_seed = init_seed;
        for (int r = 0; r < num_runs; r++) begin
            k_len = rand_range(1, K_MAX);
            m_act = ROWS;
            n_act = COLS;
            if (rand_range(0, 9) >= 7) begin
                m_act = rand_range(1, ROWS);
                n_act = rand_range(1, COLS);
            end
            stall_at  = -1;
            stall_len = 0;
            if (rand_range(0, 3) == 0) begin
                stall_at  = rand_range(0, k_len - 1);
                stall_len = rand_range(1, 3);
            end
            abuse = (rand_range(0, 3) == 0) ? rand_range(1, 3) : ABUSE_NONE;
            fill_mixed();
            seed_accumulators($sformatf("wrap-%0d", r), 2);
            run_tile($sformatf("wrap-%0d", r), k_len, m_act, n_act,
                     1'b1, stall_at, stall_len, abuse, 1'b0, 1'b0);
        end
    endtask

    task automatic report_bin(input string name, input int hits, input bit possible);
        if (!possible)      $display("  %-24s: n/a for this size", name);
        else if (hits > 0)  $display("  %-24s: %0d hits [HIT]", name, hits);
        else begin
            $display("  %-24s: %0d hits [MISS]", name, hits);
            bins_missed++;
        end
    endtask

    task automatic report_coverage();
        $display("==============================================");
        $display("COVERAGE:");
        report_bin("K = 1",                    cov_k1,             1);
        report_bin("K = K_MAX",                cov_kmax,           1);
        report_bin("leading stall",            cov_stall_lead,     1);
        report_bin("mid-stream stall",         cov_stall_mid,      1);
        report_bin("partial rows",             cov_partial_rows,   ROWS > 1);
        report_bin("partial columns",          cov_partial_cols,   COLS > 1);
        report_bin("empty tile",               cov_empty_tile,     1);
        report_bin("accumulate w/o clear",     cov_accum_no_clear, 1);
        report_bin("extreme operands",         cov_extreme_ops,    1);
        report_bin("negative PE result",       cov_neg_result,     1);
        report_bin("positive PE result",       cov_pos_result,     1);
        report_bin("back-to-back start",       cov_back_to_back,   1);
        report_bin("abuse: start in stream",   cov_abuse_start_s,  1);
        report_bin("abuse: start in done",     cov_abuse_start_d,  1);
        report_bin("abuse: slices after last", cov_abuse_slice_d,  1);
        report_bin("abuse: slices while idle", cov_idle_junk,      1);
        report_bin("reset mid-stream",         cov_reset_stream,   1);
        report_bin("reset mid-drain",          cov_reset_drain,    1);
        report_bin("seeded accumulators",      cov_seeded,         1);
        report_bin("wrap past positive limit", cov_wrap_pos,       ACC_WIDTH < 64);
        report_bin("wrap past negative limit", cov_wrap_neg,       ACC_WIDTH < 64);
        $display("==============================================");
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    initial begin
        $dumpfile("matrix_engine.vcd");
        $dumpvars(0, matrix_engine_tb);

        clk = 0;
        rst_n = 0;
        start = 0;
        accumulate = 0;
        tile_m = '0;
        tile_n = '0;
        drive_idle();
        rng_seed = 42;
        clear_golden();

        // Step 1: release reset, engine idle, every accumulator zero
        @(posedge clk);
        #1 rst_n = 1;
        @(posedge clk);
        #1;
        expect_idle("post-reset");
        check_all("post-reset", 1);

        // Step 2: identity x B must return the top rows of B
        fill_pattern(1);
        if (ROWS <= K_MAX) begin
            for (int i = 0; i < ROWS; i++)
                for (int k = 0; k < K_MAX; k++) A_mat[i][k] = (i == k) ? 1 : 0;
            run_tile("identity", ROWS, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        end

        // Step 3: mixed-sign asymmetric pattern, full inner dimension
        fill_pattern(2);
        run_tile("pattern-full", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 4: extreme operands
        fill_const(OP_MIN, OP_MIN);
        run_tile("extreme-neg*neg", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        fill_const(OP_MIN, OP_MAX);
        run_tile("extreme-neg*pos", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        fill_const(OP_MAX, OP_MAX);
        run_tile("extreme-pos*pos", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 4b: the same extremes with an ODD number of products. With an
        // even K the largest product (OP_MIN*OP_MIN = 2^(2*DW-2)) can add up
        // to exactly 2^ACC_WIDTH and wrap to zero, hiding a wrong product.
        fill_const(OP_MIN, OP_MIN);
        run_tile("extreme-neg*neg-K1", 1, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("extreme-neg*neg-K5", 5, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        fill_const(OP_MIN, OP_MAX);
        run_tile("extreme-neg*pos-K5", 5, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        fill_const(OP_MAX, OP_MAX);
        run_tile("extreme-pos*pos-K5", 5, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 5: K = 1 (single outer product)
        fill_pattern(3);
        run_tile("K1-outer-product", 1, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 6: bubbles - leading, in the middle, right before the last slice
        fill_pattern(4);
        run_tile("stall-mid",      imin(6, K_MAX), ROWS, COLS, 0, 3, 2, ABUSE_NONE, 0, 1);
        run_tile("stall-leading",  5,              ROWS, COLS, 0, 0, 3, ABUSE_NONE, 0, 1);
        run_tile("stall-pre-last", 5,              ROWS, COLS, 0, 4, 2, ABUSE_NONE, 0, 1);

        // Step 7: partial tiles - inactive lanes carry live data
        fill_pattern(5);
        run_tile("partial-2x3",       5, imin(2, ROWS), imin(3, COLS), 0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("partial-1x1",       4, 1,             1,             0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("partial-all-x1",    4, ROWS,          1,             0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("partial-1-xall",    4, 1,             COLS,          0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("partial-rows-only", 4, imax(1, ROWS - 1), COLS,      0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("partial-cols-only", 4, ROWS, imax(1, COLS - 1),      0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 8: empty tile - nothing fires, accumulators end up cleared
        run_tile("empty-m0", 4, 0,    COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        run_tile("empty-n0", 4, ROWS, 0,    0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 9: K-chunking - three runs accumulating onto each other
        fill_pattern(6);
        run_tile("accum-part1", 4, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        fill_pattern(7);
        run_tile("accum-part2", 4, ROWS, COLS, 1, -1, 0, ABUSE_NONE, 0, 1);
        fill_pattern(8);
        run_tile("accum-part3", 3, ROWS, COLS, 1, 1, 2, ABUSE_NONE, 0, 1);

        // Step 9b: accumulator wrap-around at engine level. Accumulators are
        // seeded next to a limit, then a short accumulating run pushes the
        // sums over it. Golden = wrap_acc; wrap bins prove it really wrapped.
        fill_const(OP_MAX, OP_MAX);
        seed_accumulators("wrap-pos", 0);
        run_tile("wrap-pos-up", 4, ROWS, COLS, 1, -1, 0, ABUSE_NONE, 0, 1);
        // ...and back across the limit with negative products
        fill_const(OP_MIN, OP_MAX);
        run_tile("wrap-pos-back", 4, ROWS, COLS, 1, -1, 0, ABUSE_NONE, 0, 1);
        fill_const(OP_MIN, OP_MAX);
        seed_accumulators("wrap-neg", 1);
        run_tile("wrap-neg-down", 4, ROWS, COLS, 1, -1, 0, ABUSE_NONE, 0, 1);
        fill_const(OP_MAX, OP_MAX);
        run_tile("wrap-neg-back", 4, ROWS, COLS, 1, -1, 0, ABUSE_NONE, 0, 1);
        // wrap with partial tiles: inactive PEs keep their seeds
        fill_const(OP_MAX, OP_MAX);
        seed_accumulators("wrap-partial", 0);
        run_tile("wrap-partial", 4, imax(1, ROWS - 1), imax(1, COLS - 1), 1, -1, 0, ABUSE_NONE, 0, 1);
        // a clearing run wipes a wrapped accumulator
        run_tile("wrap-then-clear", 3, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 10: protocol abuse - the engine must shrug all of this off
        fill_pattern(2);
        run_tile("abuse-start-in-stream", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_START_STREAM, 0, 1);
        run_tile("abuse-start-in-done",   K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_START_DONE,   0, 1);
        run_tile("abuse-slices-in-drain", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_SLICE_DRAIN,  0, 1);
        idle_junk("idle-junk", 6);

        // Step 11: back-to-back runs, each start in the earliest legal cycle
        fill_pattern(3);
        run_tile("b2b-1", 3, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 1, 1);
        fill_pattern(4);
        run_tile("b2b-2", 4, ROWS, COLS, 1, -1, 0, ABUSE_NONE, 1, 1);
        fill_pattern(5);
        run_tile("b2b-3", 2, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 12: reset in the middle of a stream and of a drain; the engine
        // must work normally right afterwards
        reset_mid_run("reset-stream", 0);
        fill_pattern(2);
        run_tile("after-reset-stream", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);
        reset_mid_run("reset-drain", 1);
        fill_pattern(3);
        run_tile("after-reset-drain", K_MAX, ROWS, COLS, 0, -1, 0, ABUSE_NONE, 0, 1);

        // Step 13: constrained-random regression
        random_regression(200, 42);

        // Step 14: accumulator wrap-around regression (seeded accumulators)
        wrap_regression(80, 7);

        $display("==============================================");
        $display("SUMMARY: %0d / %0d checks passed (%0d failed)",
                  total_checks - total_fails, total_checks, total_fails);
        report_coverage();
        if (total_fails == 0 && bins_missed == 0) $display("RESULT: PASS");
        else                                       $display("RESULT: FAIL (%0d failed checks, %0d missed bins)",
                                                            total_fails, bins_missed);
        $display("Testbench complete.");
        $finish;
    end

endmodule