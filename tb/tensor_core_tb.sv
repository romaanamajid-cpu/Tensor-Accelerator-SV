`timescale 1ns/1ps

// tensor_core_tb.sv
// Self-checking testbench for the tensor core (buffers + controller + engine).
// VERSION 4 (M5 step 5): row-serial result stream, partial tiles, accumulation /
// K-chunking, rejected settings, protocol abuse and reset in the middle of a job.
//
// The testbench plays the host: it loads k-slices into the operand buffers,
// pulses job_start, waits for job_done, reads the result stream and compares
// everything with a golden model. exp_c[i][j] accumulates
// sum_k A[i][k]*B[k][j] over the ACTIVE rows/columns of each job and is zeroed
// whenever a job runs with accumulate=0.
//
// Checked on every job (run_job):
//   - busy low before the job, high from the cycle after job_start until the
//     last result row (or the job_done cycle when nothing is streamed), low
//     again the cycle after
//   - job_done exactly K + ROWS + COLS cycles after the job_start cycle (for
//     partial tiles too), one cycle wide
//   - every C[i][j] inside the engine equals the golden model in the job_done
//     cycle (white-box look at dut.c_tile, so inactive PEs are checked too)
//   - result stream: when cfg_emit = 1 and m > 0, rows 0..m-1 arrive in the
//     m cycles after job_done, one per cycle, with the right res_row, data
//     (columns >= n read 0) and res_last on the final row; with cfg_emit = 0
//     or m = 0 nothing is streamed
//   - busy stays high until the last row has gone out, low the cycle after
//   - job_err only in the cycle after a deliberate bad job_start
// Every cycle (monitor): job_done only while busy, never two cycles in a row;
// res_valid only while busy, never with job_done; res_last only with res_valid;
// res_data is 0 whenever res_valid is low; every job_err pulse and every stream
// row in the whole run must be one the testbench provoked / expected.
//
// Abuse the core must shrug off (junk is never zero):
//   - job_start while busy, in every phase (LAUNCH, STREAM, DRAIN, done cycle,
//     readout)
//     -> job_err, running job undisturbed
//   - job_start with cfg_k = 0, cfg_k > K_DEPTH, cfg_m > ROWS, cfg_n > COLS
//     -> job_err, nothing starts, results untouched
//   - loads (ld_en) while busy (also during readout) -> ignored
// Reset (async, active low) pulled in every phase of a job - LAUNCH, STREAM,
// DRAIN, the done cycle, first and last result row: all outputs go idle at
// once, the engine sums are cleared, and the next job runs correctly.
// Also checked: a load in the SAME cycle as an accepted job_start is visible
// to that job ("load-and-go").
//
// Sizes are module parameters, e.g.
//   iverilog -Ptensor_core_tb.ROWS=3 -Ptensor_core_tb.COLS=5 -Ptensor_core_tb.K_DEPTH=8 ...
module tensor_core_tb;

    parameter ROWS       = 4;
    parameter COLS       = 4;
    parameter DATA_WIDTH = 8;
    parameter ACC_WIDTH  = 32;
    parameter K_DEPTH    = 16;

    localparam ADDR_W = (K_DEPTH > 1) ? $clog2(K_DEPTH) : 1;
    localparam K_W    = $clog2(K_DEPTH + 1);
    localparam M_W    = $clog2(ROWS + 1);
    localparam N_W    = $clog2(COLS + 1);

    // Largest value each settings field can carry. Out-of-range settings can
    // only be driven when that is bigger than the legal limit.
    localparam K_REP_MAX = (1 << K_W) - 1;
    localparam M_REP_MAX = (1 << M_W) - 1;
    localparam N_REP_MAX = (1 << N_W) - 1;
    localparam CAN_K_OVER = (K_REP_MAX > K_DEPTH);
    localparam CAN_M_OVER = (M_REP_MAX > ROWS);
    localparam CAN_N_OVER = (N_REP_MAX > COLS);

    // Directed-test job lengths, clamped so tiny K_DEPTH values still work
    localparam K_MID = (K_DEPTH > 5) ? 5 : K_DEPTH;
    localparam K_TWO = (K_DEPTH > 2) ? 2 : K_DEPTH;
    localparam K_TRI = (K_DEPTH > 3) ? 3 : K_DEPTH;

    localparam logic signed [DATA_WIDTH-1:0] JUNK_A = 8'sh55;
    localparam logic signed [DATA_WIDTH-1:0] JUNK_B = 8'shAA;

    logic clk;
    logic rst_n;

    logic                                    ld_en;
    logic [ADDR_W-1:0]                       ld_addr;
    logic signed [ROWS-1:0][DATA_WIDTH-1:0]  ld_a;
    logic signed [COLS-1:0][DATA_WIDTH-1:0]  ld_b;

    logic              job_start;
    logic [K_W-1:0]    cfg_k;
    logic [M_W-1:0]    cfg_m;
    logic [N_W-1:0]    cfg_n;
    logic              cfg_accumulate;
    logic              cfg_emit;
    logic              busy;
    logic              job_done;
    logic              job_err;

    logic                                     res_valid;
    logic [M_W-1:0]                           res_row;
    logic signed [COLS-1:0][ACC_WIDTH-1:0]    res_data;
    logic                                     res_last;

    // Flat views so loop variables can index them. c_flat is the tile INSIDE
    // the engine (white-box), res_flat is what the result stream shows.
    logic [ROWS*COLS*ACC_WIDTH-1:0] c_flat;
    logic [COLS*ACC_WIDTH-1:0]      res_flat;
    assign c_flat   = dut.c_tile;
    assign res_flat = res_data;

    tensor_core #(
        .ROWS      (ROWS),
        .COLS      (COLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH),
        .K_DEPTH   (K_DEPTH)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .ld_en         (ld_en),
        .ld_addr       (ld_addr),
        .ld_a          (ld_a),
        .ld_b          (ld_b),
        .job_start     (job_start),
        .cfg_k         (cfg_k),
        .cfg_m         (cfg_m),
        .cfg_n         (cfg_n),
        .cfg_accumulate(cfg_accumulate),
        .cfg_emit      (cfg_emit),
        .busy          (busy),
        .job_done      (job_done),
        .job_err       (job_err),
        .res_valid     (res_valid),
        .res_row       (res_row),
        .res_data      (res_data),
        .res_last      (res_last)
    );

    // Clock: 10ns period
    always #5 clk = ~clk;

    // Watchdog so a hung job can never spin forever
    initial begin
        #400_000_000;
        $display("TIMEOUT: testbench did not finish");
        $finish;
    end

    // ------------------------------------------------------------------
    // Golden-model state and counters
    // ------------------------------------------------------------------
    int A_mat [ROWS][K_DEPTH];          // what the buffers hold (slice k = column k of A)
    int B_mat [K_DEPTH][COLS];
    int A_big [ROWS][3*K_DEPTH];        // full-K operands for the chunking test
    int B_big [3*K_DEPTH][COLS];
    int exp_c [ROWS][COLS];

    int total_checks = 0;
    int total_fails  = 0;

    int last_loaded_k = 0;      // slices currently valid in the buffers
    bit prev_tight    = 1'b0;   // previous job ended in its first idle cycle
    bit err_pending   = 1'b0;   // a job_err pulse is due in the current cycle (abuse in
                                // the done cycle, next job starting right away)

    int err_seen      = 0;      // job_err pulses counted by the monitor
    int err_expected  = 0;      // job_err pulses the testbench provoked
    int beats_seen    = 0;      // result rows counted by the monitor
    int beats_expected= 0;      // result rows the testbench expects in total

    // Functional coverage (manual counters - no covergroups in this Icarus)
    int cov_k1            = 0;  // K = 1
    int cov_kmax          = 0;  // K = K_DEPTH
    int cov_extreme_ops   = 0;  // -128 / 127 operands
    int cov_neg_result    = 0;  // a PE checked with a negative value
    int cov_pos_result    = 0;  // a PE checked with a positive value
    int cov_back_to_back  = 0;  // job_start in the earliest legal cycle
    int cov_reuse         = 0;  // job re-run on buffers loaded earlier
    int cov_reload        = 0;  // buffers overwritten between two jobs
    int cov_partial_rows  = 0;  // 0 < m < ROWS
    int cov_partial_cols  = 0;  // 0 < n < COLS
    int cov_empty_tile    = 0;  // m = 0 or n = 0
    int cov_accumulate    = 0;  // job kept the previous sums
    int cov_chunked       = 0;  // 3-chunk K split checked end to end
    int cov_late_load     = 0;  // load in the job_start cycle
    int cov_err_k0        = 0;  // rejected: cfg_k = 0
    int cov_err_kover     = 0;  // rejected: cfg_k > K_DEPTH
    int cov_err_mover     = 0;  // rejected: cfg_m > ROWS
    int cov_err_nover     = 0;  // rejected: cfg_n > COLS
    int cov_abuse_launch  = 0;  // start while busy, LAUNCH cycle
    int cov_abuse_stream  = 0;  // start while busy, during STREAM
    int cov_abuse_drain   = 0;  // start while busy, during DRAIN (before done)
    int cov_abuse_done    = 0;  // start while busy, in the job_done cycle
    int cov_abuse_load    = 0;  // load while busy
    int cov_emit_off      = 0;  // job with cfg_emit = 0 (nothing streamed)
    int cov_rows_one      = 0;  // streamed tile with a single row
    int cov_rows_full     = 0;  // streamed tile with all ROWS rows
    int cov_abuse_readout = 0;  // start while busy, during readout
    int cov_abuse_ld_rd   = 0;  // load while busy, during readout
    int cov_reset_early   = 0;  // reset during LAUNCH / STREAM / DRAIN
    int cov_reset_done    = 0;  // reset in the job_done cycle
    int cov_reset_readout = 0;  // reset during readout
    int bins_missed       = 0;

    integer rng_seed;

    function int rand_range(input int lo, input int hi);
        rand_range = lo + ($unsigned($random(rng_seed)) % (hi - lo + 1));
    endfunction

    function int imin(input int a, input int b);
        imin = (a < b) ? a : b;
    endfunction

    function int imax(input int a, input int b);
        imax = (a > b) ? a : b;
    endfunction

    // ------------------------------------------------------------------
    // Cycle monitor: rules that must hold on every single cycle
    // ------------------------------------------------------------------
    logic prev_done = 1'b0;
    always @(posedge clk) begin
        if (rst_n) begin
            total_checks++;
            if (job_done && !busy) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: job_done asserted while not busy", $time);
            end
            if (job_done && prev_done) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: job_done held for more than one cycle", $time);
            end
            if (job_err) err_seen++;
            if (res_valid && !busy) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: res_valid asserted while not busy", $time);
            end
            if (res_valid && job_done) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: res_valid together with job_done", $time);
            end
            if (res_last && !res_valid) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: res_last without res_valid", $time);
            end
            if (!res_valid && res_flat !== '0) begin
                total_fails++;
                $display("FAIL [monitor] t=%0t: res_data not zero while res_valid is low", $time);
            end
            if (res_valid) beats_seen++;
        end
        prev_done <= job_done;
    end

    // ------------------------------------------------------------------
    // Host-side helpers
    // ------------------------------------------------------------------

    // Everything idle; every bus carries non-zero junk. The settings pins hold
    // junk too, because they must only matter in the job_start cycle.
    task automatic drive_idle();
        ld_en          = 1'b0;
        ld_addr        = '1;
        for (int i = 0; i < ROWS; i++) ld_a[i] = JUNK_A;
        for (int j = 0; j < COLS; j++) ld_b[j] = JUNK_B;
        job_start      = 1'b0;
        cfg_k          = '1;
        cfg_m          = '1;
        cfg_n          = '1;
        cfg_accumulate = 1'b1;
        cfg_emit       = 1'b1;
    endtask

    task automatic fill_random();
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_DEPTH; k++) A_mat[i][k] = rand_range(-128, 127);
        for (int k = 0; k < K_DEPTH; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = rand_range(-128, 127);
    endtask

    task automatic fill_const(input int a_val, input int b_val);
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_DEPTH; k++) A_mat[i][k] = a_val;
        for (int k = 0; k < K_DEPTH; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = b_val;
    endtask

    // Write slice k of A_mat / B_mat into the buffers (the bus is left driven)
    task automatic drive_slice(input int k);
        ld_en   = 1'b1;
        ld_addr = k;
        for (int i = 0; i < ROWS; i++) ld_a[i] = A_mat[i][k];
        for (int j = 0; j < COLS; j++) ld_b[j] = B_mat[k][j];
    endtask

    // Write slices 0..k_len-1 into the buffers (one per cycle)
    task automatic load_buffers(input int k_len);
        if (k_len > 0) err_pending = 1'b0;       // at least one cycle passes
        for (int k = 0; k < k_len; k++) begin
            drive_slice(k);
            @(posedge clk);
            #1;
        end
        drive_idle();
        last_loaded_k = k_len;
    endtask

    // Golden model for one job. Inactive PEs keep what they had (zero after a
    // clearing job).
    task automatic compute_golden(input int k_len, input int m_act, input int n_act,
                                  input bit accumulate_i);
        if (!accumulate_i)
            for (int i = 0; i < ROWS; i++)
                for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;
        for (int i = 0; i < m_act; i++)
            for (int j = 0; j < n_act; j++)
                for (int k = 0; k < k_len; k++)
                    exp_c[i][j] += A_mat[i][k] * B_mat[k][j];
    endtask

    // Compares every C[i][j] with the golden model
    task automatic check_result(input string label);
        logic signed [ACC_WIDTH-1:0] got;
        for (int i = 0; i < ROWS; i++) begin
            for (int j = 0; j < COLS; j++) begin
                got = c_flat[(i*COLS + j)*ACC_WIDTH +: ACC_WIDTH];
                total_checks++;
                if (got !== exp_c[i][j]) begin
                    total_fails++;
                    $display("FAIL [%0s]: C[%0d][%0d] = %0d, expected %0d",
                             label, i, j, got, exp_c[i][j]);
                end
                if (exp_c[i][j] > 0) cov_pos_result++;
                if (exp_c[i][j] < 0) cov_neg_result++;
            end
        end
    endtask

    task automatic expect_bit(input logic actual, input logic expected, input string label);
        total_checks++;
        if (actual !== expected) begin
            total_fails++;
            $display("FAIL [%0s] t=%0t: got %b, expected %b", label, $time, actual, expected);
        end
    endtask

    task automatic sample_coverage(input int k_len, input int m_act, input int n_act,
                                   input bit accumulate_i, input bit emit_i,
                                   input int abuse_start_at, input int abuse_ld_at,
                                   input bit late_load);
        bit extreme = 0;
        int done_cyc = k_len + ROWS + COLS;
        if (k_len == 1)                    cov_k1++;
        if (k_len == K_DEPTH)              cov_kmax++;
        if (prev_tight)                    cov_back_to_back++;
        if (m_act > 0 && m_act < ROWS)     cov_partial_rows++;
        if (n_act > 0 && n_act < COLS)     cov_partial_cols++;
        if (m_act == 0 || n_act == 0)      cov_empty_tile++;
        if (accumulate_i)                  cov_accumulate++;
        if (late_load)                     cov_late_load++;
        if (abuse_ld_at > 0)               cov_abuse_load++;
        if (!emit_i && m_act > 0)          cov_emit_off++;
        if (emit_i && m_act == 1)          cov_rows_one++;
        if (emit_i && m_act == ROWS)       cov_rows_full++;
        if (emit_i && m_act > 0 && abuse_start_at > done_cyc) cov_abuse_readout++;
        if (emit_i && m_act > 0 && abuse_ld_at    > done_cyc) cov_abuse_ld_rd++;
        if (abuse_start_at == 1)                                   cov_abuse_launch++;
        if (abuse_start_at >= 2 && abuse_start_at <= k_len + 1)    cov_abuse_stream++;
        if (abuse_start_at >= k_len + 2 && abuse_start_at < done_cyc) cov_abuse_drain++;
        if (abuse_start_at == done_cyc)                            cov_abuse_done++;
        for (int i = 0; i < m_act; i++)
            for (int k = 0; k < k_len; k++)
                if (A_mat[i][k] == -128 || A_mat[i][k] == 127) extreme = 1;
        for (int k = 0; k < k_len; k++)
            for (int j = 0; j < n_act; j++)
                if (B_mat[k][j] == -128 || B_mat[k][j] == 127) extreme = 1;
        if (extreme) cov_extreme_ops++;
    endtask

    // ------------------------------------------------------------------
    // Checks one row of the result stream (called in the cycle it is due)
    // ------------------------------------------------------------------
    task automatic check_stream_row(input string label, input int r, input int m_act,
                                    input int n_act);
        logic signed [ACC_WIDTH-1:0] got;
        int expv;
        expect_bit(res_valid, 1'b1, {label, ": res_valid"});
        expect_bit(res_last,  (r == m_act - 1), {label, ": res_last"});
        total_checks++;
        if (res_row !== r) begin
            total_fails++;
            $display("FAIL [%0s] t=%0t: res_row = %0d, expected %0d", label, $time, res_row, r);
        end
        for (int j = 0; j < COLS; j++) begin
            got  = res_flat[j*ACC_WIDTH +: ACC_WIDTH];
            expv = (j < n_act) ? exp_c[r][j] : 0;       // columns >= n read 0
            total_checks++;
            if (got !== expv) begin
                total_fails++;
                $display("FAIL [%0s] stream row %0d col %0d = %0d, expected %0d",
                         label, r, j, got, expv);
            end
        end
    endtask

    // ------------------------------------------------------------------
    // One complete job: start, wait for job_done, read the result stream,
    // check timing and result.
    //   m_act / n_act   : active rows / columns (partial tiles). Inactive lanes
    //                     still carry live data, to prove the engine masks them.
    //   emit_i          : cfg_emit. 0 = keep the sums inside, stream nothing
    //   gap             : idle cycles left after the cycle that follows the
    //                     last busy cycle (0 = next job_start in the earliest
    //                     legal cycle)
    //   abuse_start_at  : 0 = none, else a rejected job_start is driven in that
    //                     cycle of the job (cycle 0 = the real job_start cycle)
    //   abuse_ld_at     : 0 = none, else a junk load is driven in that cycle,
    //                     aimed at a slice that has not been read yet
    //   late_load       : the caller loaded slices 0..K-2; this task writes the
    //                     last slice in the job_start cycle itself
    // Cycle numbering: job_done is in cycle K+ROWS+COLS, result row r in cycle
    // K+ROWS+COLS+1+r, so the job is busy through cycle last_cyc.
    // ------------------------------------------------------------------
    task automatic run_job_e(input string label, input int k_len,
                             input int m_act, input int n_act, input bit accumulate_i,
                             input bit emit_i, input int gap,
                             input int abuse_start_at, input int abuse_ld_at,
                             input bit late_load);
        int cyc;
        int exp_done_cyc;
        int n_rows;
        int last_cyc;

        exp_done_cyc = k_len + ROWS + COLS;      // cycle 0 = the job_start cycle
        n_rows       = (emit_i && m_act > 0) ? m_act : 0;
        last_cyc     = exp_done_cyc + n_rows;
        compute_golden(k_len, m_act, n_act, accumulate_i);
        sample_coverage(k_len, m_act, n_act, accumulate_i, emit_i,
                        abuse_start_at, abuse_ld_at, late_load);
        prev_tight = (gap == 0);
        if (abuse_start_at > 0) err_expected++;
        beats_expected += n_rows;

        // Idle before the job
        expect_bit(busy,      1'b0, {label, ": busy before start"});
        expect_bit(job_done,  1'b0, {label, ": job_done before start"});
        expect_bit(res_valid, 1'b0, {label, ": res_valid before start"});
        expect_bit(job_err,   err_pending, {label, ": job_err before start"});
        err_pending = 1'b0;

        // --- job_start pulse (one cycle), optionally with the last load ---
        if (late_load) drive_slice(k_len - 1);
        job_start      = 1'b1;
        cfg_k          = k_len;
        cfg_m          = m_act;
        cfg_n          = n_act;
        cfg_accumulate = accumulate_i;
        cfg_emit       = emit_i;
        @(posedge clk);
        #1;
        drive_idle();                            // scramble the settings
        if (late_load) last_loaded_k = k_len;

        // --- watch the job, one cycle at a time ---
        for (cyc = 1; cyc <= last_cyc; cyc++) begin
            expect_bit(busy, 1'b1, {label, ": busy during job"});
            expect_bit(job_err, (abuse_start_at > 0 && cyc == abuse_start_at + 1),
                       {label, ": job_err"});
            expect_bit(job_done, (cyc == exp_done_cyc), {label, ": job_done timing"});
            if (cyc == exp_done_cyc) check_result(label);
            if (cyc > exp_done_cyc) check_stream_row(label, cyc - exp_done_cyc - 1, m_act, n_act);
            else                    expect_bit(res_valid, 1'b0, {label, ": res_valid too early"});

            // Attacks launched in this cycle
            if (cyc == abuse_start_at) begin
                job_start      = 1'b1;           // looks like a perfectly good job
                cfg_k          = 1;
                cfg_m          = 1;
                cfg_n          = 1;
                cfg_accumulate = 1'b0;
            end
            if (cyc == abuse_ld_at) begin
                drive_slice((abuse_ld_at <= k_len) ? k_len - 1 : 0);
                for (int i = 0; i < ROWS; i++) ld_a[i] = rand_range(-128, 127);
                for (int j = 0; j < COLS; j++) ld_b[j] = rand_range(-128, 127);
            end

            if (cyc < last_cyc) begin
                @(posedge clk);
                #1;
                drive_idle();
            end
        end

        // --- the cycle after the last busy cycle: idle again, nothing more ---
        @(posedge clk);
        #1;
        drive_idle();
        expect_bit(busy,      1'b0, {label, ": busy after job"});
        expect_bit(job_done,  1'b0, {label, ": job_done lasts one cycle"});
        expect_bit(res_valid, 1'b0, {label, ": res_valid after last row"});
        expect_bit(job_err,   (abuse_start_at == last_cyc), {label, ": job_err after job"});
        check_result({label, " (after job)"});

        repeat (gap) begin
            @(posedge clk);
            #1;
        end
        // A start rejected in the last busy cycle pulses job_err in the first
        // idle cycle - that is where we are when gap = 0
        err_pending = (abuse_start_at == last_cyc) && (gap == 0);
    endtask

    // The usual job: result stream on
    task automatic run_job(input string label, input int k_len,
                           input int m_act, input int n_act, input bit accumulate_i,
                           input int gap,
                           input int abuse_start_at, input int abuse_ld_at,
                           input bit late_load);
        run_job_e(label, k_len, m_act, n_act, accumulate_i, 1'b1, gap,
                  abuse_start_at, abuse_ld_at, late_load);
    endtask

    // Convenience: a plain full-tile job
    task automatic run_full(input string label, input int k_len, input int gap);
        run_job(label, k_len, ROWS, COLS, 1'b0, gap, 0, 0, 1'b0);
    endtask

    // ------------------------------------------------------------------
    // Reset in the middle of a job. A full-tile job is started and rst_n is
    // pulled low in cycle `cut` (cycle 0 = the job_start cycle). The reset is
    // asynchronous, so everything must be idle immediately - no clock edge
    // needed. The engine sums must be cleared, and the next job must run
    // correctly from scratch.
    // ------------------------------------------------------------------
    task automatic reset_test(input string label, input int cut);
        int cyc;
        int done_cyc;
        done_cyc = K_MID + ROWS + COLS;

        expect_bit(busy,     1'b0, {label, ": busy before start"});
        expect_bit(res_valid,1'b0, {label, ": res_valid before start"});
        expect_bit(job_err,  err_pending, {label, ": job_err before start"});
        err_pending = 1'b0;

        if (cut == done_cyc)      cov_reset_done++;
        else if (cut > done_cyc)  cov_reset_readout++;
        else                      cov_reset_early++;

        job_start      = 1'b1;
        cfg_k          = K_MID;
        cfg_m          = ROWS;
        cfg_n          = COLS;
        cfg_accumulate = 1'b0;
        cfg_emit       = 1'b1;
        @(posedge clk);
        #1;
        drive_idle();

        // Watch the job up to the cut cycle (the monitor counts the rows that
        // are sent before the reset, so the stream tally stays exact)
        for (cyc = 1; cyc <= cut; cyc++) begin
            expect_bit(busy,     1'b1, {label, ": busy before reset"});
            expect_bit(job_done, (cyc == done_cyc), {label, ": job_done before reset"});
            expect_bit(res_valid, (cyc > done_cyc), {label, ": res_valid before reset"});
            if (cyc > done_cyc && cyc < cut) beats_expected++;   // sampled at the edge after it
            if (cyc < cut) begin
                @(posedge clk);
                #1;
                drive_idle();
            end
        end

        // Pull reset (asynchronous: checked straight away, no clock edge)
        rst_n = 1'b0;
        #1;
        expect_bit(busy,      1'b0, {label, ": busy during reset"});
        expect_bit(job_done,  1'b0, {label, ": job_done during reset"});
        expect_bit(res_valid, 1'b0, {label, ": res_valid during reset"});
        expect_bit(res_last,  1'b0, {label, ": res_last during reset"});
        expect_bit(job_err,   1'b0, {label, ": job_err during reset"});
        total_checks++;
        if (res_flat !== '0) begin
            total_fails++;
            $display("FAIL [%0s]: res_data not zero during reset", label);
        end
        repeat (2) begin
            @(posedge clk);
            #1;
        end
        rst_n = 1'b1;
        @(posedge clk);
        #1;

        // Back from reset: idle, engine sums cleared
        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;
        expect_bit(busy,      1'b0, {label, ": busy after reset"});
        expect_bit(job_done,  1'b0, {label, ": job_done after reset"});
        expect_bit(res_valid, 1'b0, {label, ": res_valid after reset"});
        expect_bit(job_err,   1'b0, {label, ": job_err after reset"});
        check_result({label, " (sums cleared)"});
    endtask

    // ------------------------------------------------------------------
    // A job_start with settings the core must reject: job_err in the next
    // cycle, the core never becomes busy, results are untouched.
    // ------------------------------------------------------------------
    task automatic try_bad_job(input string label, input int k, input int m, input int n,
                               input bit accumulate_i);
        expect_bit(busy,     1'b0, {label, ": busy before bad start"});
        expect_bit(job_done, 1'b0, {label, ": job_done before bad start"});
        expect_bit(job_err,  err_pending, {label, ": job_err before bad start"});
        err_pending = 1'b0;

        if (k == 0)       cov_err_k0++;
        if (k > K_DEPTH)  cov_err_kover++;
        if (m > ROWS)     cov_err_mover++;
        if (n > COLS)     cov_err_nover++;
        err_expected++;

        job_start      = 1'b1;
        cfg_k          = k;
        cfg_m          = m;
        cfg_n          = n;
        cfg_accumulate = accumulate_i;           // 0 would wipe the tile if it started
        @(posedge clk);
        #1;
        drive_idle();

        expect_bit(job_err,  1'b1, {label, ": job_err pulse"});
        expect_bit(busy,     1'b0, {label, ": busy after bad start"});
        expect_bit(job_done, 1'b0, {label, ": job_done after bad start"});
        check_result({label, " (result untouched)"});

        // Nothing may start now or later: watch past the longest possible job
        repeat (K_DEPTH + ROWS + COLS + 3) begin
            @(posedge clk);
            #1;
            expect_bit(job_err,  1'b0, {label, ": job_err lasts one cycle"});
            expect_bit(busy,     1'b0, {label, ": core stayed idle"});
            expect_bit(job_done, 1'b0, {label, ": no job_done"});
        end
        check_result({label, " (result still untouched)"});
    endtask

    // Random rejected job: one field is pushed out of range
    task automatic random_bad_job(input string label);
        int k, m, n, pick;
        k = rand_range(1, K_DEPTH);
        m = rand_range(0, ROWS);
        n = rand_range(0, COLS);
        pick = rand_range(0, 3);
        if      (pick == 1 && CAN_K_OVER) k = rand_range(K_DEPTH + 1, K_REP_MAX);
        else if (pick == 2 && CAN_M_OVER) m = rand_range(ROWS + 1, M_REP_MAX);
        else if (pick == 3 && CAN_N_OVER) n = rand_range(COLS + 1, N_REP_MAX);
        else                              k = 0;
        try_bad_job(label, k, m, n, rand_range(0, 1));
    endtask

    // ------------------------------------------------------------------
    // K-chunking: one long product (K = 2*K_DEPTH + K_TRI) is run as three
    // jobs, the later ones with accumulate = 1, and the final tile is also
    // compared with the full product computed directly from the whole matrices.
    // ------------------------------------------------------------------
    task automatic chunk_test(input string label, input int m_act, input int n_act);
        int k_total, k_off, k_len, full;
        logic signed [ACC_WIDTH-1:0] got;

        k_total = 2 * K_DEPTH + K_TRI;
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < k_total; k++) A_big[i][k] = rand_range(-128, 127);
        for (int k = 0; k < k_total; k++)
            for (int j = 0; j < COLS; j++) B_big[k][j] = rand_range(-128, 127);

        k_off = 0;
        for (int c = 0; c < 3; c++) begin
            k_len = (c < 2) ? K_DEPTH : K_TRI;
            for (int i = 0; i < ROWS; i++)
                for (int k = 0; k < k_len; k++) A_mat[i][k] = A_big[i][k_off + k];
            for (int k = 0; k < k_len; k++)
                for (int j = 0; j < COLS; j++) B_mat[k][j] = B_big[k_off + k][j];
            load_buffers(k_len);
            run_job_e($sformatf("%0s-chunk%0d", label, c), k_len, m_act, n_act,
                      (c > 0), (c == 2), 1, 0, 0, 1'b0);
            k_off += k_len;
        end

        for (int i = 0; i < m_act; i++) begin
            for (int j = 0; j < n_act; j++) begin
                full = 0;
                for (int k = 0; k < k_total; k++) full += A_big[i][k] * B_big[k][j];
                got = c_flat[(i*COLS + j)*ACC_WIDTH +: ACC_WIDTH];
                total_checks++;
                if (got !== full) begin
                    total_fails++;
                    $display("FAIL [%0s full-K]: C[%0d][%0d] = %0d, expected %0d",
                             label, i, j, got, full);
                end
            end
        end
        cov_chunked++;
    endtask

    // ------------------------------------------------------------------
    // Coverage report
    // ------------------------------------------------------------------
    task automatic report_bin(input string name, input int count, input bit required);
        if (!required)      $display("  [n/a ] %0s", name);
        else if (count > 0) $display("  [hit ] %0s (%0d)", name, count);
        else begin
            $display("  [MISS] %0s", name);
            bins_missed++;
        end
    endtask

    task automatic report_coverage();
        $display("Functional coverage:");
        report_bin("K = 1",                       cov_k1,           1);
        report_bin("K = K_DEPTH",                 cov_kmax,         1);
        report_bin("extreme operands",            cov_extreme_ops,  1);
        report_bin("negative PE result",          cov_neg_result,   1);
        report_bin("positive PE result",          cov_pos_result,   1);
        report_bin("back-to-back job_start",      cov_back_to_back, 1);
        report_bin("job on reused buffers",       cov_reuse,        1);
        report_bin("buffers reloaded",            cov_reload,       1);
        report_bin("partial rows",                cov_partial_rows, ROWS > 1);
        report_bin("partial columns",             cov_partial_cols, COLS > 1);
        report_bin("empty tile",                  cov_empty_tile,   1);
        report_bin("accumulate (keep sums)",      cov_accumulate,   1);
        report_bin("3-chunk K split, end to end", cov_chunked,      1);
        report_bin("load in job_start cycle",     cov_late_load,    1);
        report_bin("rejected: cfg_k = 0",         cov_err_k0,       1);
        report_bin("rejected: cfg_k > K_DEPTH",   cov_err_kover,    CAN_K_OVER);
        report_bin("rejected: cfg_m > ROWS",      cov_err_mover,    CAN_M_OVER);
        report_bin("rejected: cfg_n > COLS",      cov_err_nover,    CAN_N_OVER);
        report_bin("start while busy: LAUNCH",    cov_abuse_launch, 1);
        report_bin("start while busy: STREAM",    cov_abuse_stream, 1);
        report_bin("start while busy: DRAIN",     cov_abuse_drain,  (ROWS + COLS) > 2);
        report_bin("start while busy: done cycle",cov_abuse_done,   1);
        report_bin("load while busy",             cov_abuse_load,   1);
        report_bin("cfg_emit = 0 (no stream)",    cov_emit_off,     1);
        report_bin("stream of a single row",      cov_rows_one,     1);
        report_bin("stream of all ROWS rows",     cov_rows_full,    1);
        report_bin("start while busy: readout",   cov_abuse_readout,1);
        report_bin("load while busy: readout",    cov_abuse_ld_rd,  1);
        report_bin("reset during LAUNCH/STREAM/DRAIN", cov_reset_early,   1);
        report_bin("reset in the job_done cycle",      cov_reset_done,    1);
        report_bin("reset during readout",             cov_reset_readout, 1);
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    int k_pick, m_pick, n_pick, gap_pick, roll;
    int ab_s, ab_l;
    bit acc_pick, late_pick, emit_pick;
    int len_pick;

    initial begin
        $dumpfile("tensor_core.vcd");
        $dumpvars(0, tensor_core_tb);

        clk      = 0;
        rst_n    = 0;
        rng_seed = 42;
        drive_idle();

        // Step 1: release reset; zero the buffers so no word is ever unknown
        @(posedge clk);
        #1 rst_n = 1;
        @(posedge clk);
        #1;
        expect_bit(busy,     1'b0, "post-reset busy");
        expect_bit(job_done, 1'b0, "post-reset job_done");
        expect_bit(job_err,  1'b0, "post-reset job_err");
        expect_bit(res_valid, 1'b0, "post-reset res_valid");
        fill_const(0, 0);
        load_buffers(K_DEPTH);

        // Step 2: full-tile jobs - shortest, longest, middle
        fill_random();
        load_buffers(1);
        run_full("K1", 1, 2);
        fill_random();
        load_buffers(K_DEPTH);
        run_full("K-max", K_DEPTH, 2);
        fill_random();
        load_buffers(K_MID);
        run_full("K-mid", K_MID, 2);

        // Step 3: extreme operands
        fill_const(-128, -128);
        load_buffers(K_DEPTH);
        run_full("extreme-neg*neg", K_DEPTH, 2);
        fill_const(-128, 127);
        load_buffers(K_DEPTH);
        run_full("extreme-neg*pos", K_DEPTH, 2);
        fill_const(127, 127);
        load_buffers(K_DEPTH);
        run_full("extreme-pos*pos", K_DEPTH, 2);

        // Step 4: back-to-back jobs on the same buffers (earliest legal start)
        fill_random();
        load_buffers(K_DEPTH);
        run_full("b2b-1", K_DEPTH, 0);
        cov_reuse++;
        run_full("b2b-2", K_TWO, 0);
        cov_reuse++;
        run_full("b2b-3", K_TRI, 2);

        // Step 5: partial tiles - inactive lanes carry live data
        fill_random();
        load_buffers(K_MID);
        run_job("partial-2x3",       K_MID, imin(2, ROWS), imin(3, COLS), 1'b0, 2, 0, 0, 1'b0);
        run_job("partial-1x1",       K_MID, 1,             1,             1'b0, 2, 0, 0, 1'b0);
        run_job("partial-all-x1",    K_MID, ROWS,          1,             1'b0, 2, 0, 0, 1'b0);
        run_job("partial-1-xall",    K_MID, 1,             COLS,          1'b0, 2, 0, 0, 1'b0);
        run_job("partial-rows-only", K_MID, imax(1, ROWS - 1), COLS,      1'b0, 2, 0, 0, 1'b0);
        run_job("partial-cols-only", K_MID, ROWS, imax(1, COLS - 1),      1'b0, 2, 0, 0, 1'b0);

        // Step 6: empty tiles - nothing fires; accumulate = 0 still clears
        run_full("before-empty", K_MID, 2);
        run_job("empty-keep-m0",  K_MID, 0,    COLS, 1'b1, 2, 0, 0, 1'b0);   // sums survive
        run_job("empty-clear-m0", K_MID, 0,    COLS, 1'b0, 2, 0, 0, 1'b0);   // sums cleared
        run_job("empty-clear-n0", K_MID, ROWS, 0,    1'b0, 2, 0, 0, 1'b0);

        // Step 7: accumulate - the same job again on top of itself
        run_full("accum-base", K_MID, 2);
        run_job("accum-again", K_MID, ROWS, COLS, 1'b1, 2, 0, 0, 1'b0);
        run_job("accum-again-partial", K_MID, imax(1, ROWS - 1), imax(1, COLS - 1), 1'b1, 2, 0, 0, 1'b0);

        // Step 8: K-chunking - one long product split over three jobs
        chunk_test("chunk-full", ROWS, COLS);
        chunk_test("chunk-partial", imax(1, ROWS - 1), imax(1, COLS - 1));

        // Step 9: rejected settings (after a job, so the tile is not all zero)
        fill_random();
        load_buffers(K_MID);
        run_full("before-bad", K_MID, 2);
        try_bad_job("bad-k0",        0, ROWS, COLS, 1'b0);
        if (CAN_K_OVER) try_bad_job("bad-kover", K_DEPTH + 1, ROWS, COLS, 1'b0);
        if (CAN_K_OVER) try_bad_job("bad-kmaxrep", K_REP_MAX, ROWS, COLS, 1'b0);
        if (CAN_M_OVER) try_bad_job("bad-mover", K_MID, ROWS + 1, COLS, 1'b0);
        if (CAN_N_OVER) try_bad_job("bad-nover", K_MID, ROWS, COLS + 1, 1'b0);
        run_full("after-bad", K_MID, 2);            // the core still works

        // Step 10: job_start while busy, in every phase of a job
        run_job("abuse-start-launch", K_MID, ROWS, COLS, 1'b0, 2, 1,                 0, 1'b0);
        run_job("abuse-start-stream", K_MID, ROWS, COLS, 1'b0, 2, 2,                 0, 1'b0);
        run_job("abuse-start-lastslice", K_MID, ROWS, COLS, 1'b0, 2, K_MID + 1,      0, 1'b0);
        run_job("abuse-start-drain",  K_MID, ROWS, COLS, 1'b0, 2, K_MID + 2,         0, 1'b0);
        run_job("abuse-start-done",   K_MID, ROWS, COLS, 1'b0, 2, K_MID + ROWS + COLS, 0, 1'b0);

        // Step 11: loads while busy, aimed at slices still to be read
        run_job("abuse-load-launch", K_MID, ROWS, COLS, 1'b0, 2, 0, 1,          1'b0);
        run_job("abuse-load-stream", K_MID, ROWS, COLS, 1'b0, 2, 0, 2,          1'b0);
        run_job("abuse-load-done",   K_MID, ROWS, COLS, 1'b0, 2, 0, K_MID + ROWS + COLS, 1'b0);
        run_full("buffers-intact-after-abuse", K_MID, 2);   // junk must not have landed

        // Step 12: a load in the job_start cycle is visible to that job
        fill_random();
        load_buffers(K_MID - 1);
        run_job("load-and-go", K_MID, ROWS, COLS, 1'b0, 2, 0, 0, 1'b1);
        run_full("load-and-go-reuse", K_MID, 2);

        // Step 13: result stream - emit off keeps the sums, the next emitting
        // job accumulates on top of them; abuse during readout
        fill_random();
        load_buffers(K_MID);
        run_job_e("emit-off-base",  K_MID, ROWS, COLS, 1'b0, 1'b0, 2, 0, 0, 1'b0);
        run_job_e("emit-on-after",  K_MID, ROWS, COLS, 1'b1, 1'b1, 2, 0, 0, 1'b0);
        run_job_e("emit-off-empty", K_MID, 0,    COLS, 1'b1, 1'b0, 2, 0, 0, 1'b0);
        run_job("single-row",       K_MID, 1,    COLS, 1'b0, 2, 0, 0, 1'b0);
        run_job("abuse-start-row0", K_MID, ROWS, COLS, 1'b0, 2, K_MID + ROWS + COLS + 1,    0, 1'b0);
        run_job("abuse-start-lastrow", K_MID, ROWS, COLS, 1'b0, 2, K_MID + ROWS + COLS + ROWS, 0, 1'b0);
        run_job("abuse-load-row0",  K_MID, ROWS, COLS, 1'b0, 2, 0, K_MID + ROWS + COLS + 1, 1'b0);
        run_job("abuse-both-lastrow", K_MID, ROWS, COLS, 1'b0, 0, K_MID + ROWS + COLS + ROWS,
                K_MID + ROWS + COLS + ROWS, 1'b0);
        run_full("buffers-intact-after-readout-abuse", K_MID, 2);

        // Step 14: reset in the middle of a job, in every phase. After each one
        // the same buffers are used for a normal job (they are not reset).
        reset_test("reset-launch",    1);
        run_full("after-reset-launch", K_MID, 2);
        reset_test("reset-stream",    2);
        run_full("after-reset-stream", K_MID, 2);
        reset_test("reset-lastslice", K_MID + 1);
        run_full("after-reset-lastslice", K_MID, 2);
        reset_test("reset-drain",     K_MID + 2);
        run_full("after-reset-drain", K_MID, 2);
        reset_test("reset-done",      K_MID + ROWS + COLS);
        run_full("after-reset-done", K_MID, 2);
        reset_test("reset-row0",      K_MID + ROWS + COLS + 1);
        run_full("after-reset-row0", K_MID, 0);
        reset_test("reset-lastrow",   K_MID + ROWS + COLS + ROWS);
        run_full("after-reset-lastrow", K_MID, 2);

        // Step 15: constrained-random regression - random sizes, partial
        // tiles, accumulate, rejected settings, abuse, reuse / reload
        for (int n = 0; n < 150; n++) begin
            roll = rand_range(0, 99);
            if (roll < 10) begin
                random_bad_job($sformatf("random-%0d-bad", n));
            end else begin
                gap_pick  = rand_range(0, 2);
                m_pick    = rand_range(0, ROWS);
                n_pick    = rand_range(0, COLS);
                acc_pick  = (rand_range(0, 2) == 0);
                late_pick = 1'b0;
                if (rand_range(0, 3) == 0) begin
                    k_pick = rand_range(1, last_loaded_k);
                    cov_reuse++;
                end else begin
                    k_pick = rand_range(1, K_DEPTH);
                    fill_random();
                    cov_reload++;
                    if (rand_range(0, 5) == 0) begin
                        late_pick = 1'b1;
                        load_buffers(k_pick - 1);
                    end else begin
                        load_buffers(k_pick);
                    end
                end
                emit_pick = (rand_range(0, 3) != 0);
                len_pick  = k_pick + ROWS + COLS + ((emit_pick && m_pick > 0) ? m_pick : 0);
                ab_s = (rand_range(0, 5) == 0) ? rand_range(1, len_pick) : 0;
                ab_l = (rand_range(0, 5) == 0) ? rand_range(1, len_pick) : 0;
                run_job_e($sformatf("random-%0d(K=%0d m=%0d n=%0d)", n, k_pick, m_pick, n_pick),
                          k_pick, m_pick, n_pick, acc_pick, emit_pick, gap_pick,
                          ab_s, ab_l, late_pick);
            end
        end

        // Every job_err pulse in the whole run must have been provoked
        total_checks++;
        if (err_seen !== err_expected) begin
            total_fails++;
            $display("FAIL [job_err tally]: saw %0d pulses, provoked %0d", err_seen, err_expected);
        end

        // Every streamed row must have been expected (and none missing)
        total_checks++;
        if (beats_seen !== beats_expected) begin
            total_fails++;
            $display("FAIL [stream tally]: saw %0d rows, expected %0d", beats_seen, beats_expected);
        end

        $display("==============================================");
        $display("SUMMARY: %0d / %0d checks passed (%0d failed)",
                  total_checks - total_fails, total_checks, total_fails);
        report_coverage();
        if (total_fails == 0 && bins_missed == 0) $display("RESULT: PASS");
        else $display("RESULT: FAIL (%0d failed checks, %0d missed bins)", total_fails, bins_missed);
        $display("Testbench complete.");
        $finish;
    end

endmodule