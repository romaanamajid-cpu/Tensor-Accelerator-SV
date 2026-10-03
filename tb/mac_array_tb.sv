`timescale 1ns/1ps

// mac_array_tb.sv
// Self-checking testbench for the output-stationary MAC array (M3).
//
// Golden model: for every run, exp_c[i][j] += sum_k A[i][k] * B[k][j]
// over the ACTIVE rows/columns only. The model accumulates across runs
// exactly like the hardware does, and is zeroed whenever clear_acc pulses.
//
// The array does not skew its own inputs, so drive_matmul() does it:
// element k of row i enters at cycle tk(k)+i, element k of column j at
// cycle tk(k)+j, so A[i][k] and B[k][j] meet at PE(i,j) in the same cycle.
module mac_array_tb;

    localparam ROWS       = 4;
    localparam COLS       = 4;
    localparam DATA_WIDTH = 8;
    localparam ACC_WIDTH  = 32;
    localparam K_MAX      = 8;   // longest inner dimension we drive

    // Non-zero junk driven on any lane/cycle that is NOT valid, to prove
    // the array gates on the valid bit and not on the data being zero.
    localparam logic signed [DATA_WIDTH-1:0] JUNK_A = 8'sh55;
    localparam logic signed [DATA_WIDTH-1:0] JUNK_B = 8'shAA;

    logic clk;
    logic rst_n;
    logic clear_acc;
    logic signed [ROWS-1:0][DATA_WIDTH-1:0]                 a_in;
    logic        [ROWS-1:0]                                 a_valid_in;
    logic signed [COLS-1:0][DATA_WIDTH-1:0]                 b_in;
    logic        [COLS-1:0]                                 b_valid_in;
    logic signed [ROWS-1:0][COLS-1:0][ACC_WIDTH-1:0]        c_out;
    logic        [ROWS-1:0][COLS-1:0]                       c_valid_out;

    // Flat view of c_out so we can index it with loop variables
    logic [ROWS*COLS*ACC_WIDTH-1:0] c_flat;
    assign c_flat = c_out;

    mac_array #(
        .ROWS      (ROWS),
        .COLS      (COLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .clear_acc  (clear_acc),
        .a_in       (a_in),
        .a_valid_in (a_valid_in),
        .b_in       (b_in),
        .b_valid_in (b_valid_in),
        .c_out      (c_out),
        .c_valid_out(c_valid_out)
    );

    // Clock: 10ns period
    always #5 clk = ~clk;

    // Watchdog so a hung run can never spin forever
    initial begin
        #20_000_000;
        $display("TIMEOUT: testbench did not finish");
        $finish;
    end

    // ------------------------------------------------------------------
    // Golden-model state
    // ------------------------------------------------------------------
    int A_mat [ROWS][K_MAX];
    int B_mat [K_MAX][COLS];
    int exp_c [ROWS][COLS];

    int total_checks = 0;
    int total_fails  = 0;

    // Functional coverage (manual counters, same reason as M2)
    int cov_k1             = 0;  // K = 1 (single outer product)
    int cov_kmax           = 0;  // K = K_MAX
    int cov_stall          = 0;  // bubble inserted mid-stream
    int cov_partial_tile   = 0;  // fewer than ROWS x COLS lanes active
    int cov_accum_no_clear = 0;  // run accumulated on top of previous result
    int cov_extreme_ops    = 0;  // run used -128 or 127 operands
    int cov_neg_result     = 0;  // a PE ended with a negative value
    int cov_pos_result     = 0;  // a PE ended with a positive value

    integer rng_seed;

    function int rand_range(input int lo, input int hi);
        rand_range = lo + ($unsigned($random(rng_seed)) % (hi - lo + 1));
    endfunction

    // ------------------------------------------------------------------
    // Stream scheduling
    // ------------------------------------------------------------------
    // Which k-element is on the wire in logical time slot s?  Returns -1 for
    // an idle slot. stall_len idle slots are inserted before element
    // stall_at (stall_at < 0 means no stall).
    function int slot_to_k(input int s, input int k_len,
                           input int stall_at, input int stall_len);
        int k;
        k = s;
        if (s < 0) k = -1;
        else if (stall_at >= 0 && s >= stall_at) begin
            if (s < stall_at + stall_len) k = -1;
            else                          k = s - stall_len;
        end
        if (k >= k_len) k = -1;
        slot_to_k = k;
    endfunction

    // ------------------------------------------------------------------
    // Tasks
    // ------------------------------------------------------------------

    // Cleanly pulses clear_acc for one cycle. Same #1 settle as M2's
    // pulse_clear, to avoid racing the DUT's always_ff on the clock edge.
    task automatic pulse_clear();
        clear_acc = 1;
        @(posedge clk);
        #1;
        clear_acc = 0;
        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++)
                exp_c[i][j] = 0;
    endtask

    // Drives one k_len-deep matrix product (A_mat x B_mat) through the array
    // with the systolic skew, then updates the golden model.
    //   m_act / n_act : number of active rows / columns (partial tiles).
    //                   Inactive lanes get valid=0 and junk data.
    //   stall_at/len  : optional mid-stream bubble, applied to all lanes.
    task automatic drive_matmul(input int k_len, input int m_act, input int n_act,
                                input int stall_at, input int stall_len);
        int t_last, k;
        // last element enters its lane at tk(k_len-1), then needs
        // (ROWS-1)+(COLS-1) more cycles to reach the far corner PE
        t_last = (k_len - 1) + ((stall_at >= 0 && stall_at <= k_len - 1) ? stall_len : 0)
               + (ROWS - 1) + (COLS - 1);

        for (int t = 0; t <= t_last; t++) begin
            for (int i = 0; i < ROWS; i++) begin
                k = (i < m_act) ? slot_to_k(t - i, k_len, stall_at, stall_len) : -1;
                if (k >= 0) begin a_in[i] = A_mat[i][k]; a_valid_in[i] = 1'b1; end
                else        begin a_in[i] = JUNK_A;      a_valid_in[i] = 1'b0; end
            end
            for (int j = 0; j < COLS; j++) begin
                k = (j < n_act) ? slot_to_k(t - j, k_len, stall_at, stall_len) : -1;
                if (k >= 0) begin b_in[j] = B_mat[k][j]; b_valid_in[j] = 1'b1; end
                else        begin b_in[j] = JUNK_B;      b_valid_in[j] = 1'b0; end
            end
            @(posedge clk);
            #1; // settle after the clock edge before changing inputs
        end

        // Quiet the inputs
        a_in = '0; b_in = '0; a_valid_in = '0; b_valid_in = '0;

        // Golden model: accumulate this product on active PEs only
        for (int i = 0; i < m_act; i++)
            for (int j = 0; j < n_act; j++)
                for (int kk = 0; kk < k_len; kk++)
                    exp_c[i][j] += A_mat[i][kk] * B_mat[kk][j];
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
                                   input int stall_at);
        bit extreme = 0;
        if (k_len == 1)     cov_k1++;
        if (k_len == K_MAX) cov_kmax++;
        if (stall_at >= 0)  cov_stall++;
        if (m_act < ROWS || n_act < COLS) cov_partial_tile++;
        for (int i = 0; i < m_act; i++)
            for (int k = 0; k < k_len; k++)
                if (A_mat[i][k] == -128 || A_mat[i][k] == 127) extreme = 1;
        for (int k = 0; k < k_len; k++)
            for (int j = 0; j < n_act; j++)
                if (B_mat[k][j] == -128 || B_mat[k][j] == 127) extreme = 1;
        if (extreme) cov_extreme_ops++;
    endtask

    // One complete test: optional clear, drive, check
    task automatic run_matmul(input string label, input int k_len,
                              input int m_act, input int n_act,
                              input int stall_at, input int stall_len,
                              input bit clear_first, input bit verbose);
        if (clear_first) pulse_clear();
        else             cov_accum_no_clear++;
        sample_coverage(k_len, m_act, n_act, stall_at);
        drive_matmul(k_len, m_act, n_act, stall_at, stall_len);
        check_all(label, verbose);
    endtask

    // ------------------------------------------------------------------
    // Matrix fillers for directed tests
    // ------------------------------------------------------------------
    task automatic fill_const(input int a_val, input int b_val);
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++) A_mat[i][k] = a_val;
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = b_val;
    endtask

    // Deterministic, asymmetric, mixed-sign pattern (so a transposed or
    // mis-wired array can't accidentally match)
    task automatic fill_pattern(input int salt);
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++)
                A_mat[i][k] = ((i*5 + k*3 + salt) % 17) - 8;
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++)
                B_mat[k][j] = ((k*7 + j*2 + salt*3) % 15) - 7;
    endtask

    task automatic fill_random();
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++) A_mat[i][k] = rand_range(-128, 127);
        for (int k = 0; k < K_MAX; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = rand_range(-128, 127);
    endtask

    // Constrained-random regression: random K, random partial tiles,
    // random mid-stream stalls, random "clear first or keep accumulating".
    // Fixed seed -> a failure at run N reproduces at run N every time.
    task automatic random_regression(input int num_runs, input int init_seed);
        int k_len, m_act, n_act, stall_at, stall_len;
        bit clear_first;
        rng_seed = init_seed;
        for (int r = 0; r < num_runs; r++) begin
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
            clear_first = (rand_range(0, 3) != 0);   // ~75% clear first
            fill_random();
            run_matmul($sformatf("random-%0d", r), k_len, m_act, n_act,
                       stall_at, stall_len, clear_first, 1'b0);
        end
    endtask

    task automatic report_coverage();
        $display("==============================================");
        $display("COVERAGE:");
        $display("  K = 1                  : %0d hits %s", cov_k1,             cov_k1             > 0 ? "[HIT]" : "[MISS]");
        $display("  K = K_MAX              : %0d hits %s", cov_kmax,           cov_kmax           > 0 ? "[HIT]" : "[MISS]");
        $display("  mid-stream stall       : %0d hits %s", cov_stall,          cov_stall          > 0 ? "[HIT]" : "[MISS]");
        $display("  partial tile           : %0d hits %s", cov_partial_tile,   cov_partial_tile   > 0 ? "[HIT]" : "[MISS]");
        $display("  accumulate w/o clear   : %0d hits %s", cov_accum_no_clear, cov_accum_no_clear > 0 ? "[HIT]" : "[MISS]");
        $display("  extreme operands       : %0d hits %s", cov_extreme_ops,    cov_extreme_ops    > 0 ? "[HIT]" : "[MISS]");
        $display("  negative PE result     : %0d hits %s", cov_neg_result,     cov_neg_result     > 0 ? "[HIT]" : "[MISS]");
        $display("  positive PE result     : %0d hits %s", cov_pos_result,     cov_pos_result     > 0 ? "[HIT]" : "[MISS]");
        $display("==============================================");
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    initial begin
        $dumpfile("mac_array.vcd");
        $dumpvars(0, mac_array_tb);

        clk = 0;
        rst_n = 0;
        clear_acc = 0;
        a_in = '0; b_in = '0; a_valid_in = '0; b_valid_in = '0;
        rng_seed = 42;
        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;

        // Step 1: release reset, every accumulator must be zero
        @(posedge clk);
        #1 rst_n = 1;
        @(posedge clk);
        #1;
        check_all("post-reset", 1);

        // Step 2: identity x B must return B
        fill_pattern(1);
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_MAX; k++) A_mat[i][k] = (i == k) ? 1 : 0;
        run_matmul("identity", ROWS, ROWS, COLS, -1, 0, 1, 1);

        // Step 3: mixed-sign asymmetric pattern, full inner dimension
        fill_pattern(2);
        run_matmul("pattern-K8", K_MAX, ROWS, COLS, -1, 0, 1, 1);

        // Step 4: extreme operands
        fill_const(-128, -128);
        run_matmul("extreme-neg*neg", K_MAX, ROWS, COLS, -1, 0, 1, 1);
        fill_const(-128, 127);
        run_matmul("extreme-neg*pos", K_MAX, ROWS, COLS, -1, 0, 1, 1);

        // Step 5: K = 1 (single outer product)
        fill_pattern(3);
        run_matmul("K1-outer-product", 1, ROWS, COLS, -1, 0, 1, 1);

        // Step 6: bubbles in the stream (all lanes idle together)
        fill_pattern(4);
        run_matmul("stall-mid",     6, ROWS, COLS, 3, 2, 1, 1);
        run_matmul("stall-leading", 5, ROWS, COLS, 0, 3, 1, 1);

        // Step 7: partial tiles - inactive lanes must not accumulate
        fill_pattern(5);
        run_matmul("partial-2x3", 5, 2, 3, -1, 0, 1, 1);
        run_matmul("partial-1x1", 4, 1, 1, -1, 0, 1, 1);
        run_matmul("partial-4x1", 4, ROWS, 1, -1, 0, 1, 1);
        run_matmul("partial-1x4", 4, 1, COLS, -1, 0, 1, 1);

        // Step 8: accumulate two products without clearing (K-chunking)
        fill_pattern(6);
        run_matmul("accum-part1", 4, ROWS, COLS, -1, 0, 1, 1);
        fill_pattern(7);
        run_matmul("accum-part2", 4, ROWS, COLS, -1, 0, 0, 1);

        // Step 9: clear returns every PE to zero
        pulse_clear();
        check_all("clear-all", 1);

        // Step 10: constrained-random regression
        random_regression(200, 42);

        $display("==============================================");
        $display("SUMMARY: %0d / %0d checks passed (%0d failed)",
                  total_checks - total_fails, total_checks, total_fails);
        report_coverage();
        $display("Testbench complete.");
        $finish;
    end

endmodule