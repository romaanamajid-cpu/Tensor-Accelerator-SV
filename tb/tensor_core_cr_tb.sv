`timescale 1ns/1ps

// tensor_core_cr_tb.sv
// Constrained-random, coverage-driven testbench for the tensor core (M7).
//
// The directed testbench (tensor_core_tb) checks hand-picked situations. This
// one lets the computer pick the situations, and keeps picking until a table of
// manual coverage bins is full, instead of running a fixed number of jobs.
//
// How it works
//   * Every iteration draws a PROFILE from a weighted table:
//       NORMAL  - random K, tile size, accumulate, emit, operands, idle gaps
//       CORNER  - K, m and n taken from the edges (1, 2, max-1, max, 0)
//       WRAP    - accumulators pre-loaded next to the wrap point, so a short
//                 job pushes sums over it (works at any ACC_WIDTH, also 32)
//       ABUSE   - job_start / load pulses while the core is busy
//       BAD     - a job_start with illegal settings (must give job_err)
//       RESET   - reset pulled at a random cycle of a job
//       CHUNK   - a K-split over 3 jobs (accumulate, emit only on the last)
//   * Weights adapt: a profile whose bins are still empty gets extra weight.
//   * The run stops when every possible bin has at least GOAL hits (and at
//     least MIN_JOBS iterations ran), or fails if MAX_JOBS is reached first.
//   * Everything is checked against a golden model on every job, exactly like
//     tensor_core_tb: job_done latency, busy, row-by-row result stream, every
//     PE in the engine (white-box), job_err tally, stream tally, monitor.
//     The golden model wraps sums to ACC_WIDTH bits and notes every wrap.
//
// Accumulator seeding (wrap tests) is white-box: values are written straight
// into the PE accumulators through the hierarchy while the core is idle.
//
// Reproducible: the seed comes from the command line
//   vvp sim/x.vvp +seed=123 [+goal=3] [+min_jobs=150] [+max_jobs=4000]
// and a failing run prints the seed to repeat it. Default seed is 42.
//
// Sizes are module parameters, e.g.
//   iverilog -Ptensor_core_cr_tb.ROWS=3 -Ptensor_core_cr_tb.COLS=5 ...
module tensor_core_cr_tb;

    parameter ROWS       = 4;
    parameter COLS       = 4;
    parameter DATA_WIDTH = 8;
    parameter ACC_WIDTH  = 32;
    parameter K_DEPTH    = 16;

    // Operand range follows DATA_WIDTH (signed)
    localparam int OP_MIN = -(1 << (DATA_WIDTH - 1));
    localparam int OP_MAX =  (1 << (DATA_WIDTH - 1)) - 1;

    localparam ADDR_W = (K_DEPTH > 1) ? $clog2(K_DEPTH) : 1;
    localparam K_W    = $clog2(K_DEPTH + 1);
    localparam M_W    = $clog2(ROWS + 1);
    localparam N_W    = $clog2(COLS + 1);

    // Largest value each settings field can carry; illegal settings can only
    // be driven when that is bigger than the legal limit.
    localparam K_REP_MAX = (1 << K_W) - 1;
    localparam M_REP_MAX = (1 << M_W) - 1;
    localparam N_REP_MAX = (1 << N_W) - 1;
    localparam CAN_K_OVER = (K_REP_MAX > K_DEPTH);
    localparam CAN_M_OVER = (M_REP_MAX > ROWS);
    localparam CAN_N_OVER = (N_REP_MAX > COLS);

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
        #800_000_000;
        $display("TIMEOUT: testbench did not finish");
        $finish;
    end

    // ------------------------------------------------------------------
    // Golden-model state, counters and random numbers
    // ------------------------------------------------------------------
    int A_mat [ROWS][K_DEPTH];          // what the buffers hold (slice k = column k of A)
    int B_mat [K_DEPTH][COLS];
    longint exp_c [ROWS][COLS];

    int total_checks = 0;
    int total_fails  = 0;

    int last_loaded_k = 0;      // slices currently valid in the buffers
    bit prev_tight    = 1'b0;   // previous job ended in its first idle cycle
    bit prev_streamed = 1'b0;   // previous job streamed result rows
    bit err_pending   = 1'b0;   // a job_err pulse is due in the current cycle

    int err_seen       = 0;     // job_err pulses counted by the monitor
    int err_expected   = 0;     // job_err pulses the testbench provoked
    int beats_seen     = 0;     // result rows counted by the monitor
    int beats_expected = 0;     // result rows the testbench expects in total

    bit job_wrap_pos = 1'b0;    // the last golden-model job wrapped past +limit
    bit job_wrap_neg = 1'b0;    // ... past -limit

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

    function int clampi(input int v, input int lo, input int hi);
        clampi = imax(lo, imin(hi, v));
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

    // Folds any integer into the operand range
    function int fit_op(input int v);
        longint r, t;
        r = longint'(OP_MAX) - OP_MIN + 1;
        t = (longint'(v) - OP_MIN) % r;
        if (t < 0) t += r;
        fit_op = int'(t + OP_MIN);
    endfunction

    // ------------------------------------------------------------------
    // Coverage bins (manual: this Icarus has no covergroups)
    //   0..14  K class x tile class       25..26 back-to-back start
    //   15..18 accumulate x emit          27..33 accumulator wrap
    //   19..24 operand profile            34..40 abuse while busy
    //   41..44 rejected settings          45..48 reset phase
    //   49     3-chunk K split
    // ------------------------------------------------------------------
    localparam int NBINS   = 50;
    localparam int B_KT    = 0;     // + kclass*5 + tclass
    localparam int B_AE    = 15;    // + acc*2 + emit
    localparam int B_OP    = 19;    // + operand profile 0..5
    localparam int B_GAP   = 25;    // + 0: after a streamed job, 1: after a silent one
    localparam int B_W_POS = 27;
    localparam int B_W_NEG = 28;
    localparam int B_W_NO  = 29;    // seeded job that did not wrap
    localparam int B_W_PAR = 30;    // wrap inside a partial tile
    localparam int B_W_SIL = 31;    // wrap in a job with cfg_emit = 0
    localparam int B_W_K1  = 32;    // wrap in a K = 1 job
    localparam int B_W_CHK = 33;    // wrap in the 2nd/3rd chunk of a K split
    localparam int B_ABS   = 34;    // + 0 launch, 1 stream, 2 drain, 3 done, 4 readout
    localparam int B_ABL   = 39;    // + 0 before done, 1 during readout
    localparam int B_BAD   = 41;    // + 0 k=0, 1 k>K_DEPTH, 2 m>ROWS, 3 n>COLS
    localparam int B_RST   = 45;    // + 0 launch/stream, 1 drain, 2 done, 3 readout
    localparam int B_CHUNK = 49;

    string bin_name [NBINS];
    int    bin_hits [NBINS];
    bit    bin_possible [NBINS];

    function string kclass_name(input int c);
        case (c)
            0: kclass_name = "K=1";
            1: kclass_name = "1<K<K_DEPTH";
            default: kclass_name = "K=K_DEPTH";
        endcase
    endfunction

    function string tclass_name(input int c);
        case (c)
            0: tclass_name = "full";
            1: tclass_name = "part rows";
            2: tclass_name = "part cols";
            3: tclass_name = "part both";
            default: tclass_name = "empty";
        endcase
    endfunction

    function string op_name(input int c);
        case (c)
            0: op_name = "random";
            1: op_name = "min*min";
            2: op_name = "min*max";
            3: op_name = "max*max";
            4: op_name = "0/+-1 sparse";
            default: op_name = "limit-biased mix";
        endcase
    endfunction

    task automatic define_bins();
        bit wrap_ok;
        wrap_ok = (ACC_WIDTH < 64);
        for (int i = 0; i < NBINS; i++) begin
            bin_name[i]     = "";
            bin_hits[i]     = 0;
            bin_possible[i] = 1'b1;
        end
        for (int kc = 0; kc < 3; kc++) begin
            for (int tc = 0; tc < 5; tc++) begin
                bin_name[B_KT + kc*5 + tc] = $sformatf("%0s, tile %0s", kclass_name(kc), tclass_name(tc));
                bin_possible[B_KT + kc*5 + tc] =
                    ((kc != 1 || K_DEPTH >= 3) && (kc != 2 || K_DEPTH >= 2)) &&
                    ((tc != 1 || ROWS > 1) && (tc != 2 || COLS > 1) &&
                     (tc != 3 || (ROWS > 1 && COLS > 1)));
            end
        end
        for (int a = 0; a < 2; a++)
            for (int e = 0; e < 2; e++)
                bin_name[B_AE + a*2 + e] = $sformatf("accumulate=%0d emit=%0d (m,n > 0)", a, e);
        for (int p = 0; p < 6; p++) bin_name[B_OP + p] = $sformatf("operands: %0s", op_name(p));
        bin_name[B_GAP + 0] = "back-to-back start after a streamed job";
        bin_name[B_GAP + 1] = "back-to-back start after a silent job";
        bin_name[B_W_POS] = "wrap past +limit";
        bin_name[B_W_NEG] = "wrap past -limit";
        bin_name[B_W_NO]  = "seeded job without wrap";
        bin_name[B_W_PAR] = "wrap inside a partial tile";
        bin_name[B_W_SIL] = "wrap in a job with emit = 0";
        bin_name[B_W_K1]  = "wrap in a K = 1 job";
        bin_name[B_W_CHK] = "wrap in a later K-split chunk";
        bin_possible[B_W_POS] = wrap_ok;
        bin_possible[B_W_NEG] = wrap_ok;
        bin_possible[B_W_PAR] = wrap_ok && (ROWS > 1 || COLS > 1);
        bin_possible[B_W_SIL] = wrap_ok;
        bin_possible[B_W_K1]  = wrap_ok;
        bin_possible[B_W_CHK] = wrap_ok;
        bin_name[B_ABS + 0] = "start while busy: LAUNCH";
        bin_name[B_ABS + 1] = "start while busy: STREAM";
        bin_name[B_ABS + 2] = "start while busy: DRAIN";
        bin_name[B_ABS + 3] = "start while busy: done cycle";
        bin_name[B_ABS + 4] = "start while busy: readout";
        bin_possible[B_ABS + 2] = ((ROWS + COLS) > 2);
        bin_name[B_ABL + 0] = "load while busy: before done";
        bin_name[B_ABL + 1] = "load while busy: readout";
        bin_name[B_BAD + 0] = "rejected: cfg_k = 0";
        bin_name[B_BAD + 1] = "rejected: cfg_k > K_DEPTH";
        bin_name[B_BAD + 2] = "rejected: cfg_m > ROWS";
        bin_name[B_BAD + 3] = "rejected: cfg_n > COLS";
        bin_possible[B_BAD + 1] = CAN_K_OVER;
        bin_possible[B_BAD + 2] = CAN_M_OVER;
        bin_possible[B_BAD + 3] = CAN_N_OVER;
        bin_name[B_RST + 0] = "reset during LAUNCH/STREAM";
        bin_name[B_RST + 1] = "reset during DRAIN";
        bin_name[B_RST + 2] = "reset in the done cycle";
        bin_name[B_RST + 3] = "reset during readout";
        bin_possible[B_RST + 1] = ((ROWS + COLS) > 2);
        bin_name[B_CHUNK] = "3-chunk K split";
    endtask

    task automatic hit(input int idx);
        bin_hits[idx]++;
    endtask

    function bit all_hit(input int goal);
        all_hit = 1'b1;
        for (int i = 0; i < NBINS; i++)
            if (bin_possible[i] && bin_hits[i] < goal) all_hit = 1'b0;
    endfunction

    function bit range_missing(input int lo, input int hi, input int goal);
        range_missing = 1'b0;
        for (int i = lo; i <= hi; i++)
            if (bin_possible[i] && bin_hits[i] < goal) range_missing = 1'b1;
    endfunction

    function int kclass_of(input int k);
        if (k == 1)            kclass_of = 0;
        else if (k == K_DEPTH) kclass_of = 2;
        else                   kclass_of = 1;
    endfunction

    function int tclass_of(input int m, input int n);
        if (m == 0 || n == 0)          tclass_of = 4;
        else if (m < ROWS && n < COLS) tclass_of = 3;
        else if (m < ROWS)             tclass_of = 1;
        else if (n < COLS)             tclass_of = 2;
        else                           tclass_of = 0;
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

    // Everything idle; every bus carries non-zero junk, also the settings pins
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

    // Operand generators. Classes: 0 random, 1 OP_MIN, 2 OP_MAX,
    // 3 sparse (-1, 0, 1), 4 limit-biased mix (limits, -1, 0, 1, random)
    function int gen_op(input int cls);
        case (cls)
            0: gen_op = rand_range(OP_MIN, OP_MAX);
            1: gen_op = OP_MIN;
            2: gen_op = OP_MAX;
            3: gen_op = fit_op(rand_range(-1, 1));
            default: begin
                case (rand_range(0, 7))
                    0:       gen_op = OP_MIN;
                    1:       gen_op = OP_MAX;
                    2:       gen_op = -1;
                    3:       gen_op = 0;
                    4:       gen_op = 1;
                    default: gen_op = rand_range(OP_MIN, OP_MAX);
                endcase
            end
        endcase
    endfunction

    // Operand profile p (0..5): which classes feed A and B
    task automatic fill_profile(input int p);
        int ca, cb;
        case (p)
            0: begin ca = 0; cb = 0; end
            1: begin ca = 1; cb = 1; end     // min*min: the largest product
            2: begin ca = 1; cb = 2; end     // min*max: the most negative product
            3: begin ca = 2; cb = 2; end     // max*max
            4: begin ca = 3; cb = 3; end
            default: begin ca = 4; cb = 4; end
        endcase
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_DEPTH; k++) A_mat[i][k] = gen_op(ca);
        for (int k = 0; k < K_DEPTH; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = gen_op(cb);
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

    // Golden model: add one product to PE (i,j), wrap, note every real wrap
    task automatic acc_add(input int i, input int j, input longint prod);
        longint raw, w;
        raw = exp_c[i][j] + prod;
        w   = wrap_acc(raw);
        if (ACC_WIDTH < 64 && w != raw) begin
            if (raw > 0) job_wrap_pos = 1'b1;
            else         job_wrap_neg = 1'b1;
        end
        exp_c[i][j] = w;
    endtask

    task automatic compute_golden(input int k_len, input int m_act, input int n_act,
                                  input bit accumulate_i);
        job_wrap_pos = 1'b0;
        job_wrap_neg = 1'b0;
        if (!accumulate_i)
            for (int i = 0; i < ROWS; i++)
                for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;
        for (int i = 0; i < m_act; i++)
            for (int j = 0; j < n_act; j++)
                for (int k = 0; k < k_len; k++)
                    acc_add(i, j, longint'(A_mat[i][k]) * B_mat[k][j]);
    endtask

    // Compares every C[i][j] inside the engine with the golden model
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

    // ------------------------------------------------------------------
    // White-box accumulator seeding (accumulator wrap-around tests)
    // The generate block gives every PE its own always block; ->seed_now makes
    // them all write seed_flat into the PE accumulators. Only call it while
    // the core is idle, a short time after a clock edge.
    // ------------------------------------------------------------------
    logic [ROWS*COLS*ACC_WIDTH-1:0] seed_flat;
    event seed_now;

    generate
        for (genvar gi = 0; gi < ROWS; gi++) begin : g_seed_row
            for (genvar gj = 0; gj < COLS; gj++) begin : g_seed_col
                always @(seed_now)
                    dut.u_engine.u_array.g_row[gi].g_col[gj].u_pe.u_mac.acc_out =
                        seed_flat[(gi*COLS + gj)*ACC_WIDTH +: ACC_WIDTH];
            end
        end
    endgenerate

    // mode 0: just below the positive limit, spread so some PEs wrap in a short job
    // mode 1: just above the negative limit, same idea
    // mode 2: random mix - near either limit, near zero, or anywhere
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
                    default: v = {$random(rng_seed), $random(rng_seed)};
                endcase
                exp_c[i][j] = wrap_acc(v);
                seed_flat[(i*COLS + j)*ACC_WIDTH +: ACC_WIDTH] = exp_c[i][j];
            end
        end
        -> seed_now;
        #1;
        check_result({label, "-seeded"});     // proves the deposit landed
    endtask

    // ------------------------------------------------------------------
    // Coverage sampling for one job (abuse phases, back-to-back)
    // ------------------------------------------------------------------
    task automatic sample_job(input int k_len, input int m_act, input bit emit_i,
                              input int abuse_start_at, input int abuse_ld_at);
        int done_cyc, rows, last_cyc;
        done_cyc = k_len + ROWS + COLS;
        rows     = (emit_i && m_act > 0) ? m_act : 0;
        last_cyc = done_cyc + rows;
        if (prev_tight) hit(B_GAP + (prev_streamed ? 0 : 1));
        if (abuse_start_at == 1)                                      hit(B_ABS + 0);
        if (abuse_start_at >= 2 && abuse_start_at <= k_len + 1)       hit(B_ABS + 1);
        if (abuse_start_at >= k_len + 2 && abuse_start_at < done_cyc) hit(B_ABS + 2);
        if (abuse_start_at == done_cyc)                               hit(B_ABS + 3);
        if (abuse_start_at > done_cyc && abuse_start_at <= last_cyc)  hit(B_ABS + 4);
        if (abuse_ld_at >= 1 && abuse_ld_at <= done_cyc)              hit(B_ABL + 0);
        if (abuse_ld_at > done_cyc && abuse_ld_at <= last_cyc)        hit(B_ABL + 1);
    endtask

    // ------------------------------------------------------------------
    // Checks one row of the result stream (called in the cycle it is due)
    // ------------------------------------------------------------------
    task automatic check_stream_row(input string label, input int r, input int m_act,
                                    input int n_act);
        logic signed [ACC_WIDTH-1:0] got;
        longint expv;
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
    //   gap            : idle cycles left after the cycle that follows the last
    //                    busy cycle (0 = next job_start in the earliest legal cycle)
    //   abuse_start_at : 0 = none, else a rejected job_start is driven in that
    //                    cycle of the job (cycle 0 = the real job_start cycle)
    //   abuse_ld_at    : 0 = none, else a junk load is driven in that cycle
    // Cycle numbering: job_done is in cycle K+ROWS+COLS, result row r in cycle
    // K+ROWS+COLS+1+r, so the job is busy through cycle last_cyc.
    // ------------------------------------------------------------------
    task automatic run_job_e(input string label, input int k_len,
                             input int m_act, input int n_act, input bit accumulate_i,
                             input bit emit_i, input int gap,
                             input int abuse_start_at, input int abuse_ld_at);
        int cyc;
        int exp_done_cyc;
        int n_rows;
        int last_cyc;

        exp_done_cyc = k_len + ROWS + COLS;      // cycle 0 = the job_start cycle
        n_rows       = (emit_i && m_act > 0) ? m_act : 0;
        last_cyc     = exp_done_cyc + n_rows;
        compute_golden(k_len, m_act, n_act, accumulate_i);
        sample_job(k_len, m_act, emit_i, abuse_start_at, abuse_ld_at);
        prev_tight    = (gap == 0);
        prev_streamed = (n_rows > 0);
        if (abuse_start_at > 0) err_expected++;
        beats_expected += n_rows;

        // Idle before the job
        expect_bit(busy,      1'b0, {label, ": busy before start"});
        expect_bit(job_done,  1'b0, {label, ": job_done before start"});
        expect_bit(res_valid, 1'b0, {label, ": res_valid before start"});
        expect_bit(job_err,   err_pending, {label, ": job_err before start"});
        err_pending = 1'b0;

        // --- job_start pulse (one cycle) ---
        job_start      = 1'b1;
        cfg_k          = k_len;
        cfg_m          = m_act;
        cfg_n          = n_act;
        cfg_accumulate = accumulate_i;
        cfg_emit       = emit_i;
        @(posedge clk);
        #1;
        drive_idle();                            // scramble the settings

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
                for (int i = 0; i < ROWS; i++) ld_a[i] = rand_range(OP_MIN, OP_MAX);
                for (int j = 0; j < COLS; j++) ld_b[j] = rand_range(OP_MIN, OP_MAX);
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

    // ------------------------------------------------------------------
    // Reset in the middle of a job (asynchronous, active low): pulled in cycle
    // `cut` (cycle 0 = the job_start cycle). Everything must be idle at once,
    // the engine sums cleared, and the next job must run from scratch.
    // ------------------------------------------------------------------
    task automatic reset_test(input string label, input int k_len, input int m_act,
                              input int n_act, input bit accumulate_i, input bit emit_i,
                              input int cut);
        int cyc;
        int done_cyc;
        done_cyc = k_len + ROWS + COLS;

        expect_bit(busy,     1'b0, {label, ": busy before start"});
        expect_bit(res_valid,1'b0, {label, ": res_valid before start"});
        expect_bit(job_err,  err_pending, {label, ": job_err before start"});
        err_pending = 1'b0;

        if (cut == done_cyc)     hit(B_RST + 2);
        else if (cut > done_cyc) hit(B_RST + 3);
        else if (cut <= k_len + 1) hit(B_RST + 0);
        else                     hit(B_RST + 1);

        job_start      = 1'b1;
        cfg_k          = k_len;
        cfg_m          = m_act;
        cfg_n          = n_act;
        cfg_accumulate = accumulate_i;
        cfg_emit       = emit_i;
        @(posedge clk);
        #1;
        drive_idle();

        // Watch the job up to the cut cycle (the monitor counts the rows that
        // are sent before the reset, so the stream tally stays exact)
        for (cyc = 1; cyc <= cut; cyc++) begin
            expect_bit(busy,     1'b1, {label, ": busy before reset"});
            expect_bit(job_done, (cyc == done_cyc), {label, ": job_done before reset"});
            expect_bit(res_valid, (cyc > done_cyc), {label, ": res_valid before reset"});
            if (cyc > done_cyc && cyc < cut) beats_expected++;
            if (cyc < cut) begin
                @(posedge clk);
                #1;
                drive_idle();
            end
        end

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

        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;
        expect_bit(busy,      1'b0, {label, ": busy after reset"});
        expect_bit(job_done,  1'b0, {label, ": job_done after reset"});
        expect_bit(res_valid, 1'b0, {label, ": res_valid after reset"});
        expect_bit(job_err,   1'b0, {label, ": job_err after reset"});
        check_result({label, " (sums cleared)"});
        prev_tight = 1'b0;
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

        if (k == 0)       hit(B_BAD + 0);
        if (k > K_DEPTH)  hit(B_BAD + 1);
        if (m > ROWS)     hit(B_BAD + 2);
        if (n > COLS)     hit(B_BAD + 3);
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

        repeat (K_DEPTH + ROWS + COLS + 3) begin
            @(posedge clk);
            #1;
            expect_bit(job_err,  1'b0, {label, ": job_err lasts one cycle"});
            expect_bit(busy,     1'b0, {label, ": core stayed idle"});
            expect_bit(job_done, 1'b0, {label, ": no job_done"});
        end
        check_result({label, " (result still untouched)"});
        prev_tight = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // Profiles. Each one picks its own random settings, then drives the core.
    // ------------------------------------------------------------------
    localparam int P_NORMAL = 0;
    localparam int P_CORNER = 1;
    localparam int P_WRAP   = 2;
    localparam int P_ABUSE  = 3;
    localparam int P_BAD    = 4;
    localparam int P_RESET  = 5;
    localparam int P_CHUNK  = 6;
    localparam int N_PROF   = 7;

    // After a job that fired junk loads at a busy core, the next job must run on
    // the SAME buffers (no reload), or a load that wrongly landed would be
    // overwritten before anything read it
    bit force_reuse = 1'b0;

    int goal_arg;
    int min_jobs_arg;
    int max_jobs_arg;

    // A value from the edges of 0..maxv: 0, 1, maxv-1, maxv
    function int pick_edge(input int maxv);
        case (rand_range(0, 3))
            0: pick_edge = 0;
            1: pick_edge = imin(1, maxv);
            2: pick_edge = imax(0, maxv - 1);
            default: pick_edge = maxv;
        endcase
    endfunction

    // Weighted choice. A profile whose bins are still short gets extra weight.
    function int pick_profile();
        int w [N_PROF];
        int total, r, acc;
        bit found;
        w[P_NORMAL] = 40;
        w[P_CORNER] = 15;
        w[P_WRAP]   = 15;
        w[P_ABUSE]  = 10;
        w[P_BAD]    = 5;
        w[P_RESET]  = 5;
        w[P_CHUNK]  = 5;
        if (range_missing(0, 26, goal_arg))                          w[P_NORMAL] += 25;
        if (range_missing(B_KT, B_OP + 5, goal_arg))                 w[P_CORNER] += 25;
        if (range_missing(B_W_POS, B_W_K1, goal_arg))                w[P_WRAP]   += 25;
        if (range_missing(B_ABS, B_ABL + 1, goal_arg))               w[P_ABUSE]  += 25;
        if (range_missing(B_BAD, B_BAD + 3, goal_arg))               w[P_BAD]    += 25;
        if (range_missing(B_RST, B_RST + 3, goal_arg))               w[P_RESET]  += 25;
        if (range_missing(B_W_CHK, B_W_CHK, goal_arg) ||
            range_missing(B_CHUNK, B_CHUNK, goal_arg))               w[P_CHUNK]  += 25;
        total = 0;
        for (int p = 0; p < N_PROF; p++) total += w[p];
        r   = rand_range(0, total - 1);
        acc = 0;
        pick_profile = P_NORMAL;
        found = 1'b0;
        for (int p = 0; p < N_PROF; p++) begin
            acc += w[p];
            if (!found && r < acc) begin
                pick_profile = p;
                found = 1'b1;
            end
        end
    endfunction

    // After a job that used operand profile `prof`: record the cross bins
    task automatic sample_cross(input int k_len, input int m_act, input int n_act,
                                input bit acc_i, input bit emit_i, input int prof);
        int kc, tc;
        kc = kclass_of(k_len);
        tc = tclass_of(m_act, n_act);
        hit(B_KT + kc*5 + tc);
        if (m_act > 0 && n_act > 0) hit(B_AE + (acc_i ? 2 : 0) + (emit_i ? 1 : 0));
        if (prof >= 0) hit(B_OP + prof);     // reused buffers have no profile of their own
    endtask

    // Wrap bins for a job that started from seeded accumulators
    task automatic sample_wrap(input int k_len, input int m_act, input int n_act,
                               input bit emit_i, input bit seeded, input bit in_chunk);
        bit wrapped;
        wrapped = job_wrap_pos || job_wrap_neg;
        if (job_wrap_pos) hit(B_W_POS);
        if (job_wrap_neg) hit(B_W_NEG);
        if (seeded && !wrapped) hit(B_W_NO);
        if (wrapped && m_act > 0 && n_act > 0 && (m_act < ROWS || n_act < COLS)) hit(B_W_PAR);
        if (wrapped && !emit_i) hit(B_W_SIL);
        if (wrapped && k_len == 1) hit(B_W_K1);
        if (wrapped && in_chunk) hit(B_W_CHK);
    endtask

    // One iteration of the NORMAL / CORNER / WRAP / ABUSE profiles
    task automatic job_profile(input string label, input int prof_kind);
        int k_len, m_act, n_act, gap, op_prof, seed_mode, ab_s, ab_l, len;
        bit acc_i, emit_i, reuse, seeded;

        // --- settings ---
        k_len  = rand_range(1, K_DEPTH);
        m_act  = (rand_range(0, 9) < 7) ? ROWS : rand_range(0, ROWS);
        n_act  = (rand_range(0, 9) < 7) ? COLS : rand_range(0, COLS);
        acc_i  = (rand_range(0, 2) == 0);
        emit_i = (rand_range(0, 3) != 0);
        gap    = rand_range(0, 2);
        op_prof = rand_range(0, 5);
        seed_mode = -1;
        ab_s = 0;
        ab_l = 0;

        case (prof_kind)
            P_CORNER: begin
                case (rand_range(0, 3))
                    0: k_len = 1;
                    1: k_len = clampi(2, 1, K_DEPTH);
                    2: k_len = clampi(K_DEPTH - 1, 1, K_DEPTH);
                    default: k_len = K_DEPTH;
                endcase
                m_act   = pick_edge(ROWS);
                n_act   = pick_edge(COLS);
                op_prof = (rand_range(0, 3) == 0) ? 5 : rand_range(1, 3);
            end
            P_WRAP: begin
                k_len   = (rand_range(0, 3) == 0) ? 1 : rand_range(1, imin(4, K_DEPTH));
                if (rand_range(0, 1) == 0) begin
                    m_act = rand_range(1, ROWS);
                    n_act = rand_range(1, COLS);
                end else begin
                    m_act = ROWS;
                    n_act = COLS;
                end
                acc_i     = 1'b1;
                emit_i    = (rand_range(0, 1) == 1);
                op_prof   = (rand_range(0, 3) == 0) ? 5 : rand_range(1, 3);
                seed_mode = rand_range(0, 2);
            end
            P_ABUSE: begin
                emit_i = (rand_range(0, 3) != 0);
                len    = k_len + ROWS + COLS + ((emit_i && m_act > 0) ? m_act : 0);
                ab_s   = (rand_range(0, 3) != 0) ? rand_range(1, len) : 0;
                ab_l   = (rand_range(0, 1) == 0) ? rand_range(1, len) : 0;
            end
            default: ;
        endcase

        // --- operands: reload, or reuse what the buffers still hold ---
        reuse = (prof_kind == P_NORMAL) && (last_loaded_k >= 1) && (rand_range(0, 3) == 0);
        if (force_reuse) begin
            reuse = 1'b1;
            k_len = last_loaded_k;          // read every slice the junk could have hit
        end else if (reuse) begin
            k_len = rand_range(1, last_loaded_k);
        end
        if (reuse) begin
            force_reuse = 1'b0;
        end else begin
            fill_profile(op_prof);
            load_buffers(k_len);
        end

        seeded = (seed_mode >= 0);
        if (seeded) seed_accumulators(label, seed_mode);

        run_job_e(label, k_len, m_act, n_act, acc_i, emit_i, gap, ab_s, ab_l);
        if (ab_l > 0) force_reuse = 1'b1;

        sample_cross(k_len, m_act, n_act, acc_i, emit_i, reuse ? -1 : op_prof);
        sample_wrap(k_len, m_act, n_act, emit_i, seeded, 1'b0);
    endtask

    // A K-split over three jobs: accumulate on top of each other, emit only
    // on the last. Optionally started from seeded accumulators.
    task automatic chunk_profile(input string label);
        int k_len, m_act, n_act, op_prof;
        bit seeded;
        m_act = (rand_range(0, 1) == 0) ? ROWS : rand_range(1, ROWS);
        n_act = (rand_range(0, 1) == 0) ? COLS : rand_range(1, COLS);
        seeded = (rand_range(0, 3) != 0);
        if (seeded) seed_accumulators(label, rand_range(0, 2));
        for (int c = 0; c < 3; c++) begin
            k_len   = rand_range(1, K_DEPTH);
            op_prof = (rand_range(0, 1) == 0) ? 5 : rand_range(1, 3);
            fill_profile(op_prof);
            load_buffers(k_len);
            run_job_e($sformatf("%0s-chunk%0d", label, c), k_len, m_act, n_act, 1'b1,
                      (c == 2), rand_range(0, 1), 0, 0);
            sample_wrap(k_len, m_act, n_act, (c == 2), seeded && c == 0, (c > 0));
        end
        hit(B_CHUNK);
    endtask

    task automatic reset_profile(input string label);
        int k_len, m_act, n_act, rows, cut;
        bit emit_i;
        k_len  = rand_range(1, K_DEPTH);
        m_act  = rand_range(0, ROWS);
        n_act  = rand_range(0, COLS);
        emit_i = (rand_range(0, 3) != 0);
        rows   = (emit_i && m_act > 0) ? m_act : 0;
        // readout cuts are only possible with rows to send; bias towards them
        if (rows > 0 && rand_range(0, 2) == 0)
            cut = rand_range(k_len + ROWS + COLS, k_len + ROWS + COLS + rows);
        else
            cut = rand_range(1, k_len + ROWS + COLS);
        fill_profile(rand_range(0, 5));
        load_buffers(k_len);
        reset_test(label, k_len, m_act, n_act, (rand_range(0, 1) == 1), emit_i, cut);
    endtask

    task automatic bad_profile(input string label);
        int k, m, n, pick;
        k = rand_range(1, K_DEPTH);
        m = rand_range(0, ROWS);
        n = rand_range(0, COLS);
        pick = rand_range(0, 3);
        if      (pick == 1 && CAN_K_OVER) k = rand_range(K_DEPTH + 1, K_REP_MAX);
        else if (pick == 2 && CAN_M_OVER) m = rand_range(ROWS + 1, M_REP_MAX);
        else if (pick == 3 && CAN_N_OVER) n = rand_range(COLS + 1, N_REP_MAX);
        else                              k = 0;
        try_bad_job(label, k, m, n, (rand_range(0, 1) == 1));
    endtask

    // ------------------------------------------------------------------
    // Coverage report
    // ------------------------------------------------------------------
    int bins_missed = 0;

    task automatic report_coverage();
        $display("Functional coverage (goal: %0d hits per bin):", goal_arg);
        for (int i = 0; i < NBINS; i++) begin
            if (!bin_possible[i])
                $display("  [n/a ] %0s", bin_name[i]);
            else if (bin_hits[i] >= goal_arg)
                $display("  [hit ] %0s (%0d)", bin_name[i], bin_hits[i]);
            else begin
                $display("  [MISS] %0s (%0d)", bin_name[i], bin_hits[i]);
                bins_missed++;
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    int jobs;
    int prof_count [N_PROF];
    int seed_arg;
    int p_now;

    initial begin
        $dumpfile("tensor_core_cr.vcd");
        $dumpvars(0, tensor_core_cr_tb);

        if (!$value$plusargs("seed=%d", seed_arg))         seed_arg     = 42;
        if (!$value$plusargs("goal=%d", goal_arg))         goal_arg     = 3;
        if (!$value$plusargs("min_jobs=%d", min_jobs_arg)) min_jobs_arg = 150;
        if (!$value$plusargs("max_jobs=%d", max_jobs_arg)) max_jobs_arg = 4000;
        rng_seed = seed_arg;
        define_bins();
        for (int p = 0; p < N_PROF; p++) prof_count[p] = 0;

        clk      = 0;
        rst_n    = 0;
        drive_idle();
        for (int i = 0; i < ROWS; i++)
            for (int j = 0; j < COLS; j++) exp_c[i][j] = 0;

        // Release reset; zero the buffers so no word is ever unknown
        @(posedge clk);
        #1 rst_n = 1;
        @(posedge clk);
        #1;
        expect_bit(busy,      1'b0, "post-reset busy");
        expect_bit(job_done,  1'b0, "post-reset job_done");
        expect_bit(job_err,   1'b0, "post-reset job_err");
        expect_bit(res_valid, 1'b0, "post-reset res_valid");
        for (int i = 0; i < ROWS; i++)
            for (int k = 0; k < K_DEPTH; k++) A_mat[i][k] = 0;
        for (int k = 0; k < K_DEPTH; k++)
            for (int j = 0; j < COLS; j++) B_mat[k][j] = 0;
        load_buffers(K_DEPTH);

        // Coverage-driven loop
        jobs = 0;
        while (jobs < max_jobs_arg && !(jobs >= min_jobs_arg && all_hit(goal_arg))) begin
            p_now = force_reuse ? P_NORMAL : pick_profile();
            prof_count[p_now]++;
            case (p_now)
                P_RESET: reset_profile($sformatf("cr-%0d-reset", jobs));
                P_BAD:   bad_profile($sformatf("cr-%0d-bad", jobs));
                P_CHUNK: chunk_profile($sformatf("cr-%0d-chunk", jobs));
                default: job_profile($sformatf("cr-%0d", jobs), p_now);
            endcase
            jobs++;
        end

        // A start rejected in the very last busy cycle pulses job_err one cycle
        // later: let the monitor see it before the tallies are compared
        repeat (3) begin
            @(posedge clk);
            #1;
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
        $display("seed=%0d  iterations=%0d  profiles: normal=%0d corner=%0d wrap=%0d abuse=%0d bad=%0d reset=%0d chunk=%0d",
                 seed_arg, jobs, prof_count[0], prof_count[1], prof_count[2], prof_count[3],
                 prof_count[4], prof_count[5], prof_count[6]);
        $display("SUMMARY: %0d / %0d checks passed (%0d failed)",
                  total_checks - total_fails, total_checks, total_fails);
        report_coverage();
        if (total_fails == 0 && bins_missed == 0) begin
            $display("RESULT: PASS");
        end else begin
            $display("RESULT: FAIL (%0d failed checks, %0d missed bins) - repeat with +seed=%0d",
                     total_fails, bins_missed, seed_arg);
        end
        $display("Testbench complete.");
        $finish;
    end

endmodule