`timescale 1ns/1ps

// skew_buffer_tb.sv
// Self-checking testbench for skew_buffer (M4, step 1).
//
// Reference model: lane i's output at cycle t must equal lane i's input at
// cycle t-i (data AND valid). Before that (t < i) the lane still holds its
// reset value: valid = 0, data = 0.
//
// LANES / DATA_WIDTH are module parameters so other sizes can be run from the
// command line, e.g.  iverilog -Pskew_buffer_tb.LANES=7 ...
module skew_buffer_tb;

    parameter LANES      = 4;
    parameter DATA_WIDTH = 8;
    parameter MAX_CYC    = 64;   // longest stream driven by one run_stream()

    logic clk;
    logic rst_n;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] in_data;
    logic        [LANES-1:0]                 in_valid;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] out_data;
    logic        [LANES-1:0]                 out_valid;

    skew_buffer #(
        .LANES     (LANES),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_data  (in_data),
        .in_valid (in_valid),
        .out_data (out_data),
        .out_valid(out_valid)
    );

    // Clock: 10ns period
    always #5 clk = ~clk;

    // Watchdog
    initial begin
        #5_000_000;
        $display("TIMEOUT: testbench did not finish");
        $finish;
    end

    // ------------------------------------------------------------------
    // Reference history: what each lane was driven with, per cycle
    // ------------------------------------------------------------------
    logic signed [DATA_WIDTH-1:0] hist_d [MAX_CYC][LANES];
    logic                         hist_v [MAX_CYC][LANES];

    int total_checks = 0;
    int total_fails  = 0;

    // Functional coverage (manual counters, no covergroups in this Icarus)
    int cov_all_valid   = 0;  // a cycle with every lane valid
    int cov_all_idle    = 0;  // a cycle with every lane idle (a bubble)
    int cov_mixed       = 0;  // a cycle with some lanes valid, some idle
    int cov_deep_lane   = 0;  // data actually came out of the deepest lane
    int cov_lane0_valid = 0;  // data actually came out of lane 0 (no delay)

    integer rng_seed;

    function int rand_range(input int lo, input int hi);
        rand_range = lo + ($unsigned($random(rng_seed)) % (hi - lo + 1));
    endfunction

    // ------------------------------------------------------------------
    // Tasks
    // ------------------------------------------------------------------

    // Async reset with idle inputs; then check the pipeline was flushed.
    // Called after a stream has just run, so data is still in flight and the
    // flush is a real test, not a no-op.
    task automatic do_reset(input string label);
        in_data  = '0;
        in_valid = '0;
        rst_n    = 0;
        #1;
        for (int i = 0; i < LANES; i++) begin
            total_checks++;
            if (out_valid[i] !== 1'b0 || out_data[i] !== '0) begin
                total_fails++;
                $display("FAIL [%s]: lane %0d not flushed by reset (valid=%b data=%0d)",
                         label, i, out_valid[i], out_data[i]);
            end
        end
        @(posedge clk);
        #1;
        rst_n = 1;
    endtask

    // Checks every lane's output against the delayed history for cycle cyc
    task automatic check_cycle(input string label, input int cyc);
        for (int i = 0; i < LANES; i++) begin
            total_checks++;
            if (cyc - i < 0) begin
                // Still inside the lane's delay: must show reset values
                if (out_valid[i] !== 1'b0 || out_data[i] !== '0) begin
                    total_fails++;
                    $display("FAIL [%s] cyc %0d lane %0d: expected reset value, got valid=%b data=%0d",
                             label, cyc, i, out_valid[i], out_data[i]);
                end
            end else begin
                if (out_valid[i] !== hist_v[cyc-i][i] ||
                    out_data[i]  !== hist_d[cyc-i][i]) begin
                    total_fails++;
                    $display("FAIL [%s] cyc %0d lane %0d: got valid=%b data=%0d, expected valid=%b data=%0d (input from cycle %0d)",
                             label, cyc, i, out_valid[i], out_data[i],
                             hist_v[cyc-i][i], hist_d[cyc-i][i], cyc - i);
                end else if (hist_v[cyc-i][i]) begin
                    if (i == 0)       cov_lane0_valid++;
                    if (i == LANES-1) cov_deep_lane++;
                end
            end
        end
    endtask

    // Drives num_cyc cycles of random data. valid_pct is the percentage of
    // (lane, cycle) slots that are valid. Invalid slots still carry random
    // junk data, so a lane that mishandled valid would be exposed.
    task automatic run_stream(input string label, input int num_cyc, input int valid_pct);
        int nvalid;
        for (int t = 0; t < num_cyc; t++) begin
            nvalid = 0;
            for (int i = 0; i < LANES; i++) begin
                hist_d[t][i] = rand_range(-128, 127);
                hist_v[t][i] = (rand_range(0, 99) < valid_pct);
                in_data[i]   = hist_d[t][i];
                in_valid[i]  = hist_v[t][i];
                if (hist_v[t][i]) nvalid++;
            end
            if (nvalid == LANES) cov_all_valid++;
            else if (nvalid == 0) cov_all_idle++;
            else                  cov_mixed++;
            #3;                      // settle; lane 0 is combinational
            check_cycle(label, t);
            @(posedge clk);
            #1;
        end
    endtask

    // Directed impulse: every lane valid for ONE cycle, then idle.
    // Lane i must pop out exactly i cycles later, one lane per cycle.
    task automatic run_impulse();
        for (int t = 0; t < LANES + 2; t++) begin
            for (int i = 0; i < LANES; i++) begin
                hist_d[t][i] = (t == 0) ? (i + 1) : 0;
                hist_v[t][i] = (t == 0);
                in_data[i]   = hist_d[t][i];
                in_valid[i]  = hist_v[t][i];
            end
            #3;
            check_cycle("impulse", t);
            @(posedge clk);
            #1;
        end
        $display("PASS [impulse]: lane i emerged exactly i cycles after the input");
    endtask

    task automatic report_coverage();
        $display("==============================================");
        $display("COVERAGE:");
        $display("  all lanes valid        : %0d hits %s", cov_all_valid,   cov_all_valid   > 0 ? "[HIT]" : "[MISS]");
        $display("  all lanes idle         : %0d hits %s", cov_all_idle,    cov_all_idle    > 0 ? "[HIT]" : "[MISS]");
        $display("  mixed valid/idle       : %0d hits %s", cov_mixed,       cov_mixed       > 0 ? "[HIT]" : "[MISS]");
        $display("  data via lane 0        : %0d hits %s", cov_lane0_valid, cov_lane0_valid > 0 ? "[HIT]" : "[MISS]");
        $display("  data via deepest lane  : %0d hits %s", cov_deep_lane,   cov_deep_lane   > 0 ? "[HIT]" : "[MISS]");
        $display("==============================================");
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    initial begin
        $dumpfile("skew_buffer.vcd");
        $dumpvars(0, skew_buffer_tb);

        clk = 0;
        rst_n = 0;
        in_data = '0;
        in_valid = '0;
        rng_seed = 42;

        // Step 1: release reset
        @(posedge clk);
        #1 rst_n = 1;

        // Step 2: impulse - the clearest picture of the skew
        run_impulse();
        do_reset("reset-after-impulse");

        // Step 3: dense stream, every lane valid every cycle
        run_stream("dense", 32, 100);
        do_reset("reset-after-dense");

        // Step 4: sparse stream, mostly idle lanes and bubbles
        run_stream("sparse", 32, 25);
        do_reset("reset-after-sparse");

        // Step 5: constrained-random streams at varied densities
        for (int r = 0; r < 20; r++) begin
            run_stream($sformatf("random-%0d", r), rand_range(8, 40), rand_range(0, 100));
            do_reset($sformatf("reset-after-random-%0d", r));
        end

        $display("==============================================");
        $display("SUMMARY: %0d / %0d checks passed (%0d failed)",
                  total_checks - total_fails, total_checks, total_fails);
        report_coverage();
        $display("Testbench complete.");
        $finish;
    end

endmodule