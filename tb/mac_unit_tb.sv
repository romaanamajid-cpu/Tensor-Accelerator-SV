`timescale 1ns/1ps

module mac_unit_tb;

    localparam DATA_WIDTH = 8;
    localparam ACC_WIDTH  = 32;

    logic clk;
    logic rst_n;
    logic clear_acc;
    logic valid_in;
    logic signed [DATA_WIDTH-1:0] a, b;
    logic signed [ACC_WIDTH-1:0]  acc_out;
    logic valid_out;

    // Reference model
    logic signed [ACC_WIDTH-1:0] expected_acc;

    // Pass/fail tally across the whole run (directed + random)
    int total_checks = 0;
    int total_fails  = 0;

    // Functional coverage (manual counters — this Icarus build doesn't
    // support SystemVerilog covergroups)
    int cov_both_positive   = 0;
    int cov_both_negative   = 0;
    int cov_mixed_sign      = 0;
    int cov_either_zero     = 0;
    int cov_valid_low       = 0;
    int cov_clear_and_valid = 0;

    mac_unit #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .clear_acc(clear_acc),
        .valid_in(valid_in),
        .a(a),
        .b(b),
        .acc_out(acc_out),
        .valid_out(valid_out)
    );

    // Clock: 10ns period
    always #5 clk = ~clk;

    task automatic sample_coverage();
        if (a == 0 || b == 0)
            cov_either_zero++;
        else if (a > 0 && b > 0)
            cov_both_positive++;
        else if (a < 0 && b < 0)
            cov_both_negative++;
        else
            cov_mixed_sign++;

        if (!valid_in)
            cov_valid_low++;

        if (clear_acc && valid_in)
            cov_clear_and_valid++;
    endtask

    task automatic report_coverage();
        $display("==============================================");
        $display("COVERAGE:");
        $display("  both operands positive : %0d hits %s", cov_both_positive,   cov_both_positive   > 0 ? "[HIT]" : "[MISS]");
        $display("  both operands negative : %0d hits %s", cov_both_negative,   cov_both_negative   > 0 ? "[HIT]" : "[MISS]");
        $display("  mixed sign operands    : %0d hits %s", cov_mixed_sign,      cov_mixed_sign      > 0 ? "[HIT]" : "[MISS]");
        $display("  either operand zero    : %0d hits %s", cov_either_zero,     cov_either_zero     > 0 ? "[HIT]" : "[MISS]");
        $display("  valid_in deasserted    : %0d hits %s", cov_valid_low,       cov_valid_low       > 0 ? "[HIT]" : "[MISS]");
        $display("  clear_acc + valid_in   : %0d hits %s", cov_clear_and_valid, cov_clear_and_valid > 0 ? "[HIT]" : "[MISS]");
        $display("==============================================");
    endtask

    task automatic apply_input(input signed [DATA_WIDTH-1:0] a_in,
                                input signed [DATA_WIDTH-1:0] b_in,
                                input bit valid);
        a = a_in;
        b = b_in;
        valid_in = valid;
        sample_coverage();
        @(posedge clk);
    endtask

    task automatic check_acc(input string label);
        #1; // let signals settle after the clock edge
        total_checks++;
        if (acc_out !== expected_acc) begin
            total_fails++;
            $display("FAIL [%s]: acc_out = %0d, expected = %0d", label, acc_out, expected_acc);
        end else
            $display("PASS [%s]: acc_out = %0d", label, acc_out);
    endtask

    // Cleanly pulses clear_acc for one cycle. Uses the same #1 settle
    // delay as check_acc before releasing clear_acc, to avoid a race
    // against the DUT's always_ff sampling clear_acc on the same edge.
    // Also forces valid_in low so no stale operation can sneak through.
    task automatic pulse_clear();
        clear_acc = 1;
        valid_in  = 0;
        @(posedge clk);
        #1;
        clear_acc = 0;
    endtask

    // Constrained-random regression: drives num_cycles of randomized
    // (a, b, valid_in) at the DUT and self-checks every cycle against
    // the same expected_acc reference model used by the directed tests.
    // Fixed seed -> reproducible: a failure at cycle N will reproduce
    // at cycle N on every re-run with the same seed.
    task automatic random_regression(input int num_cycles, input int init_seed);
        logic signed [DATA_WIDTH-1:0] rand_a, rand_b;
        bit rand_valid;
        integer seed;
        seed = init_seed;
        pulse_clear();
        expected_acc = 0;
        for (int i = 0; i < num_cycles; i++) begin
            rand_a     = $random(seed);                        // truncates to 8 bits, signed
            rand_b     = $random(seed);
            rand_valid = ($unsigned($random(seed)) % 10) < 8;  // ~80% valid_in high
            apply_input(rand_a, rand_b, rand_valid);
            if (rand_valid)
                expected_acc += rand_a * rand_b;
            check_acc($sformatf("random-%0d", i));
        end
    endtask

    initial begin
        $dumpfile("mac_unit.vcd");
        $dumpvars(0, mac_unit_tb);
        clk = 0;
        rst_n = 0;
        clear_acc = 0;
        valid_in = 0;
        a = 0; b = 0;
        expected_acc = 0;

        // Step 1: release reset, check zeroed state
        @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        check_acc("post-reset");

        // Step 2: basic accumulation, positive values
        apply_input(3, 4, 1);   expected_acc += 3*4;   check_acc("acc1");
        apply_input(5, 2, 1);   expected_acc += 5*2;   check_acc("acc2");

        // Step 3: negative number handling
        apply_input(-8, 6, 1);  expected_acc += -8*6;  check_acc("acc-neg1");
        apply_input(-3, -3, 1); expected_acc += -3*-3; check_acc("acc-neg2");

        // Step 4: clear_acc
        pulse_clear();
        expected_acc = 0;
        check_acc("clear");

        // Step 5: valid_in gating (should hold value)
        apply_input(10, 10, 0); // valid_in low, no accumulate
        check_acc("gated-hold");

        // ============================================================
        // M2 — Boundary / edge-case tests
        // ============================================================

        // Step 6: reset reference model, then hit extreme operand values
        pulse_clear();
        expected_acc = 0;

        apply_input(-128, 127, 1);
        expected_acc += (-128) * 127;
        check_acc("edge-max-neg-product");

        apply_input(-128, -128, 1);
        expected_acc += (-128) * (-128);
        check_acc("edge-max-pos-product");

        // Step 7: drive the accumulator until it wraps past the
        // ACC_WIDTH boundary
        pulse_clear();
        expected_acc = 0;

        for (int i = 0; i < 200000; i++) begin
            apply_input(-128, 127, 1);
            expected_acc += (-128) * 127;
        end
        check_acc("acc-wraparound");

        // Step 8: clear_acc and valid_in asserted in the same cycle
        clear_acc = 1;
        valid_in  = 1;
        a = 10; b = 10;
        sample_coverage();
        @(posedge clk);
        #1;
        clear_acc = 0;
        valid_in  = 0;
        expected_acc = 0;
        check_acc("clear-and-valid-same-cycle");

        // Step 9: back-to-back valid_in, no gaps, throughput case
        apply_input(1, 1, 1);  expected_acc += 1*1;  check_acc("burst-1");
        apply_input(2, 2, 1);  expected_acc += 2*2;  check_acc("burst-2");
        apply_input(3, 3, 1);  expected_acc += 3*3;  check_acc("burst-3");
        apply_input(4, 4, 1);  expected_acc += 4*4;  check_acc("burst-4");

        // ============================================================
        // M2 — Constrained-random regression
        // ============================================================

        // Step 10: 500 cycles of randomized (a, b, valid_in), self-checked
        // against the reference model every cycle
        random_regression(500, 42);

        $display("==============================================");
        $display("SUMMARY: %0d / %0d checks passed (%0d failed)",
                  total_checks - total_fails, total_checks, total_fails);
        report_coverage();

        $display("Testbench complete.");
        $finish;
    end

endmodule