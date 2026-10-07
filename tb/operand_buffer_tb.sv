`timescale 1ns/1ps

// operand_buffer_tb.sv
// Self-checking testbench for the operand buffer (M5, step 1).
//
// A cycle-accurate golden model (mem_model) is updated with exactly the values
// the RAM sees at each clock edge, and rd_data is compared with it after every
// edge. The model implements the documented behaviour:
//   - registered read (one cycle latency)
//   - read-first on a read/write collision
//   - out-of-range writes ignored, out-of-range reads return 0
//
// Directed tests also check expected values written out by hand, so the model
// and the specification cannot drift apart unnoticed.
//
// WIDTH / DEPTH are module parameters, e.g.
//   iverilog -Poperand_buffer_tb.DEPTH=12 -Poperand_buffer_tb.WIDTH=64 ...
module operand_buffer_tb;

    parameter WIDTH = 32;
    parameter DEPTH = 16;

    localparam ADDR_W    = (DEPTH > 1) ? $clog2(DEPTH) : 1;
    localparam ADDR_SPAN = 1 << ADDR_W;       // addresses the port can express
    localparam HAS_OOR   = (ADDR_SPAN != DEPTH);

    // Addresses used by the directed tests (clamped so tiny DEPTH values work)
    localparam A_LAT_1 = (DEPTH > 3) ? 3 : 0;
    localparam A_LAT_2 = (DEPTH > 6) ? 6 : (DEPTH - 1);
    localparam A_RDW   = (DEPTH > 5) ? 5 : (DEPTH - 1);
    localparam A_GATE  = (DEPTH > 1) ? 1 : 0;

    logic clk = 1'b0;
    logic                wr_en;
    logic [ADDR_W-1:0]   wr_addr;
    logic [WIDTH-1:0]    wr_data;
    logic [ADDR_W-1:0]   rd_addr;
    logic [WIDTH-1:0]    rd_data;

    operand_buffer #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH)
    ) dut (
        .clk    (clk),
        .wr_en  (wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .rd_addr(rd_addr),
        .rd_data(rd_data)
    );

    // Clock: 10ns period
    always #5 clk = ~clk;

    // Watchdog
    initial begin
        #50_000_000;
        $display("TIMEOUT: testbench did not finish");
        $finish;
    end

    // ------------------------------------------------------------------
    // Golden model, counters, coverage
    // ------------------------------------------------------------------
    logic [WIDTH-1:0] mem_model [DEPTH];

    int total_checks = 0;
    int total_fails  = 0;

    int cov_write_read   = 0;  // a written word was read back
    int cov_first_last   = 0;  // address 0 and DEPTH-1 both read back
    int cov_rd_during_wr = 0;  // read and write of the same address, different data
    int cov_gated_write  = 0;  // wr_en = 0 while wr_addr/wr_data would have changed a word
    int cov_overwrite    = 0;  // a word was overwritten
    int cov_back_to_back = 0;  // different read address on consecutive cycles
    int cov_hold         = 0;  // same read address held while other words are written
    int cov_out_of_range = 0;  // out-of-range write or read (only when DEPTH is not 2^n)
    int bins_missed      = 0;

    int seen_first = 0;
    int seen_last  = 0;

    integer rng_seed;

    // Deterministic word for (address, salt)
    function logic [WIDTH-1:0] pat(input int addr, input int salt);
        logic [31:0] v;
        v = (addr + 1) * 32'h9E37_79B1;
        v = v ^ (salt * 32'h0101_0101) ^ 32'hA5A5_5A5A;
        pat = {WIDTH{1'b0}} | v;
    endfunction

    // Random WIDTH-bit word (32 random bits at a time)
    function logic [WIDTH-1:0] rand_word(input int dummy);
        logic [WIDTH-1:0] w;
        w = '0;
        for (int b = 0; b < WIDTH; b += 32) w[b +: 32] = $random(rng_seed);
        rand_word = w;
    endfunction

    // Previous-cycle read address, for the back-to-back / hold bins
    logic [ADDR_W-1:0] prev_ra;
    bit                have_prev = 1'b0;
    bit                prev_wrote_other = 1'b0;

    // ------------------------------------------------------------------
    // One clock cycle: drive the inputs, update the model with what the RAM
    // sees at the coming edge, then compare rd_data just after the edge.
    // ------------------------------------------------------------------
    task automatic cycle(input logic en,
                         input logic [ADDR_W-1:0] wa,
                         input logic [WIDTH-1:0]  wd,
                         input logic [ADDR_W-1:0] ra,
                         input string             tag);
        logic [WIDTH-1:0] exp_rd;
        wr_en   = en;
        wr_addr = wa;
        wr_data = wd;
        rd_addr = ra;

        // Read side first (read-first behaviour), then the write
        exp_rd = (ra < DEPTH) ? mem_model[ra] : '0;

        // Coverage that depends on the pre-edge model state
        if (en && wa < DEPTH && wa == ra && wd !== mem_model[wa]) cov_rd_during_wr++;
        if (en && wa < DEPTH && wd !== mem_model[wa])             cov_overwrite++;
        if (!en && wa < DEPTH && wd !== mem_model[wa])            cov_gated_write++;
        if (have_prev && ra !== prev_ra)                          cov_back_to_back++;
        if (have_prev && ra === prev_ra && prev_wrote_other)      cov_hold++;
        if (HAS_OOR && ((en && wa >= DEPTH) || ra >= DEPTH))      cov_out_of_range++;

        if (en && wa < DEPTH) mem_model[wa] = wd;

        prev_wrote_other = en && (wa !== ra);
        prev_ra   = ra;
        have_prev = 1'b1;

        @(posedge clk);
        #1;
        total_checks++;
        if (rd_data !== exp_rd) begin
            total_fails++;
            $display("FAIL [%0s] t=%0t: rd_addr=%0d rd_data=%h expected %h",
                     tag, $time, ra, rd_data, exp_rd);
        end
    endtask

    // Compare rd_data (already registered by the last cycle()) with a value
    // that the test wrote by hand, independent of the model.
    task automatic expect_word(input logic [WIDTH-1:0] expected, input string tag);
        total_checks++;
        if (rd_data !== expected) begin
            total_fails++;
            $display("FAIL [%0s] t=%0t: rd_data=%h expected %h", tag, $time, rd_data, expected);
        end
    endtask

    task automatic idle_cycle(input logic [ADDR_W-1:0] ra, input string tag);
        cycle(1'b0, '0, '0, ra, tag);
    endtask

    // ------------------------------------------------------------------
    // Coverage report
    // ------------------------------------------------------------------
    task automatic report_bin(input string name, input int count, input bit required);
        if (!required)        $display("  [n/a ] %0s", name);
        else if (count > 0)   $display("  [hit ] %0s (%0d)", name, count);
        else begin
            $display("  [MISS] %0s", name);
            bins_missed++;
        end
    endtask

    task automatic report_coverage();
        $display("Functional coverage:");
        report_bin("write then read back",     cov_write_read,   1);
        report_bin("first and last address",   cov_first_last,   1);
        report_bin("read during write",        cov_rd_during_wr, 1);
        report_bin("gated write (wr_en = 0)",  cov_gated_write,  1);
        report_bin("overwrite",                cov_overwrite,    1);
        report_bin("back-to-back reads",       cov_back_to_back, 1);
        report_bin("read address held",        cov_hold,         1);
        report_bin("out-of-range access",      cov_out_of_range, HAS_OOR);
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    logic [WIDTH-1:0] old_word, new_word;

    initial begin
        $dumpfile("operand_buffer.vcd");
        $dumpvars(0, operand_buffer_tb);

        wr_en   = 1'b0;
        wr_addr = '0;
        wr_data = '0;
        rd_addr = '0;
        rng_seed = 42;

        // Step 1: zero every word so the model and the RAM start defined
        for (int a = 0; a < ADDR_SPAN; a++)
            cycle(1'b1, a[ADDR_W-1:0], '0, '0, "init-sweep");

        // Step 2: write every address with its own pattern, read every one back
        for (int a = 0; a < DEPTH; a++)
            cycle(1'b1, a[ADDR_W-1:0], pat(a, 1), '0, "write-sweep");

        for (int a = 0; a < DEPTH; a++) begin
            idle_cycle(a[ADDR_W-1:0], "read-sweep");
            expect_word(pat(a, 1), "read-sweep-value");
            cov_write_read++;
            if (a == 0)       seen_first = 1;
            if (a == DEPTH-1) seen_last  = 1;
        end
        if (seen_first && seen_last) cov_first_last++;

        // Step 3: latency - the word shows up exactly one cycle after the
        // address, and not before. Change the address, check the NEXT cycle.
        idle_cycle(A_LAT_1, "latency-setup");
        expect_word(pat(A_LAT_1, 1), "latency-setup-value");
        rd_addr = A_LAT_2;                          // same cycle: still the old word
        #1 expect_word(pat(A_LAT_1, 1), "latency-no-early-change");
        idle_cycle(A_LAT_2, "latency-next");
        expect_word(pat(A_LAT_2, 1), "latency-new-word");

        // Step 4: read during write of the same address returns the OLD word,
        // the new word follows one cycle later. The word is restored afterwards
        // so later directed tests can keep expecting pat(addr, 1).
        old_word = pat(A_RDW, 1);
        new_word = pat(A_RDW, 2);
        cycle(1'b1, A_RDW, new_word, A_RDW, "rdw");
        expect_word(old_word, "rdw-old-value");
        idle_cycle(A_RDW, "rdw-next");
        expect_word(new_word, "rdw-new-value");
        cycle(1'b1, A_RDW, old_word, A_RDW, "rdw-restore");

        // Step 5: write gating - wr_en = 0 with a live address and data must not
        // change anything
        cycle(1'b0, A_GATE, pat(A_GATE, 9), A_GATE, "gated");
        expect_word(pat(A_GATE, 1), "gated-unchanged");
        idle_cycle(A_GATE, "gated-readback");
        expect_word(pat(A_GATE, 1), "gated-readback-value");

        // Step 6: overwrite one word, neighbours must stay as they were
        if (DEPTH >= 3) begin
            cycle(1'b1, 1, pat(1, 3), 0, "overwrite");
            idle_cycle(0, "overwrite-prev");
            expect_word(pat(0, 1), "overwrite-left-neighbour");
            idle_cycle(1, "overwrite-target");
            expect_word(pat(1, 3), "overwrite-target-value");
            idle_cycle(2, "overwrite-next");
            expect_word(pat(2, 1), "overwrite-right-neighbour");
        end

        // Step 7: read address held while other words are written
        idle_cycle(0, "hold-1");
        cycle(1'b1, DEPTH > 1 ? 1 : 0, rand_word(0), 0, "hold-2");
        cycle(1'b1, DEPTH > 2 ? 2 : 0, rand_word(0), 0, "hold-3");
        idle_cycle(0, "hold-4");

        // Step 8: out-of-range addresses (only exist when DEPTH is not 2^n)
        if (HAS_OOR) begin
            cycle(1'b1, ADDR_SPAN-1, {WIDTH{1'b1}}, ADDR_SPAN-1, "oor-write-read");
            expect_word('0, "oor-read-is-zero");
            idle_cycle(ADDR_SPAN-1, "oor-readback");
            expect_word('0, "oor-still-zero");
            for (int a = 0; a < DEPTH; a++)           // nothing in range was disturbed
                idle_cycle(a[ADDR_W-1:0], "oor-in-range-intact");
        end

        // Step 9: constrained-random regression - random reads and writes on
        // every cycle, full address span, checked against the model each cycle
        for (int n = 0; n < 4000; n++)
            cycle(($random(rng_seed) & 1),
                  $random(rng_seed),
                  rand_word(0),
                  $random(rng_seed),
                  "random");

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