# Tensor Accelerator Project

## Goal
Design, verify, and implement a small AI/tensor accelerator in SystemVerilog on FPGA, with a full PPA (performance/power/area) analysis. This is a portfolio piece for hardware/RTL/FPGA internship applications (NVIDIA, AMD, Arm, Apple, Intel, Qualcomm, Broadcom).

The core of the design is a parameterised MAC-array matrix-multiplication accelerator.

## Environment
- MacBook, Apple Silicon (arm64)
- Native macOS toolchain — **no WSL**
- GitHub account: romaanamajid@gmail.com

## Roadmap (13 milestones)
- [x] **M0** — Dev environment setup (simulator, linter, waveform viewer, repo structure, build scripts)
- [x] **M1** — MAC unit
- [x] **M2** — Verification of MAC unit
- [x] **M3** — MAC array
- [x] **M4** — Matrix engine
- [ ] **M5** — Control / FSM *(current milestone)*
- [ ] **M6** — Parameterisation
- [ ] **M7** — Advanced verification
- [ ] **M8** — FPGA implementation
- [ ] **M9** — Performance analysis
- [ ] **M10** — PPA comparison
- [ ] **M11** — (stretch) SoC integration
- [ ] **M12** — (stretch) ASIC flow

> Update the checkboxes above as each milestone is completed — this is the single source of truth for project status.

## Current status
M1, M2, M3 and M4 complete. Starting **M5: Control / FSM**.

M1 — MAC unit decisions:
- Data width: 8-bit signed operands (INT8-style), parameterised via `DATA_WIDTH`
- Accumulator width: parameterised via `ACC_WIDTH` (32-bit default)
- Port list: `clk`, `rst_n`, `clear_acc`, `valid_in`, `a`, `b`, `acc_out`, `valid_out`
- Style: combinational multiply, registered accumulate (1-cycle throughput) as the starting point before considering deeper pipelining

M2 — Verification of MAC unit (`rtl/mac_unit_tb.sv`):
- Directed edge cases: operand extremes (-128/127), accumulator wrap-around past ACC_WIDTH, clear_acc+valid_in same-cycle race, back-to-back valid_in throughput
- Found and fixed a real testbench race condition: deasserting clear_acc in the same simulation time step as the triggering clock edge raced against the DUT's always_ff, occasionally causing a missed clear. Fixed with a pulse_clear() task that settles one extra timestep (#1) before releasing clear_acc.
- Constrained-random regression: 500 cycles of randomized (a, b, valid_in), seeded (seed=42) for reproducibility, self-checked every cycle against a reference accumulator model
- Functional coverage (manual counters — this Icarus Verilog build doesn't support SystemVerilog covergroups): both-positive, both-negative, mixed-sign, zero-operand, valid_in-deasserted, and clear+valid-same-cycle categories, all confirmed hit
- Result: 515/515 checks passing, 0 failures, full coverage

M3 — MAC array (`rtl/mac_pe.sv`, `rtl/mac_array.sv`, `tb/mac_array_tb.sv`):
- Architecture: output-stationary systolic array, `ROWS` x `COLS` (default 4x4). A flows left->right, B flows top->bottom, each PE accumulates its own C[i][j] in place
- `mac_pe`: wraps the unchanged M1/M2 `mac_unit`; one forwarding register stage per operand (and its valid bit); MAC fires only when `a_valid_in & b_valid_in`
- `mac_array`: `generate` grid of PEs, edge lanes `a_in[ROWS]` / `b_in[COLS]` with per-lane valids, packed 2D outputs `c_out[i][j]` / `c_valid_out[i][j]`, `clear_acc` broadcast to every PE
- The array does NOT skew its inputs: the driver delays row i by i cycles and column j by j cycles (done in the M3 testbench's `drive_matmul`; taken over in hardware by the M4 `matrix_engine`)
- Testbench: golden model (`exp_c` accumulates like the hardware, zeroed on clear), directed tests (identity, mixed-sign pattern, -128/127 extremes, K=1, mid-stream and leading stalls, partial tiles, accumulate-without-clear, clear-all) + 200-run seeded (seed=42) random regression over K, tile size, stalls and clearing. Invalid lanes are driven with non-zero junk data to prove valid-gating
- Result: 3440/3440 checks passing, 0 failures, all 8 coverage bins hit
- Verification quality: mutation-tested with 8 injected design bugs (valid-gating, transposed output, missing/combinational forwarding, stuck valid, uncleared PE), all caught. Valid-gating bugs are only caught by the partial-tile tests
- Known gap: the array testbench never asserts `clear_acc` and `valid_in` in the same cycle; that behaviour is covered at unit level by the M2 testbench
- Not yet done at M3 time: input skewing/feeding hardware (done in M4), control FSM (M5), ROWS/COLS/K sweep and parameter checks (M6)

M4 — Matrix engine (`rtl/skew_buffer.sv`, `rtl/matrix_engine.sv`, `tb/skew_buffer_tb.sv`, `tb/matrix_engine_tb.sv`):
- Architecture: `matrix_engine` wraps the unchanged M3 `mac_array` with input masking, two `skew_buffer` instances (A side: ROWS lanes, B side: COLS lanes) and a small controller (IDLE -> STREAM -> DRAIN). `skew_buffer` delays lane i by i cycles (lane 0 is a wire); each lane's valid bit travels through the same registers as its data, so stalls stay aligned
- Decisions: operands arrive as a stream of k-slices (no tile buffers in the engine; M5 adds buffers/address generation), end of stream is signalled with an `in_last` tag on the final valid slice (no K counter, K unbounded), partial tiles via runtime `tile_m` / `tile_n` inputs (captured at start), result read as the whole tile on parallel `c_out` plus a `done` pulse
- Interface: `clk, rst_n`; control `start, accumulate, tile_m, tile_n` -> status `busy, done`; stream `in_valid, in_last, in_a[ROWS], in_b[COLS]`; result `c_out[ROWS][COLS]`. All widths follow ROWS/COLS/DATA_WIDTH/ACC_WIDTH (tile widths are `$clog2(ROWS+1)` / `$clog2(COLS+1)`)
- Protocol: pulse `start` while idle (`accumulate=0` clears every PE, `accumulate=1` keeps the sums for K-chunking); drive slices from the NEXT cycle (bubbles allowed anywhere, `in_last` on the final valid slice, exactly one per run); `done` pulses for one cycle exactly ROWS+COLS-1 cycles after the `in_last` slice, with the finished tile on `c_out` in that cycle. `busy` is low in the cycle after `done`, so the next `start` can land immediately (back-to-back). `c_out` is stable until the next start or next accepted slice
- Robustness: a start while busy, slices while idle or after `in_last`, and `in_last` without `in_valid` are all ignored. `tile_m = 0` or `tile_n = 0` is a legal empty tile (nothing fires). Done latency is the fixed worst case even for partial tiles
- skew_buffer_tb: impulse test, dense/sparse/random streams against a delayed-history model, reset flush with data in flight, 2168/2168 checks, 5 coverage bins hit, also passes at LANES = 1, 2, 3, 7, 8; 6 injected bugs all caught
- matrix_engine_tb: golden model, DRAIN latency asserted exactly on every run, cycle-by-cycle monitor (done only while busy, one cycle wide), result re-checked one cycle after done. Directed: identity, mixed-sign pattern, -128/127 extremes, K=1, leading/mid/pre-last stalls, partial tiles (1x1, Nx1, 1xN, rows-only, cols-only), empty tiles, 3-chunk accumulation, protocol abuse (start in stream, start in done cycle, slices after `in_last`, slices while idle), back-to-back starts, reset mid-stream and mid-drain. 200-run seeded (seed=42) random regression mixing all of these. ROWS/COLS are module parameters (`-Pmatrix_engine_tb.ROWS=..`)
- Result: 17329/17329 checks passing at 4x4 with all 18 coverage bins hit (`RESULT: PASS`); also passes at 1x1, 1x4, 4x1, 2x2, 2x3, 3x2, 5x7, 8x8
- Verification quality: mutation-tested with 26 injected engine bugs (latency +-1, idle/after-last/in-stream acceptance, start while busy or in DRAIN, mask ignored/off-by-one/wrong dimension, tile size not latched, accumulate ignored/inverted/never-clear, `in_last` without valid, either skew removed, busy stuck/low in drain, done 2 cycles wide, reset leaving state or the last token, valid cross-wired, A/B swapped, result transposed), all caught (non-square sizes re-checked for the mask/swap/transpose mutants). Thinnest catch: a reset that leaves the in-flight last token is detected only by the reset-mid-drain test (4 failing checks)
- Known limits: no backpressure (the engine is always ready while streaming); a run needs at least one slice and an `in_last` (a missing `in_last` leaves `busy` high until reset); no result snapshot register (the next `start` with `accumulate=0` clears the tile, so read `c_out` by then); accumulator wrap-around beyond ACC_WIDTH is covered at unit level (M2), not at engine level; size sweeps are run by hand with `-P` overrides
- Handed to later milestones: control FSM, operand buffers and address generation, result write-back (M5); automated ROWS/COLS/K sweep script and parameter legality checks (M6); partial-tile latency tightening is a possible M9 optimisation
- Run (from the repo root): `iverilog -g2012 -o sim/matrix_engine.vvp rtl/mac_unit.sv rtl/mac_pe.sv rtl/mac_array.sv rtl/skew_buffer.sv rtl/matrix_engine.sv tb/matrix_engine_tb.sv && vvp sim/matrix_engine.vvp`

## Conventions
- SystemVerilog (`.sv`) for all RTL and testbenches
- Parameters in `SCREAMING_SNAKE_CASE`, signals in `snake_case`
- Every module gets a matching testbench before moving to the next milestone
- Prefer synchronous, active-low reset (`rst_n`) unless a specific milestone calls for otherwise

## Notes for Claude
- When starting a new session, check the roadmap above for current milestone before suggesting next steps.
- Keep explanations and code aligned with the goal of a clean, internship-portfolio-quality codebase — favor clarity and correctness over cleverness.
- After finishing a milestone, update this file's checkboxes and "Current status" section.
