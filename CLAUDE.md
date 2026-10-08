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
- [x] **M5** — Control / FSM
- [x] **M6** — Parameterisation
- [ ] **M7** — Advanced verification *(current milestone)*
- [ ] **M8** — FPGA implementation
- [ ] **M9** — Performance analysis
- [ ] **M10** — PPA comparison
- [ ] **M11** — (stretch) SoC integration
- [ ] **M12** — (stretch) ASIC flow

> Update the checkboxes above as each milestone is completed — this is the single source of truth for project status.

## Current status
M1, M2, M3, M4, M5 and M6 complete. Starting **M7: Advanced verification**.

M1 — MAC unit decisions:
- Data width: 8-bit signed operands (INT8-style), parameterised via `DATA_WIDTH`
- Accumulator width: parameterised via `ACC_WIDTH` (32-bit default)
- Port list: `clk`, `rst_n`, `clear_acc`, `valid_in`, `a`, `b`, `acc_out`, `valid_out`
- Style: combinational multiply, registered accumulate (1-cycle throughput) as the starting point before considering deeper pipelining

M2 — Verification of MAC unit (`tb/mac_unit_tb.sv`):
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

M5 — Control / FSM (`rtl/operand_buffer.sv`, `rtl/tile_ctrl.sv`, `rtl/tensor_core.sv`, `tb/operand_buffer_tb.sv`, `tb/tensor_core_tb.sv`):
- Architecture: `tensor_core` top = two `operand_buffer`s (A word = ROWS*DATA_WIDTH, B word = COLS*DATA_WIDTH, one word per k-slice) + `tile_ctrl` FSM + the unchanged M4 `matrix_engine`. `operand_buffer` is a synchronous RAM with a registered read (1-cycle latency, read-first on a same-address collision, out-of-range writes ignored and reads return 0, no reset)
- Decisions: one tile per job (the host loops over tiles and K-chunks), explicit-address buffer writes (`ld_en` + `ld_addr`), row-serial result stream, K <= `K_DEPTH` per job (longer K = several jobs with `cfg_accumulate`)
- Host flow: load slices (`ld_en`, `ld_addr`, `ld_a`, `ld_b`), pulse `job_start` with `cfg_k / cfg_m / cfg_n / cfg_accumulate / cfg_emit`, wait for `job_done`, read rows from `res_valid / res_row / res_data[COLS] / res_last`
- `tile_ctrl` states: IDLE -> LAUNCH -> STREAM -> DRAIN -> [READOUT]. `in_valid` = (state == STREAM) so the RAM's 1-cycle read latency lines up with the engine's "slices from the next cycle" protocol; only `in_last` is registered. `job_done` pulses K+ROWS+COLS cycles after the `job_start` cycle (same cycle as the engine's `done`). With `cfg_emit=1` and `m>0`, row r is on the stream r+1 cycles after `job_done`; `busy` stays high until the last row has gone out. `cfg_emit=0` keeps the sums inside (use it for all but the last chunk of a K-split). Columns >= `cfg_n` read 0 and `res_data` is 0 whenever `res_valid` is low
- Safety rules: every `job_start` either starts a job or pulses `job_err` one cycle later (cfg_k = 0, cfg_k > K_DEPTH, cfg_m > ROWS, cfg_n > COLS, or start while busy); nothing is dropped silently. `m = 0` or `n = 0` is a legal empty tile. Loads are ignored while busy (`ld_ok = ld_en && !busy`), so operands cannot change under the engine; a load in the same cycle as an accepted `job_start` is visible to that job ("load-and-go")
- Reset: asynchronous assert, active low (as in M1-M4). Reset in any phase returns every output to idle at once and clears the engine sums; the buffers keep their contents
- operand_buffer_tb: cycle-accurate golden model, directed + 4000-cycle random, 8 coverage bins, 4089/4089 checks, also passes at tiny and non-power-of-two depths
- tensor_core_tb (v4): golden model, `job_done` timing asserted exactly on every job, row-by-row stream checks, cycle monitor, `job_err` and stream-row tallies. Directed: K = 1 / K_DEPTH / mid, extremes, back-to-back, partial tiles, empty tiles, accumulate, 3-chunk K split (checked against the full-K product), rejected settings, start while busy in every phase (LAUNCH, STREAM, DRAIN, done cycle, readout), loads while busy, load-and-go, `cfg_emit=0`, reset in 7 phases. 150-job seeded (seed=42) random regression. Looks directly at `dut.c_tile` (white-box) so inactive PEs are verified too. ROWS / COLS / K_DEPTH are module parameters (`-Ptensor_core_tb.ROWS=..` etc.)
- Result: 32200/32200 checks passing at 4x4 / K_DEPTH=16 with all 31 coverage bins hit (`RESULT: PASS`); also passes at 17 size combinations from 1x1 / K_DEPTH=1 to 8x8 and K_DEPTH=32
- Verification quality: mutation-tested with ~60 injected controller / top-level bugs over the build (limit checks, `job_err`, accumulate, tile sizes, load gating, readout enable / last / row index / column mask / transpose, `busy` dropping early, start accepted in readout, synchronous or partial reset), all caught; operand_buffer 9 of 10 mutants caught, the survivor is equivalent in simulation; one equivalent reset mutant (`err_r` reset) cannot be seen by any test
- Known limits: a load while busy is ignored without a flag (`job_err` only covers `job_start`); no ping-pong buffers, so loading the next tile waits for the readout to finish; the result stream reads the engine's tile directly, so it is only valid until the next job starts (guaranteed because `busy` blocks it); no backpressure on the stream; the testbench uses a hierarchical reference to `dut.c_tile`
- Handed to later milestones: parameter legality checks and an automated ROWS / COLS / K_DEPTH sweep script (M6); constrained-random / coverage-driven extensions (M7); sync-vs-async reset decision for FPGA, buffer-to-block-RAM mapping (M8); ping-pong buffers, readout/next-job overlap, avoiding the per-chunk drain, partial-tile latency tightening (M9)
- Run (from the repo root): `iverilog -g2012 -o sim/tensor_core.vvp rtl/mac_unit.sv rtl/mac_pe.sv rtl/mac_array.sv rtl/skew_buffer.sv rtl/matrix_engine.sv rtl/operand_buffer.sv rtl/tile_ctrl.sv rtl/tensor_core.sv tb/tensor_core_tb.sv && vvp sim/tensor_core.vvp`
- Buffer run: `iverilog -g2012 -o sim/operand_buffer.vvp rtl/operand_buffer.sv tb/operand_buffer_tb.sv && vvp sim/operand_buffer.vvp`
- Regression at M5 sign-off (all passing): M2 `mac_unit_tb` 515/515, M3 `mac_array_tb` 3440/3440, `skew_buffer_tb` 2168/2168, M4 `matrix_engine_tb` 17329/17329, `operand_buffer_tb` 4089/4089, M5 `tensor_core_tb` 32200/32200

M6 — Parameterisation (`rtl/mac_unit.sv`, `rtl/tensor_core.sv`, `tb/tensor_core_tb.sv`, `scripts/sweep.sh`):
- Finding: ROWS / COLS / K_DEPTH were already free parameters since M3-M5. What was missing was parameter checking, and DATA_WIDTH / ACC_WIDTH coverage (the M5 testbench hardcoded 8-bit operands and a 32-bit golden model, so any other width reported failures that were testbench limits, not RTL bugs)
- Legality rules: `mac_unit` needs DATA_WIDTH >= 2 and ACC_WIDTH >= 2*DATA_WIDTH; `tensor_core` needs ROWS, COLS, K_DEPTH >= 1. Checks are `initial` blocks with `$fatal` (time 0). Icarus 12 ignores `$error` inside `generate` blocks, hence `initial`. Icarus already rejects ACC_WIDTH < 2*DATA_WIDTH and size 0 at compile time, so those checks mainly give readable messages and protect other tools; DATA_WIDTH = 1 is only caught by the `$fatal`
- Headroom warning (not fatal): `tensor_core` warns when ACC_WIDTH < 2*DATA_WIDTH + clog2(K_DEPTH), the width one job of K_DEPTH worst-case products needs to never wrap (20 at the defaults). Wrap-around stays legal (K-split jobs can outgrow it on purpose); the accumulator wraps, never saturates
- Testbench: `tensor_core_tb` takes the operand range from DATA_WIDTH (`OP_MIN` / `OP_MAX`), keeps the golden model in `longint` and wraps it to ACC_WIDTH (`wrap_acc`), including the K-split full-product check. Same stimulus and checks otherwise: 32200/32200 at the defaults, and the same count at every width combination swept
- Sweep (`bash scripts/sweep.sh` from the repo root, ~30 s, logs in `sim/sweep/`): 8 size combinations (1x1 K=1 up to 8x8 K=32), 12 width combinations (DATA_WIDTH 2..16, ACC_WIDTH 4..64, several tight enough to wrap), 4 mixed, 5 illegal settings that must be refused, and the five unit testbenches at their defaults. Result: 34 passed, 0 failed
- Known limits: the sweep is only as strong as the M5 testbench (no new stimulus); DATA_WIDTH is limited to 31 bits and ACC_WIDTH to 64 by the testbench's integer / `longint` model; checks live only in `mac_unit` and `tensor_core`, not in the intermediate modules (skew_buffer, operand_buffer, etc. still accept degenerate values on their own); the sweep needs `bash` and `iverilog` on the PATH
- Handed to later milestones: constrained-random / coverage-driven extensions and mutation re-runs at non-default sizes and widths (M7); sync-vs-async reset and block-RAM mapping (M8); per-parameter PPA numbers (M9/M10)
- Run (from the repo root): `bash scripts/sweep.sh`

## Conventions
- SystemVerilog (`.sv`) for all RTL and testbenches
- Parameters in `SCREAMING_SNAKE_CASE`, signals in `snake_case`
- Every module gets a matching testbench before moving to the next milestone
- Reset: active-low `rst_n`, asynchronous assert (`always_ff @(posedge clk or negedge rst_n)`) as used in M1-M5; revisit sync vs async in M8 (FPGA)

## Notes for Claude
- When starting a new session, check the roadmap above for current milestone before suggesting next steps.
- Keep explanations and code aligned with the goal of a clean, internship-portfolio-quality codebase — favor clarity and correctness over cleverness.
- After finishing a milestone, update this file's checkboxes and "Current status" section.
