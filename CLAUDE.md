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
- [ ] **M4** — Matrix engine *(current milestone)*
- [ ] **M5** — Control / FSM
- [ ] **M6** — Parameterisation
- [ ] **M7** — Advanced verification
- [ ] **M8** — FPGA implementation
- [ ] **M9** — Performance analysis
- [ ] **M10** — PPA comparison
- [ ] **M11** — (stretch) SoC integration
- [ ] **M12** — (stretch) ASIC flow

> Update the checkboxes above as each milestone is completed — this is the single source of truth for project status.

## Current status
M1, M2 and M3 complete. Starting **M4: Matrix engine**.

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
- The array does NOT skew its inputs: the driver delays row i by i cycles and column j by j cycles (done in the testbench's `drive_matmul`; the M4 engine must take this over)
- Testbench: golden model (`exp_c` accumulates like the hardware, zeroed on clear), directed tests (identity, mixed-sign pattern, -128/127 extremes, K=1, mid-stream and leading stalls, partial tiles, accumulate-without-clear, clear-all) + 200-run seeded (seed=42) random regression over K, tile size, stalls and clearing. Invalid lanes are driven with non-zero junk data to prove valid-gating
- Result: 3440/3440 checks passing, 0 failures, all 8 coverage bins hit
- Verification quality: mutation-tested with 8 injected design bugs (valid-gating, transposed output, missing/combinational forwarding, stuck valid, uncleared PE), all caught. Valid-gating bugs are only caught by the partial-tile tests
- Known gap: the array testbench never asserts `clear_acc` and `valid_in` in the same cycle; that behaviour is covered at unit level by the M2 testbench
- Not yet done (belongs to later milestones): input skewing/feeding hardware (M4), control FSM (M5), ROWS/COLS/K sweep and parameter checks (M6)

## Conventions
- SystemVerilog (`.sv`) for all RTL and testbenches
- Parameters in `SCREAMING_SNAKE_CASE`, signals in `snake_case`
- Every module gets a matching testbench before moving to the next milestone
- Prefer synchronous, active-low reset (`rst_n`) unless a specific milestone calls for otherwise

## Notes for Claude
- When starting a new session, check the roadmap above for current milestone before suggesting next steps.
- Keep explanations and code aligned with the goal of a clean, internship-portfolio-quality codebase — favor clarity and correctness over cleverness.
- After finishing a milestone, update this file's checkboxes and "Current status" section.
