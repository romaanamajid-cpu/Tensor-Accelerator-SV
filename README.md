# Configurable Matrix-Multiplication Accelerator (SystemVerilog)

Design, verification, and FPGA implementation of a parameterised
MAC-array based matrix-multiplication accelerator, built from the
ground up as a portfolio project for hardware/RTL/FPGA internships.

## Status
Milestones 0-7 complete (RTL, verification, parameterisation, advanced
verification). Milestone 8 (FPGA implementation) is next. See CLAUDE.md
for the roadmap and the per-milestone design notes.

## Design
Output-stationary systolic MAC array (ROWS x COLS, signed DATA_WIDTH
operands, wrapping ACC_WIDTH accumulators) with input skew buffers, operand
buffers and a job controller. Everything is parameterised and checks its own
parameters.

## Verification
- Self-checking golden-model testbenches for every block
- Coverage-driven constrained-random testbench (50 manual bins, seeded)
- Mutation testing: 39 injected bugs across 4 testbenches (scripts/mutate.py)
- One-command regression: `bash scripts/sweep.sh` (72 checks)

## Structure
- /rtl      synthesizable SystemVerilog RTL
- /tb       testbenches
- /scripts  sweep.sh (regression), mutate.py (mutation testing)
- /docs     architecture notes (to come)
- /results  waveforms, synthesis reports, performance data
- /python   golden reference models (to come)
- /fpga     constraints, bitstreams, board files