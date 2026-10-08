#!/usr/bin/env python3
"""mutate.py - mutation testing for the tensor accelerator (M7).

Injects ONE bug at a time into a temporary copy of the RTL, runs the
self-checking testbench, and reports whether the testbench noticed.

    killed   (K) : the testbench failed, hung or hit $fatal -> bug was caught
    survived (S) : the testbench still printed RESULT: PASS  -> a gap, or the
                   bug is invisible at this size / width (an "equivalent" mutant)
    invalid  (I) : the mutated RTL does not compile at this configuration

The unmutated design is run first at every configuration; if it does not pass,
nothing else is run (a failing baseline would make every mutant look "killed").

Run from the repo root (needs python3 and iverilog on the PATH):

    python3 scripts/mutate.py                       # default 4x4, 8-bit config
    python3 scripts/mutate.py --suite               # the built-in set of configs
    python3 scripts/mutate.py --cfg 3 5 7 6 12      # ROWS COLS K_DEPTH DATA ACC
    python3 scripts/mutate.py --suite --only skew   # only mutants whose name has "skew"
    python3 scripts/mutate.py --list                # list the mutants

Repo files are never modified: every run happens in sim/mut/ and is cleaned up.
"""
import argparse
import concurrent.futures as cf
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

RTL = ["mac_unit.sv", "mac_pe.sv", "mac_array.sv", "skew_buffer.sv",
       "matrix_engine.sv", "operand_buffer.sv", "tile_ctrl.sv", "tensor_core.sv"]

# Testbenches the harness can drive. Each one takes ROWS / COLS / K_DEPTH /
# DATA_WIDTH / ACC_WIDTH as module parameters (-P overrides).
TESTBENCHES = {
    "core": dict(tb="tb/tensor_core_tb.sv", top="tensor_core_tb", rtl=RTL,
                 params=("ROWS", "COLS", "K_DEPTH", "DATA_WIDTH", "ACC_WIDTH")),
    "cr": dict(tb="tb/tensor_core_cr_tb.sv", top="tensor_core_cr_tb", rtl=RTL,
               params=("ROWS", "COLS", "K_DEPTH", "DATA_WIDTH", "ACC_WIDTH")),
    "engine": dict(tb="tb/matrix_engine_tb.sv", top="matrix_engine_tb",
                   rtl=["mac_unit.sv", "mac_pe.sv", "mac_array.sv",
                        "skew_buffer.sv", "matrix_engine.sv"],
                   params=("ROWS", "COLS", "DATA_WIDTH", "ACC_WIDTH")),
    # M2 unit testbench: fixed 8/32 sizes, so the configuration is ignored
    "unit": dict(tb="tb/mac_unit_tb.sv", top="mac_unit_tb",
                 rtl=["mac_unit.sv"], params=()),
}

# ROWS COLS K_DEPTH DATA_WIDTH ACC_WIDTH
DEFAULT_CFG = (4, 4, 16, 8, 32)
SUITE = [
    DEFAULT_CFG,
    (1, 1, 1, 2, 4),       # smallest legal everything
    (2, 2, 4, 3, 6),       # tiny, accumulator exactly one product wide
    (3, 5, 7, 6, 12),      # odd sizes, narrow data, wraps constantly
    (4, 4, 16, 8, 16),     # default size, tight accumulator (wraps)
    (6, 2, 9, 12, 24),     # tall array, wide data
    (5, 7, 6, 16, 40),     # non-power-of-two, 16-bit data
    (8, 8, 32, 8, 20),     # biggest array, borderline headroom
]


@dataclass
class Mutant:
    name: str
    file: str          # file in rtl/
    find: str          # text that must occur EXACTLY once in that file
    replace: str
    note: str = ""


# ----------------------------------------------------------------------------
# The mutants. Each is one small, plausible design bug.
# ----------------------------------------------------------------------------
M = []


def mut(name, file, find, replace, note=""):
    M.append(Mutant(name, file, find, replace, note))


# --- mac_unit ---------------------------------------------------------------
mut("mac_zero_extend", "mac_unit.sv",
    "{{(ACC_WIDTH-2*DATA_WIDTH){product[2*DATA_WIDTH-1]}}, product}",
    "{{(ACC_WIDTH-2*DATA_WIDTH){1'b0}}, product}",
    "product zero-extended instead of sign-extended")
mut("mac_product_narrow", "mac_unit.sv",
    "logic signed [2*DATA_WIDTH-1:0] product;",
    "logic signed [2*DATA_WIDTH-2:0] product;",
    "product loses its top bit")
mut("mac_subtract", "mac_unit.sv",
    "acc_out   <= acc_out + $signed(",
    "acc_out   <= acc_out - $signed(",
    "control: accumulates with the wrong sign")
mut("mac_saturate", "mac_unit.sv",
    "acc_out   <= acc_out + $signed({{(ACC_WIDTH-2*DATA_WIDTH){product[2*DATA_WIDTH-1]}}, product});",
    """begin : sat
                logic signed [ACC_WIDTH:0] s;
                s = $signed({acc_out[ACC_WIDTH-1], acc_out}) +
                    $signed({{(ACC_WIDTH-2*DATA_WIDTH+1){product[2*DATA_WIDTH-1]}}, product});
                if (s[ACC_WIDTH] != s[ACC_WIDTH-1])
                    acc_out <= s[ACC_WIDTH] ? {1'b1, {(ACC_WIDTH-1){1'b0}}}
                                            : {1'b0, {(ACC_WIDTH-1){1'b1}}};
                else
                    acc_out <= s[ACC_WIDTH-1:0];
            end""",
    "accumulator saturates instead of wrapping (only visible when a sum overflows)")
mut("mac_valid_ignored", "mac_unit.sv",
    "end else if (valid_in) begin",
    "end else if (1'b1) begin",
    "accumulates even when valid_in is low")
mut("mac_clear_loses_to_valid", "mac_unit.sv",
    "end else if (clear_acc) begin",
    "end else if (clear_acc && !valid_in) begin",
    "clear + valid in the same cycle (unit-level behaviour only)")

# --- mac_pe -----------------------------------------------------------------
mut("pe_a_valid_stuck", "mac_pe.sv",
    "a_valid_out <= a_valid_in;",
    "a_valid_out <= 1'b1;",
    "A valid forwarded as always-1")
mut("pe_b_forward_wrong", "mac_pe.sv",
    "b_out       <= b_in;",
    "b_out       <= a_in;",
    "B forwarding register takes the A operand")
mut("pe_valid_or", "mac_pe.sv",
    ".valid_in (a_valid_in & b_valid_in),",
    ".valid_in (a_valid_in | b_valid_in),",
    "MAC fires when only one operand is valid")
mut("pe_a_unforwarded", "mac_pe.sv",
    ".a        (a_in),",
    ".a        (a_out),",
    "MAC multiplies the delayed A operand")

# --- mac_array --------------------------------------------------------------
mut("arr_b_lanes_reversed", "mac_array.sv",
    "assign b_wire  [0][j] = b_in[j];",
    "assign b_wire  [0][j] = b_in[COLS-1-j];",
    "B edge lanes in reverse order")
mut("arr_a_valid_lane0", "mac_array.sv",
    "assign a_v_wire[i][0] = a_valid_in[i];",
    "assign a_v_wire[i][0] = a_valid_in[0];",
    "every row takes lane 0's valid")

# --- skew_buffer ------------------------------------------------------------
mut("skew_data_tap_wrong", "skew_buffer.sv",
    "data_pipe [s] <= data_pipe [s-1];",
    "data_pipe [s] <= data_pipe [0];",
    "inner stages of the data delay line skip a stage (needs a lane with 3+ stages)")
mut("skew_valid_tap_wrong", "skew_buffer.sv",
    "assign out_valid[i] = valid_pipe[i-1];",
    "assign out_valid[i] = valid_pipe[0];",
    "valid taken one stage too early on long lanes (needs a lane with 2+ stages)")
mut("skew_data_tap_early", "skew_buffer.sv",
    "assign out_data [i] = data_pipe [i-1];",
    "assign out_data [i] = data_pipe [0];",
    "data taken one stage too early on long lanes (needs a lane with 2+ stages)")

# --- matrix_engine ----------------------------------------------------------
mut("eng_drain_short", "matrix_engine.sv",
    "localparam DRAIN_CYCLES = ROWS + COLS - 1;",
    "localparam DRAIN_CYCLES = ROWS + COLS - 2;",
    "done one cycle early")
mut("eng_drain_long", "matrix_engine.sv",
    "localparam DRAIN_CYCLES = ROWS + COLS - 1;",
    "localparam DRAIN_CYCLES = ROWS + COLS;",
    "done one cycle late")
mut("eng_row_mask_le", "matrix_engine.sv",
    "a_lane_valid[i] = accept && (i < tile_m_r);",
    "a_lane_valid[i] = accept && (i <= tile_m_r);",
    "row mask off by one")
mut("eng_col_mask_le", "matrix_engine.sv",
    "b_lane_valid[j] = accept && (j < tile_n_r);",
    "b_lane_valid[j] = accept && (j <= tile_n_r);",
    "column mask off by one")
mut("eng_accumulate_ignored", "matrix_engine.sv",
    "assign clear_acc   = start_ok && !accumulate;",
    "assign clear_acc   = start_ok;",
    "start always clears")
mut("eng_tile_m_full", "matrix_engine.sv",
    "tile_m_r <= tile_m;",
    "tile_m_r <= ROWS;",
    "tile_m not latched, all rows active")
mut("eng_busy_drops_early", "matrix_engine.sv",
    "S_DRAIN:  if (done)        state <= S_IDLE;",
    "S_DRAIN:                   state <= S_IDLE;",
    "engine leaves DRAIN immediately")
mut("eng_last_without_valid", "matrix_engine.sv",
    "last_pipe <= (last_pipe << 1) | accept_last;",
    "last_pipe <= (last_pipe << 1) | in_last;",
    "in_last accepted without in_valid (engine-level protocol)")
mut("eng_accept_in_drain", "matrix_engine.sv",
    "assign accept      = in_valid && (state == S_STREAM);",
    "assign accept      = in_valid && (state != S_IDLE);",
    "slices accepted during DRAIN (engine-level protocol)")

# --- tile_ctrl --------------------------------------------------------------
mut("ctl_k_limit_lt", "tile_ctrl.sv",
    "(cfg_k <= K_DEPTH)",
    "(cfg_k < K_DEPTH)",
    "K = K_DEPTH wrongly rejected")
mut("ctl_k_zero_ok", "tile_ctrl.sv",
    "(cfg_k != '0) &&",
    "1'b1 &&",
    "K = 0 accepted")
mut("ctl_k_limit_off", "tile_ctrl.sv",
    "(cfg_k <= K_DEPTH)",
    "1'b1",
    "no upper limit on K (unobservable when K_DEPTH = 2^n - 1)")
mut("ctl_m_limit_off", "tile_ctrl.sv",
    "(cfg_m <= ROWS) &&",
    "1'b1 &&",
    "no upper limit on m (unobservable when ROWS = 2^n - 1)")
mut("ctl_n_limit_off", "tile_ctrl.sv",
    "(cfg_n <= COLS);",
    "1'b1;",
    "no upper limit on n (unobservable when COLS = 2^n - 1)")
mut("ctl_m_limit_lt", "tile_ctrl.sv",
    "(cfg_m <= ROWS) &&",
    "(cfg_m < ROWS) &&",
    "m = ROWS wrongly rejected")
mut("ctl_last_k1", "tile_ctrl.sv",
    "last_r <= (k_len == 1);",
    "last_r <= (k_len == 2);",
    "wrong last-slice detect for the first slice")
mut("ctl_last_off_by_one", "tile_ctrl.sv",
    "last_r <= (k_cnt + 1'b1 == k_len);",
    "last_r <= (k_cnt == k_len);",
    "in_last one slice late")
mut("ctl_res_last_wrong", "tile_ctrl.sv",
    "res_valid && (row_cnt + 1'b1 == m_r)",
    "res_valid && (row_cnt == m_r)",
    "res_last on the wrong row")
mut("ctl_emit_m0", "tile_ctrl.sv",
    "if (emit_r && m_r != '0) state <= S_READOUT;",
    "if (emit_r)              state <= S_READOUT;",
    "readout entered for an empty tile")
mut("ctl_err_busy_missed", "tile_ctrl.sv",
    "err_r <= job_start && ((state != S_IDLE) || !settings_ok);",
    "err_r <= job_start && !settings_ok;",
    "start while busy not flagged")

# --- tensor_core ------------------------------------------------------------
mut("top_load_while_busy", "tensor_core.sv",
    "assign ld_ok = ld_en && !busy;",
    "assign ld_ok = ld_en;",
    "loads accepted while busy")
mut("top_col_mask_le", "tensor_core.sv",
    "res_valid && (j < eng_tile_n)",
    "res_valid && (j <= eng_tile_n)",
    "result column mask off by one")
mut("top_result_transposed", "tensor_core.sv",
    "c_flat[(res_row * COLS + j) * ACC_WIDTH +: ACC_WIDTH]",
    "c_flat[(j * COLS + res_row) * ACC_WIDTH +: ACC_WIDTH]",
    "row-serial stream reads the tile transposed")
mut("top_b_buffer_gets_a", "tensor_core.sv",
    ".wr_data(ld_b),",
    ".wr_data(ld_a),",
    "B buffer is loaded with the A data")


# ----------------------------------------------------------------------------
# Running
# ----------------------------------------------------------------------------
def cfg_tag(cfg):
    r, c, k, d, a = cfg
    return f"{r}x{c} K{k} d{d}/a{a}"


def run_one(tb_key, cfg, mutant, work_root, timeout):
    """Returns (status, detail): status is P (baseline pass), F (baseline fail),
    K (killed), S (survived), I (invalid / does not compile)."""
    tb = TESTBENCHES[tb_key]
    work = Path(tempfile.mkdtemp(prefix="m_", dir=work_root))
    try:
        files = []
        for f in tb["rtl"]:
            src = ROOT / "rtl" / f
            dst = work / f
            text = src.read_text()
            if mutant is not None and f == mutant.file:
                text = text.replace(mutant.find, mutant.replace)
            dst.write_text(text)
            files.append(str(dst))
        r, c, k, d, a = cfg
        top = tb["top"]
        vals = dict(ROWS=r, COLS=c, K_DEPTH=k, DATA_WIDTH=d, ACC_WIDTH=a)
        cmd = ["iverilog", "-g2012", "-s", top,
               *[f"-P{top}.{n}={vals[n]}" for n in tb["params"]],
               "-o", str(work / "sim.vvp"), *files, str(ROOT / tb["tb"])]
        comp = subprocess.run(cmd, capture_output=True, text=True)
        if comp.returncode != 0:
            msg = (comp.stderr.strip().splitlines() or ["compile error"])[0]
            return ("I" if mutant else "F"), msg
        try:
            sim = subprocess.run(["vvp", str(work / "sim.vvp")], cwd=work,
                                 capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return ("K" if mutant else "F"), "timeout"
        out = sim.stdout + sim.stderr
        passed = (
            "RESULT: PASS" in out or "(0 failed)" in out) and "FATAL" not in out
        if mutant is None:
            return ("P" if passed else "F"), ""
        if passed:
            return "S", ""
        first = next((ln for ln in out.splitlines()
                     if ln.startswith("FAIL")), "failed")
        return "K", first[:70]
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(
        description="Mutation testing for the tensor accelerator")
    ap.add_argument("--cfg", nargs=5, type=int, action="append",
                    metavar=("ROWS", "COLS", "K_DEPTH", "DATA", "ACC"),
                    help="a configuration to test (repeatable)")
    ap.add_argument("--suite", action="store_true",
                    help="use the built-in set of configurations")
    ap.add_argument("--tb", default="core",
                    choices=sorted(TESTBENCHES), help="testbench to run")
    ap.add_argument("--only", default="",
                    help="only mutants whose name contains this text")
    ap.add_argument("-j", type=int, default=max(1,
                    (os.cpu_count() or 2) - 1), help="parallel jobs")
    ap.add_argument("--timeout", type=int, default=300,
                    help="seconds per simulation")
    ap.add_argument("--list", action="store_true",
                    help="list the mutants and exit")
    ap.add_argument("-v", action="store_true",
                    help="show why each mutant was killed")
    args = ap.parse_args()

    if args.list:
        for m in M:
            print(f"{m.name:26s} {m.file:18s} {m.note}")
        return 0
    if shutil.which("iverilog") is None or shutil.which("vvp") is None:
        print("iverilog / vvp not found on the PATH")
        return 2

    cfgs = [tuple(c) for c in (args.cfg or [])] or (
        SUITE if args.suite else [DEFAULT_CFG])
    # Only mutants in files this testbench compiles can be judged by it
    mutants = [m for m in M if args.only in m.name
               and m.file in TESTBENCHES[args.tb]["rtl"]]

    # Every pattern must match exactly once, or the mutant would silently do nothing
    bad = 0
    for m in mutants:
        n = (ROOT / "rtl" / m.file).read_text().count(m.find)
        if n != 1:
            print(
                f"BAD MUTANT {m.name}: pattern found {n} times in {m.file} (need exactly 1)")
            bad += 1
    if bad:
        return 2

    work_root = ROOT / "sim" / "mut"
    work_root.mkdir(parents=True, exist_ok=True)

    print(
        f"Testbench: {args.tb}   mutants: {len(mutants)}   configs: {len(cfgs)}   jobs: {args.j}")
    print("Baseline (unmutated design):")
    with cf.ThreadPoolExecutor(args.j) as ex:
        base = list(ex.map(lambda c: run_one(
            args.tb, c, None, work_root, args.timeout), cfgs))
    ok = True
    for c, (st, detail) in zip(cfgs, base):
        print(f"  {'PASS' if st == 'P' else 'FAIL'}  {cfg_tag(c)}  {detail}")
        ok &= (st == "P")
    if not ok:
        print("Baseline fails somewhere: fix that first (mutation results would be meaningless).")
        return 1

    jobs = [(m, c) for m in mutants for c in cfgs]
    with cf.ThreadPoolExecutor(args.j) as ex:
        results = list(ex.map(lambda mc: run_one(
            args.tb, mc[1], mc[0], work_root, args.timeout), jobs))
    res = {(m.name, c): r for (m, c), r in zip(jobs, results)}

    print()
    print("K = killed (caught)   S = SURVIVED (not caught)   I = does not compile")
    print()
    print("config legend:")
    for i, c in enumerate(cfgs):
        print(f"  {chr(ord('A') + i)} = {cfg_tag(c)}")
    print()
    print(f"{'mutant':26s} {''.join(chr(ord('A') + i) for i in range(len(cfgs))):{max(8, len(cfgs))}s}  note")
    for m in mutants:
        row = "".join(res[(m.name, c)][0] for c in cfgs)
        print(f"{m.name:26s} {row:{max(8, len(cfgs))}s}  {m.note}")
        if args.v:
            for c in cfgs:
                st, detail = res[(m.name, c)]
                if st in "KI" and detail:
                    print(f"{'':26s}   {chr(ord('A') + cfgs.index(c))}: {detail}")

    print()
    print("Per configuration:")
    total_surv = 0
    for i, c in enumerate(cfgs):
        sts = [res[(m.name, c)][0] for m in mutants]
        valid = sum(1 for s in sts if s != "I")
        killed = sts.count("K")
        surv = sts.count("S")
        total_surv += surv
        print(f"  {chr(ord('A') + i)}  {cfg_tag(c):22s} killed {killed:3d} / {valid:3d} valid, "
              f"survived {surv:3d}, invalid {sts.count('I'):3d}")

    print()
    print("Survivors (mutant: configs where it survived):")
    any_surv = False
    for m in mutants:
        where = [chr(ord('A') + i)
                 for i, c in enumerate(cfgs) if res[(m.name, c)][0] == "S"]
        if where:
            any_surv = True
            print(f"  {m.name:26s} {','.join(where)}")
    if not any_surv:
        print("  none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
