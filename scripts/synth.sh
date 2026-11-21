#!/usr/bin/env bash
# scripts/synth.sh - FPGA "virtual board" flow for the tensor accelerator (M8).
#
# Runs Yosys (synthesis for Lattice ECP5) and nextpnr-ecp5 (place and route,
# timing) on rtl/tensor_core.sv and prints one summary: logic used, hardware
# multipliers, block RAM and the maximum clock frequency. No board is needed.
#
# Usage (from the repo root, after: source ~/oss-cad-suite/environment):
#   bash scripts/synth.sh                       # defaults: 4x4, 8-bit, 32-bit acc, K_DEPTH 16
#   bash scripts/synth.sh --rows 8 --cols 8 --freq 80 --device 45k
# Options:
#   --rows N --cols N --data-width N --acc-width N --k-depth N   design parameters
#   --freq MHz     timing target given to nextpnr (default 100)
#   --device D     ECP5 size: 25k (default), 45k or 85k
#   --seed N       placer seed (default 1)
#   --tag NAME     output folder name under build/synth/ (default: built from the settings)
# Outputs: build/synth/<tag>/{yosys.log,stat.txt,nextpnr.log,summary.txt}

set -euo pipefail

ROWS=4; COLS=4; DATA_WIDTH=8; ACC_WIDTH=32; K_DEPTH=16
FREQ=100; DEVICE=25k; SEED=1; TAG=""
# Prefer the matched pip tools (yowasp-*) if installed: the OSS CAD Suite snapshot
# can pair a Yosys and a nextpnr that do not work together. Override with YOSYS=/NEXTPNR=.
YOSYS="${YOSYS:-$(command -v yowasp-yosys || echo yosys)}"
NEXTPNR="${NEXTPNR:-$(command -v yowasp-nextpnr-ecp5 || echo nextpnr-ecp5)}"

while [ $# -gt 0 ]; do
    case "$1" in
        --rows)       ROWS="$2";       shift 2 ;;
        --cols)       COLS="$2";       shift 2 ;;
        --data-width) DATA_WIDTH="$2"; shift 2 ;;
        --acc-width)  ACC_WIDTH="$2";  shift 2 ;;
        --k-depth)    K_DEPTH="$2";    shift 2 ;;
        --freq)       FREQ="$2";       shift 2 ;;
        --device)     DEVICE="$2";     shift 2 ;;
        --seed)       SEED="$2";       shift 2 ;;
        --tag)        TAG="$2";        shift 2 ;;
        *) echo "synth.sh: unknown option '$1' (see the header of this file)" >&2; exit 2 ;;
    esac
done

case "$DEVICE" in 25k|45k|85k) ;; *) echo "synth.sh: --device must be 25k, 45k or 85k" >&2; exit 2 ;; esac

[ -f rtl/tensor_core.sv ] || { echo "synth.sh: run this from the repo root" >&2; exit 2; }
command -v "$YOSYS"   >/dev/null || { echo "synth.sh: '$YOSYS' not found - run: source ~/oss-cad-suite/environment" >&2; exit 2; }
command -v "$NEXTPNR" >/dev/null || { echo "synth.sh: '$NEXTPNR' not found - run: source ~/oss-cad-suite/environment" >&2; exit 2; }

[ -n "$TAG" ] || TAG="r${ROWS}c${COLS}_d${DATA_WIDTH}_a${ACC_WIDTH}_k${K_DEPTH}_${DEVICE}_f${FREQ}"
OUT="build/synth/$TAG"
mkdir -p "$OUT"

SRC="rtl/mac_unit.sv rtl/mac_pe.sv rtl/mac_array.sv rtl/skew_buffer.sv rtl/matrix_engine.sv rtl/operand_buffer.sv rtl/tile_ctrl.sv rtl/tensor_core.sv"

echo "== Yosys: synthesising tensor_core (ROWS=$ROWS COLS=$COLS DATA_WIDTH=$DATA_WIDTH ACC_WIDTH=$ACC_WIDTH K_DEPTH=$K_DEPTH)"
"$YOSYS" -q -l "$OUT/yosys.log" -p "
    read_verilog -sv -DSYNTHESIS $SRC
    chparam -set ROWS $ROWS -set COLS $COLS -set DATA_WIDTH $DATA_WIDTH -set ACC_WIDTH $ACC_WIDTH -set K_DEPTH $K_DEPTH tensor_core
    synth_ecp5 -top tensor_core -json $OUT/design.json
    tee -o $OUT/stat.txt stat
" >/dev/null 2>&1 || { echo "synth.sh: Yosys failed - see $OUT/yosys.log" >&2; exit 1; }

echo "== nextpnr-ecp5: place and route for ECP5-$DEVICE at ${FREQ} MHz target"
"$NEXTPNR" --"$DEVICE" --package CABGA381 --json "$OUT/design.json" \
    --freq "$FREQ" --seed "$SEED" --out-of-context --timing-allow-fail \
    --log "$OUT/nextpnr.log" >/dev/null 2>&1 \
    || { echo "synth.sh: nextpnr failed - see $OUT/nextpnr.log" >&2; exit 1; }

cell() { awk -v c="$1" '$2 == c { n = $1 } END { print (n == "" ? 0 : n) }' "$OUT/stat.txt"; }

FMAX_LINE=$(grep "Max frequency for clock" "$OUT/nextpnr.log" | tail -1 || true)
FMAX=$(echo "$FMAX_LINE" | sed -E 's/.*: ([0-9.]+) MHz.*/\1/')
if   echo "$FMAX_LINE" | grep -q "(PASS"; then MET="met"
elif echo "$FMAX_LINE" | grep -q "(FAIL"; then MET="NOT met"
else FMAX="n/a"; MET="unknown"; fi

{
    echo "Tensor accelerator - ECP5-$DEVICE (nextpnr, no board)"
    echo "Settings : ROWS=$ROWS COLS=$COLS DATA_WIDTH=$DATA_WIDTH ACC_WIDTH=$ACC_WIDTH K_DEPTH=$K_DEPTH seed=$SEED"
    echo "LUT4             : $(cell LUT4)"
    echo "Carry (CCU2C)    : $(cell CCU2C)"
    echo "Flip-flops       : $(cell TRELLIS_FF)"
    echo "Multipliers      : $(cell MULT18X18D)  (MULT18X18D)"
    echo "LUT RAM          : $(cell TRELLIS_DPR16X4)  (TRELLIS_DPR16X4)"
    echo "Block RAM        : $(cell DP16KD)  (DP16KD)"
    echo "Fmax             : $FMAX MHz  (target $FREQ MHz: $MET)"
} | tee "$OUT/summary.txt"
echo "Logs in $OUT/"