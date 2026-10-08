#!/usr/bin/env bash
# sweep.sh - parameter sweep for the tensor core (M6).
#
# Run from the repo root:   bash scripts/sweep.sh
#
# 1. Runs the self-checking tensor_core_tb at many ROWS / COLS / K_DEPTH and
#    DATA_WIDTH / ACC_WIDTH settings. A run passes when the testbench prints
#    "RESULT: PASS".
# 2. Runs deliberately ILLEGAL settings. A run passes when the design refuses
#    them (compile error or $fatal) instead of building a broken core.
# 3. Re-runs the unit-level testbenches at their default settings.
# Exit code is 0 only if everything passed. Logs go to sim/sweep/.

RTL="rtl/mac_unit.sv rtl/mac_pe.sv rtl/mac_array.sv rtl/skew_buffer.sv \
rtl/matrix_engine.sv rtl/operand_buffer.sv rtl/tile_ctrl.sv rtl/tensor_core.sv"
OUT=sim/sweep
mkdir -p "$OUT"

pass=0
fail=0

# run_core ROWS COLS K_DEPTH DATA_WIDTH ACC_WIDTH   (must pass)
run_core() {
    local tag="R$1_C$2_K$3_D$4_A$5"
    local log="$OUT/$tag.log"
    if iverilog -g2012 \
         -Ptensor_core_tb.ROWS=$1 -Ptensor_core_tb.COLS=$2 \
         -Ptensor_core_tb.K_DEPTH=$3 -Ptensor_core_tb.DATA_WIDTH=$4 \
         -Ptensor_core_tb.ACC_WIDTH=$5 \
         -o "$OUT/$tag.vvp" $RTL tb/tensor_core_tb.sv > "$log" 2>&1 \
       && vvp "$OUT/$tag.vvp" >> "$log" 2>&1 \
       && grep -q "RESULT: PASS" "$log"; then
        printf "PASS  rows=%-2s cols=%-2s k_depth=%-2s data=%-2s acc=%-2s  %s\n" \
               $1 $2 $3 $4 $5 "$(grep -o '[0-9]* / [0-9]* checks passed' "$log")"
        pass=$((pass + 1))
    else
        printf "FAIL  rows=%-2s cols=%-2s k_depth=%-2s data=%-2s acc=%-2s  (see %s)\n" \
               $1 $2 $3 $4 $5 "$log"
        fail=$((fail + 1))
    fi
    rm -f "$OUT/$tag.vvp"
}

# run_illegal ROWS COLS K_DEPTH DATA_WIDTH ACC_WIDTH   (must be refused)
run_illegal() {
    local tag="bad_R$1_C$2_K$3_D$4_A$5"
    local log="$OUT/$tag.log"
    local refused=0
    if ! iverilog -g2012 \
           -Ptensor_core_tb.ROWS=$1 -Ptensor_core_tb.COLS=$2 \
           -Ptensor_core_tb.K_DEPTH=$3 -Ptensor_core_tb.DATA_WIDTH=$4 \
           -Ptensor_core_tb.ACC_WIDTH=$5 \
           -o "$OUT/$tag.vvp" $RTL tb/tensor_core_tb.sv > "$log" 2>&1; then
        refused=1                                   # compile / elaboration error
    elif ! vvp "$OUT/$tag.vvp" >> "$log" 2>&1 || grep -q "FATAL" "$log"; then
        refused=1                                   # $fatal at time 0
    fi
    if [ $refused -eq 1 ]; then
        printf "PASS  rows=%-2s cols=%-2s k_depth=%-2s data=%-2s acc=%-2s  refused (illegal)\n" \
               $1 $2 $3 $4 $5
        pass=$((pass + 1))
    else
        printf "FAIL  rows=%-2s cols=%-2s k_depth=%-2s data=%-2s acc=%-2s  ACCEPTED an illegal setting\n" \
               $1 $2 $3 $4 $5
        fail=$((fail + 1))
    fi
    rm -f "$OUT/$tag.vvp"
}

# run_unit NAME "rtl files" tb_file
run_unit() {
    local log="$OUT/unit_$1.log"
    if iverilog -g2012 -o "$OUT/unit_$1.vvp" $2 $3 > "$log" 2>&1 \
       && vvp "$OUT/unit_$1.vvp" >> "$log" 2>&1 \
       && grep -q "SUMMARY: \([0-9]*\) / \1 checks passed" "$log"; then
        printf "PASS  %-16s %s\n" "$1" "$(grep -o '[0-9]* / [0-9]* checks passed' "$log")"
        pass=$((pass + 1))
    else
        printf "FAIL  %-16s (see %s)\n" "$1" "$log"
        fail=$((fail + 1))
    fi
    rm -f "$OUT/unit_$1.vvp"
}

echo "=== Sizes: ROWS x COLS, K_DEPTH (8-bit data, 32-bit accumulator) ==="
for cfg in "1 1 1" "1 4 8" "4 1 8" "2 3 5" "3 2 16" "4 4 16" "5 7 6" "8 8 32"; do
    set -- $cfg
    run_core $1 $2 $3 8 32
done

echo
echo "=== Widths: DATA_WIDTH / ACC_WIDTH (4x4, K_DEPTH 16) ==="
for cfg in "2 4" "2 8" "3 6" "4 8" "4 12" "8 16" "8 20" "8 32" "12 24" "16 32" "16 40" "16 64"; do
    set -- $cfg
    run_core 4 4 16 $1 $2
done

echo
echo "=== Mixed: odd sizes with narrow and wide data ==="
for cfg in "2 3 5 4 10" "3 5 7 6 16" "1 1 1 2 4" "6 2 9 12 32"; do
    set -- $cfg
    run_core $1 $2 $3 $4 $5
done

echo
echo "=== Illegal settings (must be refused) ==="
run_illegal 4 4 16 1 32       # DATA_WIDTH < 2
run_illegal 4 4 16 8 12       # ACC_WIDTH < 2*DATA_WIDTH
run_illegal 0 4 16 8 32       # ROWS = 0
run_illegal 4 0 16 8 32       # COLS = 0
run_illegal 4 4 0 8 32        # K_DEPTH = 0

echo
echo "=== Unit-level testbenches (default settings) ==="
run_unit mac_unit       "rtl/mac_unit.sv" tb/mac_unit_tb.sv
run_unit mac_array      "rtl/mac_unit.sv rtl/mac_pe.sv rtl/mac_array.sv" tb/mac_array_tb.sv
run_unit skew_buffer    "rtl/skew_buffer.sv" tb/skew_buffer_tb.sv
run_unit matrix_engine  "rtl/mac_unit.sv rtl/mac_pe.sv rtl/mac_array.sv rtl/skew_buffer.sv rtl/matrix_engine.sv" tb/matrix_engine_tb.sv
run_unit operand_buffer "rtl/operand_buffer.sv" tb/operand_buffer_tb.sv

echo
echo "SWEEP: $pass passed, $fail failed"
[ $fail -eq 0 ]