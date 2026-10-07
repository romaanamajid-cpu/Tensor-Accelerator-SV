// tensor_core.sv
// Top level of the accelerator: operand buffers + controller + matrix engine.
//
// Host flow for one job:
//   1. Load k-slices: for each k, pulse ld_en with ld_addr = k,
//      ld_a = column k of A (ROWS values), ld_b = row k of B (COLS values).
//   2. Pulse job_start together with cfg_k / cfg_m / cfg_n / cfg_accumulate /
//      cfg_emit.
//   3. job_done pulses when the engine has finished the tile. If cfg_emit = 1
//      the m active rows then come out on the result stream, one per cycle,
//      starting the cycle after job_done: res_valid, res_row, res_data
//      (COLS values, columns >= cfg_n read 0) and res_last with the final row.
//      cfg_emit = 0 keeps the sums inside (use it for all but the last chunk
//      of a K-split). busy stays high until the last row has gone out.
//
// A job_start that cannot be accepted (bad settings, or a job is already
// running) produces a job_err pulse and changes nothing.
// Loads (ld_en) are only accepted while the core is idle: a load during a job
// is ignored so the operands being streamed can never change under the engine.
module tensor_core #(
    parameter ROWS       = 4,
    parameter COLS       = 4,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32,
    parameter K_DEPTH    = 16,
    localparam ADDR_W    = (K_DEPTH > 1) ? $clog2(K_DEPTH) : 1,
    localparam K_W       = $clog2(K_DEPTH + 1),
    localparam M_W       = $clog2(ROWS + 1),
    localparam N_W       = $clog2(COLS + 1)
)(
    input  logic                                            clk,
    input  logic                                            rst_n,

    // Operand load (one k-slice per write)
    input  logic                                            ld_en,
    input  logic [ADDR_W-1:0]                               ld_addr,
    input  logic signed [ROWS-1:0][DATA_WIDTH-1:0]          ld_a,
    input  logic signed [COLS-1:0][DATA_WIDTH-1:0]          ld_b,

    // Job command / status
    input  logic                                            job_start,
    input  logic [K_W-1:0]                                  cfg_k,
    input  logic [M_W-1:0]                                  cfg_m,
    input  logic [N_W-1:0]                                  cfg_n,
    input  logic                                            cfg_accumulate,
    input  logic                                            cfg_emit,
    output logic                                            busy,
    output logic                                            job_done,
    output logic                                            job_err,

    // Result stream: one row of the tile per cycle
    output logic                                            res_valid,
    output logic [M_W-1:0]                                  res_row,
    output logic signed [COLS-1:0][ACC_WIDTH-1:0]           res_data,
    output logic                                            res_last
);

    // ------------------------------------------------------------------
    // Operand buffers: one word = one k-slice
    // ------------------------------------------------------------------
    logic [ADDR_W-1:0]                      rd_addr;
    logic signed [ROWS-1:0][DATA_WIDTH-1:0] a_slice;
    logic signed [COLS-1:0][DATA_WIDTH-1:0] b_slice;
    logic                                   ld_ok;      // load accepted (core idle)

    assign ld_ok = ld_en && !busy;

    operand_buffer #(
        .WIDTH(ROWS * DATA_WIDTH),
        .DEPTH(K_DEPTH)
    ) u_buf_a (
        .clk    (clk),
        .wr_en  (ld_ok),
        .wr_addr(ld_addr),
        .wr_data(ld_a),
        .rd_addr(rd_addr),
        .rd_data(a_slice)
    );

    operand_buffer #(
        .WIDTH(COLS * DATA_WIDTH),
        .DEPTH(K_DEPTH)
    ) u_buf_b (
        .clk    (clk),
        .wr_en  (ld_ok),
        .wr_addr(ld_addr),
        .wr_data(ld_b),
        .rd_addr(rd_addr),
        .rd_data(b_slice)
    );

    // ------------------------------------------------------------------
    // Controller
    // ------------------------------------------------------------------
    logic           eng_start;
    logic           eng_accumulate;
    logic [M_W-1:0] eng_tile_m;
    logic [N_W-1:0] eng_tile_n;
    logic           eng_in_valid;
    logic           eng_in_last;
    logic           eng_done;
    logic signed [ROWS-1:0][COLS-1:0][ACC_WIDTH-1:0] c_tile;   // full tile inside the engine

    tile_ctrl #(
        .ROWS   (ROWS),
        .COLS   (COLS),
        .K_DEPTH(K_DEPTH)
    ) u_ctrl (
        .clk           (clk),
        .rst_n         (rst_n),
        .job_start     (job_start),
        .cfg_k         (cfg_k),
        .cfg_m         (cfg_m),
        .cfg_n         (cfg_n),
        .cfg_accumulate(cfg_accumulate),
        .cfg_emit      (cfg_emit),
        .busy          (busy),
        .job_done      (job_done),
        .job_err       (job_err),
        .res_valid     (res_valid),
        .res_row       (res_row),
        .res_last      (res_last),
        .rd_addr       (rd_addr),
        .eng_start     (eng_start),
        .eng_accumulate(eng_accumulate),
        .eng_tile_m    (eng_tile_m),
        .eng_tile_n    (eng_tile_n),
        .eng_in_valid  (eng_in_valid),
        .eng_in_last   (eng_in_last),
        .eng_done      (eng_done)
    );

    // ------------------------------------------------------------------
    // The verified M4 engine, unchanged
    // ------------------------------------------------------------------
    matrix_engine #(
        .ROWS      (ROWS),
        .COLS      (COLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_engine (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (eng_start),
        .accumulate(eng_accumulate),
        .tile_m    (eng_tile_m),
        .tile_n    (eng_tile_n),
        .busy      (),
        .done      (eng_done),
        .in_valid  (eng_in_valid),
        .in_last   (eng_in_last),
        .in_a      (a_slice),
        .in_b      (b_slice),
        .c_out     (c_tile)
    );

    // ------------------------------------------------------------------
    // Result stream: pick the current row out of the tile. Columns that are
    // not part of the job (>= cfg_n) read 0, and so does the whole bus when
    // no row is being sent. The tile cannot change while rows go out because
    // a new job cannot start until busy is low.
    // ------------------------------------------------------------------
    logic [ROWS*COLS*ACC_WIDTH-1:0] c_flat;     // flat view, indexable by res_row
    assign c_flat = c_tile;

    always_comb begin
        for (int j = 0; j < COLS; j++)
            res_data[j] = (res_valid && (j < eng_tile_n))
                          ? c_flat[(res_row * COLS + j) * ACC_WIDTH +: ACC_WIDTH]
                          : '0;
    end

endmodule