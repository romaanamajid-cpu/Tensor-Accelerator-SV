// tile_ctrl.sv
// Controller that runs ONE matrix job on the matrix engine.
//
// A job = "multiply the K k-slices currently stored in the operand buffers".
// The host loads the buffers, then pulses job_start with the job settings.
// The controller walks the buffer addresses 0..K-1, feeds the engine one
// slice per cycle and waits for the engine to finish.
//
//   IDLE   : waiting for job_start. Settings are captured on that edge.
//   LAUNCH : one cycle. Pulses the engine's start and puts address 0 on the
//            buffer. The buffer answers one cycle later.
//   STREAM : K cycles. The buffer output IS slice k, so in_valid is simply
//            "state == STREAM". in_last marks the final slice.
//   DRAIN  : waits for the engine's done. job_done pulses in that same cycle,
//            and the finished tile is ready inside the engine.
//   READOUT: only when cfg_emit = 1 and cfg_m > 0. Sends the m active rows of
//            the tile out, one row per cycle (res_valid / res_row / res_last).
//            The cycle after the last row the controller is idle again.
//
// Timeline for K = 3 (cycle 0 = the cycle job_start is high):
//   cycle      : 0     1       2      3      4      5 ...
//   state      : IDLE  LAUNCH  STREAM STREAM STREAM DRAIN ...
//   rd_addr    : -     0       1      2      3(x)   -
//   buffer out : -     -       s0     s1     s2     -
//   in_last    : 0     0       0      0      1(*)   0
//   (*) in_last is high in the cycle s2 is on the wire.
// job_done comes K + ROWS + COLS cycles after the job_start cycle.
// Row r of the result is on the stream r + 1 cycles after the job_done cycle.
// busy stays high until the last row has gone out.
//
// Settings rules (cfg_*): cfg_k must be 1..K_DEPTH, cfg_m <= ROWS and
// cfg_n <= COLS (0 is a legal, empty tile). Every job_start either starts a
// job or is REJECTED with a one-cycle job_err pulse in the following cycle:
//   - settings out of range (nothing is latched, the engine is not touched)
//   - job_start while busy (the running job is not disturbed)
// Nothing is ever dropped silently.
module tile_ctrl #(
    parameter ROWS    = 4,
    parameter COLS    = 4,
    parameter K_DEPTH = 16,
    localparam ADDR_W = (K_DEPTH > 1) ? $clog2(K_DEPTH) : 1,
    localparam K_W    = $clog2(K_DEPTH + 1),
    localparam M_W    = $clog2(ROWS + 1),
    localparam N_W    = $clog2(COLS + 1)
)(
    input  logic               clk,
    input  logic               rst_n,

    // Job command from the host
    input  logic               job_start,
    input  logic [K_W-1:0]     cfg_k,            // number of k-slices
    input  logic [M_W-1:0]     cfg_m,            // active rows
    input  logic [N_W-1:0]     cfg_n,            // active columns
    input  logic               cfg_accumulate,   // 1 = keep the previous sums
    input  logic               cfg_emit,         // 1 = stream the result rows out
    output logic               busy,
    output logic               job_done,         // one-cycle pulse: compute finished
    output logic               job_err,          // one-cycle pulse: job_start rejected

    // Result stream (row-serial). res_last is high with the final row.
    output logic               res_valid,
    output logic [M_W-1:0]     res_row,
    output logic               res_last,

    // Operand buffers (read side). Data comes back one cycle later.
    output logic [ADDR_W-1:0]  rd_addr,

    // Matrix engine
    output logic               eng_start,
    output logic               eng_accumulate,
    output logic [M_W-1:0]     eng_tile_m,
    output logic [N_W-1:0]     eng_tile_n,
    output logic               eng_in_valid,
    output logic               eng_in_last,
    input  logic               eng_done
);

    typedef enum logic [2:0] {S_IDLE, S_LAUNCH, S_STREAM, S_DRAIN, S_READOUT} state_t;
    state_t state;

    // Job settings, captured at job_start
    logic [K_W-1:0] k_len;
    logic [M_W-1:0] m_r;
    logic [N_W-1:0] n_r;
    logic           acc_r;
    logic           emit_r;

    logic [K_W-1:0] k_cnt;      // address of the NEXT slice to read
    logic           last_r;     // the slice on the wire is the final one
    logic [M_W-1:0] row_cnt;    // row being sent in READOUT

    // A job_start is only acceptable with legal settings
    logic settings_ok;
    logic err_r;

    assign settings_ok = (cfg_k != '0) && (cfg_k <= K_DEPTH) &&
                         (cfg_m <= ROWS) && (cfg_n <= COLS);

    assign busy           = (state != S_IDLE);
    assign job_err        = err_r;
    assign job_done       = (state == S_DRAIN) && eng_done;

    assign res_valid      = (state == S_READOUT);
    assign res_row        = row_cnt;
    assign res_last       = res_valid && (row_cnt + 1'b1 == m_r);

    assign rd_addr        = k_cnt[ADDR_W-1:0];

    assign eng_start      = (state == S_LAUNCH);
    assign eng_accumulate = acc_r;
    assign eng_tile_m     = m_r;
    assign eng_tile_n     = n_r;
    assign eng_in_valid   = (state == S_STREAM);
    assign eng_in_last    = last_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            k_len   <= '0;
            k_cnt   <= '0;
            m_r     <= '0;
            n_r     <= '0;
            acc_r   <= 1'b0;
            emit_r  <= 1'b0;
            last_r  <= 1'b0;
            row_cnt <= '0;
            err_r   <= 1'b0;
        end else begin
            // Rejected: wrong settings, or a start while a job is running
            err_r <= job_start && ((state != S_IDLE) || !settings_ok);

            case (state)
                S_IDLE: if (job_start && settings_ok) begin
                    k_len  <= cfg_k;
                    m_r    <= cfg_m;
                    n_r    <= cfg_n;
                    acc_r  <= cfg_accumulate;
                    emit_r <= cfg_emit;
                    k_cnt  <= '0;
                    state  <= S_LAUNCH;
                end

                // Address 0 is on rd_addr right now, so slice 0 is on the
                // wire in the next cycle. Move on to address 1.
                S_LAUNCH: begin
                    last_r <= (k_len == 1);
                    k_cnt  <= 1;
                    state  <= S_STREAM;
                end

                S_STREAM: begin
                    if (last_r) begin
                        last_r <= 1'b0;
                        state  <= S_DRAIN;
                    end else begin
                        last_r <= (k_cnt + 1'b1 == k_len);
                        k_cnt  <= k_cnt + 1'b1;
                    end
                end

                // Engine finished: send the rows out, or go idle if the host
                // does not want them (K-chunk) or there are none (m = 0).
                S_DRAIN: if (eng_done) begin
                    row_cnt <= '0;
                    if (emit_r && m_r != '0) state <= S_READOUT;
                    else                     state <= S_IDLE;
                end

                S_READOUT: begin
                    if (res_last) state <= S_IDLE;
                    else          row_cnt <= row_cnt + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule