// operand_buffer.sv
// Small synchronous RAM that holds k-slices for the matrix engine.
//
// One word is one k-slice of one operand:
//   A buffer: WIDTH = ROWS * DATA_WIDTH  (one column of A)
//   B buffer: WIDTH = COLS * DATA_WIDTH  (one row of B)
// so a single read hands the engine exactly the slice it wants.
//
// Behaviour
//   - One write port (wr_en / wr_addr / wr_data), one read port (rd_addr).
//   - The read is registered: rd_data shows mem[rd_addr] ONE cycle after
//     rd_addr was applied. The controller relies on this fixed latency.
//   - Read-during-write to the same address returns the OLD word (read-first);
//     the new word is visible from the next cycle on.
//   - Addresses >= DEPTH (only possible when DEPTH is not a power of two):
//     writes are ignored and reads return 0.
//   - No reset and no initial contents (that is what lets FPGA tools map it to
//     block RAM). Words must be written before they are read.
module operand_buffer #(
    parameter WIDTH = 32,
    parameter DEPTH = 16,
    localparam ADDR_W = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    input  logic               clk,

    input  logic               wr_en,
    input  logic [ADDR_W-1:0]  wr_addr,
    input  logic [WIDTH-1:0]   wr_data,

    input  logic [ADDR_W-1:0]  rd_addr,
    output logic [WIDTH-1:0]   rd_data
);

    logic [WIDTH-1:0] mem [DEPTH];

    always_ff @(posedge clk) begin
        if (wr_en && (wr_addr < DEPTH))
            mem[wr_addr] <= wr_data;

        rd_data <= (rd_addr < DEPTH) ? mem[rd_addr] : '0;
    end

endmodule