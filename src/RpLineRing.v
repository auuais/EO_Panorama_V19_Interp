//============================================================================
// RpLineRing
//
// The source-driven renderer's line cache: three source rows per camera,
// slot = row mod 3.  While output pixels for source row y0 are rendered, rows
// y0 and y0+1 are read and row y0+2 is written into the third slot, so the
// DDR replay and the renderer run concurrently without ever touching the same
// slot.  The renderer only moves to y0+1 after its pipeline has drained,
// which is what frees the y0 slot for row y0+3.
//
// Write side: the DDR replay in BEAT_OUT mode presents one 256-bit beat per
// camera with its row and beat index, then holds while this module writes it
// into a 480-word row of 64-bit words, 4 x 16-bit {Y, C} pixels each:
//   EO (EoV19DdrReplay)  16 pixels, pixel p in [p*16 +: 16]    -> 4 words
//   IR (IrV19DdrReplay)  32 luma pixels, pixel p in [p*8 +: 8]  -> 8 words,
//                        stored as {Y, 8'h80} so the read side is identical  Beats outside the
// replay's fetch window are never presented; their words keep old contents,
// which the renderer never addresses because the fetch window is derived from
// the same tables that bound every sampled column.
//
// Read side: one shared word address, two slots (rows y0 and y0+1), one
// camera.  Latency RD_LAT = 4 from rd_* to rd_pa/rd_pb.
//
// EO_GRID / IR_GRID (power of two, <= 16 / 32): interpolation experiments.
// Every pixel is stored as a copy of the first pixel of its GRID-wide block,
// i.e. a source with GRID x fewer columns.  EO keeps 4:2:2 phase: luma from
// the block's first pixel, Cb/Cr from its first pair.  A block never leaves
// its beat, so the fetch window is unaffected.  1 = normal.  (The replay's
// ROW_GRID does the same for rows.)
//============================================================================
`timescale 1ns/1ps
module RpLineRing #(
    parameter integer EO_GRID = 1,
    parameter integer IR_GRID = 1
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          frame_start,      // replay frame_edge: forget all rows
    input  wire          sensor_ir,        // static while a pass runs

    input  wire          beat_valid,
    input  wire [10:0]   beat_row,
    input  wire [6:0]    beat_idx,
    input  wire [1535:0] beat_data,        // cam k in [k*256 +: 256]
    input  wire          row_done,         // replay finished row_done_row
    input  wire [10:0]   row_done_row,
    output reg  [11:0]   rows_done,        // last completed row + 1 (0 = none)
    output wire          writer_idle,

    input  wire [8:0]    rd_word,
    input  wire [1:0]    rd_sel,
    input  wire [2:0]    rd_cam,
    input  wire [1:0]    rd_slot_a,
    input  wire [1:0]    rd_slot_b,
    output reg  [15:0]   rd_pa,
    output reg  [15:0]   rd_pb
);
    localparam integer WORDS = 480;

    //------------------------------------------------------------------------
    // Writer: one beat -> four words, all six cameras in parallel
    //------------------------------------------------------------------------
    reg [1535:0] wbeat;
    wire [1535:0] beat_eo_g, beat_ir_g;
    genvar bc, bp;
    generate
        for (bc = 0; bc < 6; bc = bc + 1) begin : g_grid_cam
            for (bp = 0; bp < 16; bp = bp + 1) begin : g_eo
                assign beat_eo_g[bc*256 + bp*16 + 8 +: 8] = beat_data[bc*256 + (bp & ~(EO_GRID-1))*16 + 8 +: 8];
                assign beat_eo_g[bc*256 + bp*16     +: 8] = beat_data[bc*256 + ((bp & ~(EO_GRID-1)) | (bp & 1))*16 +: 8];
            end
            for (bp = 0; bp < 32; bp = bp + 1) begin : g_ir
                assign beat_ir_g[bc*256 + bp*8 +: 8] = beat_data[bc*256 + (bp & ~(IR_GRID-1))*8 +: 8];
            end
        end
    endgenerate
    reg [8:0]    wbase;          // beat_idx * 4
    reg [1:0]    wslot;
    reg [3:0]    wleft;          // words still to write for the latched beat
    reg [2:0]    wk;             // word within the beat
    reg          row_done_pend;
    reg [10:0]   row_done_pend_row;
    assign writer_idle = (wleft == 4'd0);

    function [1:0] mod3;
        input [10:0] v;
        begin
            mod3 = v % 3;
        end
    endfunction

    always @(posedge clk) begin
        if (rst || frame_start) begin
            wleft <= 4'd0; wk <= 3'd0;
            rows_done <= 12'd0;
            row_done_pend <= 1'b0;
        end else begin
            if (beat_valid) begin
                wbeat <= sensor_ir ? beat_ir_g : beat_eo_g;
                wbase <= sensor_ir ? {beat_idx[5:0], 3'b000} : {beat_idx, 2'b00};
                wslot <= mod3(beat_row);
                wleft <= sensor_ir ? 4'd8 : 4'd4;
                wk    <= 3'd0;
            end else if (wleft != 4'd0) begin
                wleft <= wleft - 4'd1;
                wk    <= wk + 3'd1;
            end
            if (row_done) begin
                row_done_pend     <= 1'b1;
                row_done_pend_row <= row_done_row;
            end else if (row_done_pend && writer_idle && !beat_valid) begin
                row_done_pend <= 1'b0;
                rows_done     <= {1'b0, row_done_pend_row} + 12'd1;
            end
        end
    end

    wire        we_any = (wleft != 4'd0);
    wire [8:0]  waddr  = wbase + {6'd0, wk};

    //------------------------------------------------------------------------
    // 18 row memories (6 cams x 3 slots), 480 x 64 bits each
    //------------------------------------------------------------------------
    reg  [8:0]  rd_word_q;
    reg  [1:0]  sel_d1, sel_d2;
    reg  [2:0]  cam_d1, cam_d2;
    reg  [1:0]  sa_d1, sa_d2, sb_d1, sb_d2;
    wire [63:0] q [0:17];

    genvar gc, gs;
    generate
        for (gc = 0; gc < 6; gc = gc + 1) begin : g_cam
            for (gs = 0; gs < 3; gs = gs + 1) begin : g_slot
                (* ram_style = "block" *) reg [63:0] mem [0:511];
                reg [63:0] q1, q2;
                // IR word k: luma pixels 4k..4k+3, each widened to {Y, 8'h80}.
                wire [63:0] word_eo = wbeat[gc*256 + wk*64 +: 64];
                wire [63:0] word_ir = {wbeat[gc*256 + (wk*4+3)*8 +: 8], 8'h80,
                                       wbeat[gc*256 + (wk*4+2)*8 +: 8], 8'h80,
                                       wbeat[gc*256 + (wk*4+1)*8 +: 8], 8'h80,
                                       wbeat[gc*256 + (wk*4)*8   +: 8], 8'h80};
                always @(posedge clk) begin
                    if (we_any && (wslot == gs))
                        mem[waddr] <= sensor_ir ? word_ir : word_eo;
                end
                always @(posedge clk) begin
                    q1 <= mem[rd_word_q];
                    q2 <= q1;
                end
                assign q[gc*3 + gs] = q2;
            end
        end
    endgenerate

    // latency: rd_* -> rd_word_q (1) -> q1 (2) -> q2 (3) -> rd_pa/rd_pb (4)
    reg [63:0] wa, wb;
    always @(posedge clk) begin
        rd_word_q <= rd_word;
        sel_d1 <= rd_sel;    sel_d2 <= sel_d1;
        cam_d1 <= rd_cam;    cam_d2 <= cam_d1;
        sa_d1  <= rd_slot_a; sa_d2  <= sa_d1;
        sb_d1  <= rd_slot_b; sb_d2  <= sb_d1;
    end
    // q2 is valid for the address issued three cycles earlier; cam/slot/sel
    // are delayed to match and the mux+select is the fourth register.
    reg [1:0] sel_d3; reg [2:0] cam_d3; reg [1:0] sa_d3, sb_d3;
    always @(posedge clk) begin
        sel_d3 <= sel_d2; cam_d3 <= cam_d2; sa_d3 <= sa_d2; sb_d3 <= sb_d2;
    end
    wire [63:0] word_a = q[cam_d3*3 + sa_d3];
    wire [63:0] word_b = q[cam_d3*3 + sb_d3];
    always @(posedge clk) begin
        rd_pa <= word_a[sel_d3*16 +: 16];
        rd_pb <= word_b[sel_d3*16 +: 16];
    end
endmodule
