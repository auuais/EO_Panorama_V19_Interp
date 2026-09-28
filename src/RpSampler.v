//============================================================================
// RpSampler
//
// Walker token -> sampled pixel -> RowWindow write.  II=1, no back-pressure.
//
//   valid  = 0 <= ax <= (SRC_W-2)<<16  and  0 <= ay <= (SRC_H-2)<<16
//   rows   = the two line-ring slots for y0 and y0+1 (the walker only issues
//            a valid pixel at the y0 it samples)
//   out    = valid ? {Y, C} : BLACK
//
// EO_INTERP / IR_INTERP select how {Y, C} is taken from the source, per
// sensor (the two panoramas share this sampler but never run together):
//   0  none (nearest neighbour): x = round(ax), row = round(ay), i.e. y0+1
//      when ay[15] is set; one source pixel, no arithmetic.  EO x is still
//      phase-locked to pano_x parity so Cb/Cr alternate correctly.
//   1  M1: x = floor(ax) (EO phase-locked), Y and C interpolated between y0
//      and y0+1 with ay[15:0] (EO interp8 rounded, IR truncating vlerp).
// Both datapaths are built when the two sensors differ; the mode is picked
// from sensor_ir, which is static while a pass runs.
//
// mark_mask[cam] (EO only) marks that camera's pixels, per MARK_MODE:
//   2  noise: luma +-128 and chroma +-64 scaled by NOISE_GAIN_HALVES/2
//      (3: +-192 / +-96) from a free-running xorshift32, so it changes every
//      frame, clamped to legal video (Y 16..235, C 16..240) because 0x00/0xFF
//      in active video would read as SDI sync words
//   3  grey: chroma replaced by neutral 0x80, luma untouched, so the tile is
//      the same picture in monochrome
// Out-of-source (black) pixels stay black either way -- BLACK is already
// {0x10, 0x80}, so grey leaves it alone.
//
// The RowWindow address is {sy mod 256, strip_x >> 2}; pixel strip_x & 3 of
// the 72-bit word lives at [sel*18 +: 16] and is written with byte enables,
// so no read-modify-write is needed.
//
// Latency: tok -> rw write strobe registered 7 cycles later.
//============================================================================
`timescale 1ns/1ps
module RpSampler #(
    parameter integer SRC_W = 1920,
    parameter integer SRC_H = 1080,
    parameter [15:0]  BLACK = 16'h1080,
    parameter integer EO_INTERP = 0,       // 0 nearest, 1 M1 vertical bilinear
    parameter integer IR_INTERP = 0,
    parameter integer MARK_MODE = 0,           // 2 noise, 3 grey (see header)
    parameter integer NOISE_GAIN_HALVES = 3,   // mark_mask noise amplitude x N/2, N = 1..8
    parameter integer CAM1_START = 633,  parameter integer CAM2_START = 1271,
    parameter integer CAM3_START = 1909, parameter integer CAM4_START = 2547,
    parameter integer CAM5_START = 3185,
    parameter integer STRIP1 = 650,  parameter integer STRIP2 = 1305,
    parameter integer STRIP3 = 1960, parameter integer STRIP4 = 2615,
    parameter integer STRIP5 = 3270,
    // IR geometry
    parameter integer IR_SRC_W = 640,
    parameter integer IR_SRC_H = 512,
    parameter integer IR_STRIP1 = 616,  parameter integer IR_STRIP2 = 1237,
    parameter integer IR_STRIP3 = 1877, parameter integer IR_STRIP4 = 2498,
    parameter integer IR_STRIP5 = 3119
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               sensor_ir,
    input  wire [5:0]         mark_mask,   // per camera, EO; see header

    input  wire               tok_valid,
    input  wire [2:0]         tok_cam,
    input  wire [8:0]         tok_sy,
    input  wire [9:0]         tok_lx,
    input  wire signed [31:0] tok_ax,
    input  wire signed [31:0] tok_ay,

    input  wire [10:0]        y0,          // for the debug row check only

    // line ring
    output reg  [8:0]         ring_word,
    output reg  [1:0]         ring_sel,
    output reg  [2:0]         ring_cam,
    input  wire [15:0]        ring_pa,     // latency 4 from ring_*
    input  wire [15:0]        ring_pb,

    // RowWindow port A
    output reg  [7:0]         rw_we,
    output reg  [17:0]        rw_addr,
    output reg  [71:0]        rw_din,

    output wire               busy,        // any token in flight
    output reg  [31:0]        row_mismatch_count
);
    // EO: RowWindow word = 4 x {2'b0, Y, C}; x phase-locked to pano_x parity.
    // IR: RowWindow word = 8 x {1'b0, Y}; C is 0x80.
    // Sampling per EO_INTERP / IR_INTERP (header).
    wire use_interp = sensor_ir ? (IR_INTERP != 0) : (EO_INTERP != 0);
    wire rnd = ~use_interp;                // round x instead of floor
    wire signed [31:0] XMAXQ = sensor_ir ? ((IR_SRC_W - 2) <<< 16) : ((SRC_W - 2) <<< 16);
    wire signed [31:0] YMAXQ = sensor_ir ? ((IR_SRC_H - 2) <<< 16) : ((SRC_H - 2) <<< 16);

    function [11:0] cam_start;
        input [2:0] cam;
        begin
            case (cam)
                3'd0: cam_start = 12'd0;               3'd1: cam_start = CAM1_START;
                3'd2: cam_start = CAM2_START;          3'd3: cam_start = CAM3_START;
                3'd4: cam_start = CAM4_START;          default: cam_start = CAM5_START;
            endcase
        end
    endfunction
    function [11:0] strip_base;
        input [2:0] cam;
        input       ir;
        begin
            if (ir)
                case (cam)
                    3'd0: strip_base = 12'd0;     3'd1: strip_base = IR_STRIP1;
                    3'd2: strip_base = IR_STRIP2; 3'd3: strip_base = IR_STRIP3;
                    3'd4: strip_base = IR_STRIP4; default: strip_base = IR_STRIP5;
                endcase
            else
                case (cam)
                    3'd0: strip_base = 12'd0;   3'd1: strip_base = STRIP1;
                    3'd2: strip_base = STRIP2;  3'd3: strip_base = STRIP3;
                    3'd4: strip_base = STRIP4;  default: strip_base = STRIP5;
                endcase
        end
    endfunction
    function [10:0] qx_phase;
        input signed [31:0] c;
        input               parity;
        input               rnd_in;
        reg [10:0] q;
        begin
            // c <= 1918<<16 keeps the rounded q <= 1918 too.
            if (c < 0)                  q = 11'd0;
            else if (c > (1918 <<< 16)) q = 11'd1918;
            else                        q = c[26:16] + (rnd_in & c[15]);
            if (q == 0)            qx_phase = parity ? 11'd1 : 11'd0;
            else if (q >= 11'd1918) qx_phase = parity ? 11'd1919 : 11'd1918;
            else                   qx_phase = {q[10:1], parity};
        end
    endfunction

    // ---- S1: decode -------------------------------------------------------
    reg        v1, ok1, nz1;
    reg [15:0] fr1;
    reg [8:0]  slot1;
    reg [11:0] strip1;
    always @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else     v1 <= tok_valid;
        ok1    <= (tok_ax >= 0) && (tok_ax <= XMAXQ) &&
                  (tok_ay >= 0) && (tok_ay <= YMAXQ);
        fr1    <= tok_ay[15:0];
        nz1    <= mark_mask[tok_cam] && !sensor_ir;
        slot1  <= tok_sy;
        strip1 <= strip_base(tok_cam, sensor_ir) + {2'd0, tok_lx};
        begin : rd
            reg [11:0] px;
            reg [10:0] x;
            px = cam_start(tok_cam) + {2'd0, tok_lx};
            // IR: floor (or round).  Out-of-range samples are written black,
            // so the clamp only has to keep the ring address in range.
            if (sensor_ir)
                x = tok_ax[31] ? 11'd0 :
                    (tok_ax[26:16] > (IR_SRC_W - 2)) ? (IR_SRC_W - 2) : tok_ax[26:16] + (rnd & tok_ax[15]);
            else
                x = qx_phase(tok_ax, px[0], rnd);
            ring_word <= x[10:2];
            ring_sel  <= x[1:0];
            ring_cam  <= tok_cam;
        end
        // A valid pixel must be sampled at the y0 being rendered.
        if (rst) row_mismatch_count <= 32'd0;
        else if (tok_valid && (tok_ax >= 0) && (tok_ax <= XMAXQ) &&
                 (tok_ay >= 0) && (tok_ay <= YMAXQ) && (tok_ay[26:16] != y0))
            row_mismatch_count <= row_mismatch_count + 32'd1;
    end

    // ---- S2..S5: wait for the ring (latency 4) ----------------------------
    reg [4:1]  vd;
    reg [3:0]  okd, nzd;
    reg [15:0] frd [1:4];
    reg [8:0]  slotd [1:4];
    reg [11:0] stripd [1:4];
    integer k;
    always @(posedge clk) begin
        if (rst) vd <= 4'd0;
        else     vd <= {vd[3:1], v1};
        okd[0] <= ok1; nzd[0] <= nz1; frd[1] <= fr1; slotd[1] <= slot1; stripd[1] <= strip1;
        for (k = 2; k <= 4; k = k + 1) begin
            okd[k-1] <= okd[k-2]; nzd[k-1] <= nzd[k-2];
            frd[k] <= frd[k-1]; slotd[k] <= slotd[k-1]; stripd[k] <= stripd[k-1];
        end
    end
    // vd[4], okd[3], frd[4], slotd[4], stripd[4] are aligned with ring_pa/pb.

    // ---- S6: sample ---------------------------------------------------------
    function [32:0] wacc;
        input [7:0] a, b;
        input [15:0] f;
        begin
            wacc = ({1'b0, a} * (17'd65536 - {1'b0, f})) + ({1'b0, b} * {1'b0, f}) + 33'd32768;
        end
    endfunction
    function [7:0] vlerp;
        input [7:0] p0, p1;
        input [15:0] f;
        reg signed [8:0]  d;
        reg signed [24:0] m;
        begin
            d = $signed({1'b0, p1}) - $signed({1'b0, p0});
            m = d * $signed({1'b0, f});
            vlerp = p0 + m[24:16];
        end
    endfunction
    reg        v6, ok6, nz6;
    reg [15:0] near6;                      // no interpolation: the nearest row's {Y, C}
    reg [32:0] accy6, accc6;               // interpolated
    reg        ui6;                        // use_interp, aligned with the data
    reg [7:0]  ir6;
    reg [8:0]  slot6;
    reg [11:0] strip6;
    always @(posedge clk) begin
        if (rst) v6 <= 1'b0;
        else     v6 <= vd[4];
        ok6    <= okd[3];
        nz6    <= nzd[3];
        near6  <= frd[4][15] ? ring_pb : ring_pa;
        ui6    <= use_interp;
        accy6  <= wacc(ring_pa[15:8], ring_pb[15:8], frd[4]);
        accc6  <= wacc(ring_pa[7:0],  ring_pb[7:0],  frd[4]);
        ir6    <= vlerp(ring_pa[15:8], ring_pb[15:8], frd[4]);
        slot6  <= slotd[4];
        strip6 <= stripd[4];
    end

    // ---- offset-camera mark (mark_mask) -----------------------------------
    // Free running and never reset (pass_rst included), so every frame gets
    // new noise: live static over the image, not a frozen pattern.
    reg  [31:0] ns = 32'h2545F491;
    wire [31:0] ns_a = ns ^ (ns << 13);
    wire [31:0] ns_b = ns_a ^ (ns_a >> 17);
    always @(posedge clk) ns <= ns_b ^ (ns_b << 5);
    wire signed [11:0] n_y = $signed({{4{ns[7]}}, ns[7:0]});        // -128..127
    wire signed [11:0] n_c = $signed({{5{ns[15]}}, ns[15:9]});      // -64..63
    // x NOISE_GAIN_HALVES / 2 (a constant multiply: shifts and adds)
    wire signed [11:0] noise_y = (n_y * NOISE_GAIN_HALVES) >>> 1;
    wire signed [11:0] noise_c = (n_c * NOISE_GAIN_HALVES) >>> 1;
    function [7:0] add_clamp;
        input [7:0]         v;
        input signed [11:0] n;
        input [7:0]         lo, hi;
        reg signed [13:0]   t;
        begin
            t = $signed({6'd0, v}) + n;
            if (t < $signed({6'd0, lo}))      add_clamp = lo;
            else if (t > $signed({6'd0, hi})) add_clamp = hi;
            else                              add_clamp = t[7:0];
        end
    endfunction
    wire [15:0] eo_pix6   = !ui6 ? near6 : {accy6[23:16], accc6[23:16]};
    wire [15:0] eo_noisy6 = {add_clamp(eo_pix6[15:8], noise_y, 8'd16, 8'd235),
                             add_clamp(eo_pix6[7:0],  noise_c, 8'd16, 8'd240)};
    wire [15:0] eo_grey6  = {eo_pix6[15:8], 8'h80};
    wire [15:0] eo_mark6  = (MARK_MODE == 3) ? eo_grey6 : eo_noisy6;

    // ---- S7: write --------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            rw_we <= 8'd0;
        end else begin
            rw_we <= 8'd0;
            if (v6 && sensor_ir) begin
                rw_we[strip6[2:0]] <= 1'b1;
            end else if (v6) begin
                rw_we[{strip6[1:0], 1'b0}] <= 1'b1;
                rw_we[{strip6[1:0], 1'b1}] <= 1'b1;
            end
        end
        if (sensor_ir) begin
            // 512 rows x 512 words of 8 one-byte luma pixels.
            rw_addr <= {slot6[8:0], strip6[11:3]};
            rw_din  <= {8{1'b0, (!ok6 ? BLACK[15:8] : !ui6 ? near6[15:8] : ir6)}};
        end else begin
            rw_addr <= {slot6[7:0], strip6[11:2]};
            rw_din  <= {4{2'b00, (!ok6 ? BLACK : nz6 ? eo_mark6 : eo_pix6)}};
        end
    end

    assign busy = v1 | (|vd) | v6 | (|rw_we);
endmodule
