`timescale 1ns/1ps
//
// EoV19DdrReplay in BEAT_OUT mode feeding RpLineRing, against the late-ack
// arbiter model of tb_EoV19DdrReplayArbiterAck.
//
// Checks, per pass:
//   * every beat of the dynamic fetch window arrives once, row-major, in beat
//     order, with the data of the address it names (camera, bank, row, beat)
//   * beats are at least 4 cycles apart (RpLineRing writes a beat as 4 words)
//   * row_done names each row after its last beat, and the replay holds at
//     the end of a row until source_need_row allows the next one
//   * the ring's rows_done follows, and pixels read back from the two rows
//     the renderer would read (rows r and r-1) match the fetched data
//   * a pass aborted at arbitrary points leaves the next pass clean
module tb_EoV19DdrReplayBeatOut;
    localparam integer LAT       = 24;
    localparam integer START_ROW = 42;       // not on the ROW_GRID, so the start remap is exercised
    localparam integer ROW_GRID  = 4;
    localparam [10:0]  RMASK     = ~(ROW_GRID - 1);

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0, ui_rst = 1'b1, run_enable = 1'b0;

    wire        req_valid;  wire [28:0] req_addr;
    reg         data_valid = 1'b0;  reg [383:0] data;
    reg         arb_busy = 1'b0;
    reg [28:0]  arb_addr = 29'd0;
    reg [3:0]   arb_wait = 4'd0;
    reg [15:0]  lfsr = 16'hBEEF;
    wire        req_ready = arb_busy && (arb_wait == 4'd0);
    reg [28:0]  pipe_addr [0:LAT-1];
    reg         pipe_v    [0:LAT-1];

    function [255:0] beat_for;
        input [28:0] a;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1)
                beat_for[k*16 +: 16] = {a[7:0] ^ a[23:16], a[15:8]} + (k * 16'h0101) + a[28:24];
        end
    endfunction

    integer i;
    always @(posedge clk) begin
        lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
        if (ui_rst) begin
            arb_busy <= 1'b0;
            for (i = 0; i < LAT; i = i + 1) pipe_v[i] <= 1'b0;
            data_valid <= 1'b0;
        end else begin
            if (!arb_busy && req_valid) begin
                arb_busy <= 1'b1; arb_addr <= req_addr; arb_wait <= {2'b0, lfsr[1:0]};
            end else if (arb_busy) begin
                if (arb_wait != 4'd0) arb_wait <= arb_wait - 4'd1;
                else arb_busy <= 1'b0;
            end
            data_valid <= pipe_v[0];
            data       <= {128'd0, beat_for(pipe_addr[0])};
            for (i = 0; i < LAT-1; i = i + 1) begin
                pipe_v[i] <= pipe_v[i+1]; pipe_addr[i] <= pipe_addr[i+1];
            end
            pipe_v[LAT-1]    <= arb_busy && (arb_wait == 4'd0);
            pipe_addr[LAT-1] <= arb_addr;
        end
    end

    reg  [6:0]  fetch_lo = 7'd16, fetch_hi = 7'd96;
    reg         need_valid = 1'b1;
    reg  [10:0] need_row = START_ROW;
    wire        beat_valid, row_done, frame_edge;
    wire [10:0] beat_row, row_done_row, dbg_row;
    wire [6:0]  beat_idx;
    wire [1535:0] beat_data;

    EoV19DdrReplay #(.BEAT_OUT(1), .BEAT_HOLD(2), .DYN_FETCH(1), .ROW_GRID(ROW_GRID)) dut (
        .rst_n(rst_n), .clk(clk), .ui_rst(ui_rst), .run_enable(run_enable),
        .fetch_beat_lo(fetch_lo), .fetch_beat_hi(fetch_hi),
        .beat_valid(beat_valid), .beat_row(beat_row), .beat_idx(beat_idx), .beat_data(beat_data),
        .row_done(row_done), .row_done_row(row_done_row),
        .lease_valid(1'b1),
        .bank0(2'd0), .bank1(2'd1), .bank2(2'd2), .bank3(2'd3), .bank4(2'd1), .bank5(2'd2),
        .source_need_valid(need_valid), .source_need_row(need_row),
        .source_start_row(START_ROW[10:0]),
        .rd_req_valid(req_valid), .rd_req_addr(req_addr), .rd_req_ready(req_ready),
        .rd_data_valid(data_valid), .rd_data(data),
        .replay_clk(),
        .replay_hsync0(), .replay_vsync0(), .replay_pixel0(),
        .replay_hsync1(), .replay_vsync1(), .replay_pixel1(),
        .replay_hsync2(), .replay_vsync2(), .replay_pixel2(),
        .replay_hsync3(), .replay_vsync3(), .replay_pixel3(),
        .replay_hsync4(), .replay_vsync4(), .replay_pixel4(),
        .replay_hsync5(), .replay_vsync5(), .replay_pixel5(),
        .frame_edge(frame_edge), .dbg_row(dbg_row), .dbg_state(), .dbg_word(), .banks_ready());

    reg  [8:0]  rd_word = 9'd0;
    reg  [1:0]  rd_sel = 2'd0, slot_a = 2'd0, slot_b = 2'd0;
    reg  [2:0]  rd_cam = 3'd0;
    wire [15:0] pa, pb;
    wire [11:0] rows_done;
    RpLineRing ring (
        .clk(clk), .rst(ui_rst), .frame_start(frame_edge),
        .beat_valid(beat_valid), .beat_row(beat_row), .beat_idx(beat_idx), .beat_data(beat_data),
        .row_done(row_done), .row_done_row(row_done_row), .rows_done(rows_done), .writer_idle(),
        .rd_word(rd_word), .rd_sel(rd_sel), .rd_cam(rd_cam), .rd_slot_a(slot_a), .rd_slot_b(slot_b),
        .rd_pa(pa), .rd_pb(pb));

    function [28:0] bank_off;
        input integer c;
        begin
            case (c)
                0: bank_off = 29'd0;         1: bank_off = 29'd1036800;
                2: bank_off = 29'd2073600;   3: bank_off = 29'd3110400;
                4: bank_off = 29'd1036800;   default: bank_off = 29'd2073600;
            endcase
        end
    endfunction
    function [15:0] expect_px;
        input integer c, row, x;
        reg [28:0] a;
        reg [255:0] b;
        begin
            a = bank_off(c) + (row & RMASK) * 29'd960 + (x / 16) * 29'd8;
            b = beat_for(a);
            expect_px = b[(x % 16) * 16 +: 16];
        end
    endfunction

    // ---- beat stream checker ---------------------------------------------
    integer errs = 0, beats = 0, exp_row, exp_beat, last_beat_t, rows_ok;
    reg pass_live = 1'b0;
    integer c, t;
    always @(posedge clk) begin
        if (frame_edge) begin
            exp_row = START_ROW; exp_beat = {fetch_lo[6:3], 3'b000}; last_beat_t = -100;
            pass_live = 1'b1;
        end
        if (!run_enable) pass_live = 1'b0;
        if (beat_valid && pass_live) begin
            beats = beats + 1;
            if ($time/10 - last_beat_t < 4) begin
                if (errs < 5) $display("  SPACING %0d cycles", $time/10 - last_beat_t);
                errs = errs + 1;
            end
            last_beat_t = $time/10;
            if (beat_row != exp_row || beat_idx != exp_beat) begin
                if (errs < 5) $display("  ORDER got row %0d beat %0d, expected row %0d beat %0d", beat_row, beat_idx, exp_row, exp_beat);
                errs = errs + 1;
            end
            for (c = 0; c < 6; c = c + 1)
                if (beat_data[c*256 +: 256] !== beat_for(bank_off(c) + (beat_row & RMASK) * 29'd960 + beat_idx * 29'd8)) begin
                    if (errs < 5) $display("  DATA cam%0d row %0d beat %0d", c, beat_row, beat_idx);
                    errs = errs + 1;
                end
            exp_beat = beat_idx + 1;
        end
        if (row_done && pass_live) begin
            if (row_done_row != exp_row || exp_beat != fetch_hi + 8) begin
                if (errs < 5) $display("  ROW_DONE row %0d after beat %0d, expected row %0d after beat %0d",
                                       row_done_row, exp_beat - 1, exp_row, fetch_hi + 7);
                errs = errs + 1;
            end
            if (row_done_row >= need_row + 1) begin
                if (errs < 5) $display("  DEMAND row %0d completed with need_row %0d", row_done_row, need_row);
                errs = errs + 1;
            end
            exp_row = exp_row + 1; exp_beat = {fetch_lo[6:3], 3'b000};
        end
    end

    task ring_check;
        input integer row;
        integer k, x, got_a, got_b;
        begin
            slot_a = row % 3; slot_b = (row - 1) % 3;
            for (k = 0; k < 24; k = k + 1) begin
                x = ({fetch_lo[6:3], 3'b000} * 16) + ($random & 16'h7FFF) % ((fetch_hi + 8 - {fetch_lo[6:3], 3'b000}) * 16);
                @(negedge clk);
                rd_cam = k % 6; rd_word = x >> 2; rd_sel = x & 3;
                @(negedge clk); @(negedge clk); @(negedge clk); @(negedge clk); @(negedge clk);
                if (pa !== expect_px(k % 6, row, x) || pb !== expect_px(k % 6, row - 1, x)) begin
                    if (errs < 5) $display("  RING cam%0d x %0d rows %0d/%0d: got %04x/%04x want %04x/%04x",
                                           k % 6, x, row, row - 1, pa, pb, expect_px(k % 6, row, x), expect_px(k % 6, row - 1, x));
                    errs = errs + 1;
                end
            end
        end
    endtask

    task demand_pass;
        input [6:0] lo, hi;
        input integer nrows;
        integer r, guard;
        begin
            fetch_lo = lo; fetch_hi = hi;
            need_row = START_ROW;
            @(negedge clk) run_enable = 1'b1;
            for (r = START_ROW; r < START_ROW + nrows; r = r + 1) begin
                need_row = r;
                guard = 0;
                while (rows_done != r + 1) begin
                    @(negedge clk); guard = guard + 1;
                    if (guard > 200000) begin $display("FAIL: row %0d never completed", r); $finish; end
                end
                // held: nothing for row r+1 may start while need_row == r
                repeat (300) @(negedge clk);
                if (beat_valid || (dbg_row != r)) begin
                    $display("  HOLD violated at row %0d (dbg_row %0d)", r, dbg_row); errs = errs + 1;
                end
                if (r > START_ROW) ring_check(r);
            end
            @(negedge clk) run_enable = 1'b0;
            repeat (200) @(negedge clk);
        end
    endtask

    task abort_pass;
        input integer cy;
        begin
            need_row = 11'd1079;
            @(negedge clk) run_enable = 1'b1;
            repeat (cy) @(negedge clk);
            run_enable = 1'b0;
            repeat (60) @(negedge clk);
        end
    endtask

    initial begin
        repeat (10) @(posedge clk);
        ui_rst = 0; rst_n = 1;
        repeat (5) @(posedge clk);
        demand_pass(7'd16, 7'd96, 12);
        $display("window 16..96: beats=%0d errs=%0d", beats, errs);
        demand_pass(7'd8, 7'd104, 12);
        $display("window 8..104: beats=%0d errs=%0d", beats, errs);
        abort_pass(4001); abort_pass(4007); abort_pass(4013); abort_pass(9999);
        demand_pass(7'd0, 7'd112, 8);
        $display("after aborts, window 0..112: beats=%0d errs=%0d", beats, errs);
        if (errs == 0) $display("PASS tb_EoV19DdrReplayBeatOut");
        else           $display("FAIL tb_EoV19DdrReplayBeatOut: %0d errors", errs);
        $finish;
    end
endmodule
