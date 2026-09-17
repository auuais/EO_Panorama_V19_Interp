`timescale 1ns/1ps
//
// IrV19DdrReplay in BEAT_OUT mode feeding RpLineRing (sensor_ir), against the
// late-ack arbiter model.  Same checks as tb_EoV19DdrReplayBeatOut, IR
// geometry: 20 beats of 32 luma pixels per 640-pixel row, no fetch window,
// rows stored as {Y, 0x80} ring words.
module tb_IrV19DdrReplayBeatOut;
    localparam integer LAT       = 24;
    localparam integer START_ROW = 25;       // odd: exercises the ROW_GRID start remap
    localparam integer ROW_GRID  = 2;
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
            for (k = 0; k < 32; k = k + 1)
                beat_for[k*8 +: 8] = a[7:0] ^ a[15:8] ^ a[23:16] ^ (k * 8'd37) ^ a[28:24];
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

    reg  [10:0] need_row = START_ROW;
    wire        beat_valid, row_done, frame_edge;
    wire [10:0] beat_row, row_done_row, dbg_row;
    wire [6:0]  beat_idx;
    wire [1535:0] beat_data;

    IrV19DdrReplay #(.BEAT_OUT(1), .BEAT_HOLD(7), .ROW_GRID(ROW_GRID)) dut (
        .rst_n(rst_n), .clk(clk), .ui_rst(ui_rst), .run_enable(run_enable),
        .beat_valid(beat_valid), .beat_row(beat_row), .beat_idx(beat_idx), .beat_data(beat_data),
        .row_done(row_done), .row_done_row(row_done_row),
        .lease_valid(1'b1),
        .bank0(2'd0), .bank1(2'd1), .bank2(2'd2), .bank3(2'd3), .bank4(2'd1), .bank5(2'd2),
        .source_need_valid(1'b1), .source_need_row(need_row),
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
        .clk(clk), .rst(ui_rst), .frame_start(frame_edge), .sensor_ir(1'b1),
        .beat_valid(beat_valid), .beat_row(beat_row), .beat_idx(beat_idx), .beat_data(beat_data),
        .row_done(row_done), .row_done_row(row_done_row), .rows_done(rows_done), .writer_idle(),
        .rd_word(rd_word), .rd_sel(rd_sel), .rd_cam(rd_cam), .rd_slot_a(slot_a), .rd_slot_b(slot_b),
        .rd_pa(pa), .rd_pb(pb));

    function [28:0] bank_off;
        input integer c;
        begin
            case (c)
                0: bank_off = 29'd0;        1: bank_off = 29'd81920;
                2: bank_off = 29'd163840;   3: bank_off = 29'd245760;
                4: bank_off = 29'd81920;    default: bank_off = 29'd163840;
            endcase
        end
    endfunction
    function [15:0] expect_px;
        input integer c, row, x;
        reg [255:0] b;
        begin
            b = beat_for(bank_off(c) + (row & RMASK) * 29'd160 + (x / 32) * 29'd8);
            expect_px = {b[(x % 32) * 8 +: 8], 8'h80};
        end
    endfunction

    integer errs = 0, beats = 0, exp_row, exp_beat, last_beat_t;
    reg pass_live = 1'b0;
    integer c;
    always @(posedge clk) begin
        if (frame_edge) begin
            exp_row = START_ROW; exp_beat = 0; last_beat_t = -100; pass_live = 1'b1;
        end
        if (!run_enable) pass_live = 1'b0;
        if (beat_valid && pass_live) begin
            beats = beats + 1;
            if ($time/10 - last_beat_t < 8) begin
                if (errs < 5) $display("  SPACING %0d cycles", $time/10 - last_beat_t);
                errs = errs + 1;
            end
            last_beat_t = $time/10;
            if (beat_row != exp_row || beat_idx != exp_beat) begin
                if (errs < 5) $display("  ORDER got row %0d beat %0d, expected row %0d beat %0d", beat_row, beat_idx, exp_row, exp_beat);
                errs = errs + 1;
            end
            for (c = 0; c < 6; c = c + 1)
                if (beat_data[c*256 +: 256] !== beat_for(bank_off(c) + (beat_row & RMASK) * 29'd160 + beat_idx * 29'd8)) begin
                    if (errs < 5) $display("  DATA cam%0d row %0d beat %0d", c, beat_row, beat_idx);
                    errs = errs + 1;
                end
            exp_beat = beat_idx + 1;
        end
        if (row_done && pass_live) begin
            if (row_done_row != exp_row || exp_beat != 20) begin
                if (errs < 5) $display("  ROW_DONE row %0d after beat %0d", row_done_row, exp_beat - 1);
                errs = errs + 1;
            end
            exp_row = exp_row + 1; exp_beat = 0;
        end
    end

    task ring_check;
        input integer row;
        integer k, x;
        begin
            slot_a = row % 3; slot_b = (row - 1) % 3;
            for (k = 0; k < 24; k = k + 1) begin
                x = ($random & 16'h7FFF) % 640;
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
        input integer nrows;
        integer r, guard;
        begin
            need_row = START_ROW;
            @(negedge clk) run_enable = 1'b1;
            for (r = START_ROW; r < START_ROW + nrows; r = r + 1) begin
                need_row = r;
                guard = 0;
                while (rows_done != r + 1) begin
                    @(negedge clk); guard = guard + 1;
                    if (guard > 200000) begin $display("FAIL: row %0d never completed", r); $finish; end
                end
                repeat (300) @(negedge clk);
                if (beat_valid || (dbg_row != r)) begin
                    $display("  HOLD violated at row %0d", r); errs = errs + 1;
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
            need_row = 11'd511;
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
        demand_pass(10);
        $display("pass 1: beats=%0d errs=%0d", beats, errs);
        abort_pass(3001); abort_pass(3007); abort_pass(5003);
        demand_pass(6);
        $display("after aborts: beats=%0d errs=%0d", beats, errs);
        if (errs == 0) $display("PASS tb_IrV19DdrReplayBeatOut");
        else           $display("FAIL tb_IrV19DdrReplayBeatOut: %0d errors", errs);
        $finish;
    end
endmodule
