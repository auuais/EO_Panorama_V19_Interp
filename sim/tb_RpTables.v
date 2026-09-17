`timescale 1ns/1ps
// RpTables against model/golden.py build_tables() for four offset cases.
// Vectors: python model/gen_rp_roms.py  ->  sim/vectors/rp_tables_<case>/
module tb_RpTables;
    parameter ROOT = "E:/Xylinx/EO_Panorama_V19_Interp";

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst = 1'b1;

    // Segment ROMs, same contents and latency as EoV19RunRom / IrV19RunRom.
    reg [143:0] rom [0:31679];
    reg [95:0]  rom_ir [0:28799];
    wire [14:0] rom_addr;
    reg  [143:0] rom_data;
    reg  [95:0]  rom_data_ir;
    always @(posedge clk) begin
        rom_data    <= rom[rom_addr];
        rom_data_ir <= rom_ir[rom_addr];
    end
    reg sensor_ir = 1'b0;
    wire built_ir;

    reg         build_req = 1'b0;
    reg  [95:0] roll_cd = 96'd0, pitch_cd = 96'd0;
    wire        busy, tables_valid, sat_seen;
    wire [31:0] build_count;
    wire [10:0] ylo, yhi, col_lo, col_hi;
    reg  [14:0] params_raddr = 15'd0;
    wire [143:0] params_rdata;
    reg  [10:0] bend_raddr = 11'd0;
    wire [15:0] bend_rdata;
    reg  [8:0]  rready_raddr = 9'd0;
    wire [10:0] rready_rdata;

    RpTables #(.ANGLE_ROM_FILE({ROOT, "/assets/rollpitch/eo_rp_angle.mem"}),
               .IR_ANGLE_ROM_FILE({ROOT, "/assets/rollpitch/ir_rp_angle.mem"})) dut (
        .clk(clk), .rst(rst), .sensor_ir(sensor_ir), .rom_data_ir(rom_data_ir), .built_ir(built_ir),
        .build_req(build_req), .roll_cd(roll_cd), .pitch_cd(pitch_cd),
        .busy(busy), .tables_valid(tables_valid), .sat_seen(sat_seen), .build_count(build_count),
        .ylo(ylo), .yhi(yhi), .col_lo(col_lo), .col_hi(col_hi),
        .rom_addr(rom_addr), .rom_data(rom_data),
        .params_raddr(params_raddr), .params_rdata(params_rdata),
        .bend_raddr(bend_raddr), .bend_rdata(bend_rdata),
        .rready_raddr(rready_raddr), .rready_rdata(rready_rdata));

    reg [143:0] exp_params [0:31679];
    reg [15:0]  exp_bend [0:1078];
    integer nseg, nbucket;
    reg [10:0]  exp_rr [0:479];
    reg [15:0]  exp_off [0:11];
    reg [15:0]  exp_meta [0:3];

    integer errors = 0;
    integer k, cycles;

    task run_case;
        input [8*16-1:0] name;
        input            ir;
        reg [8*256-1:0] dir;
        integer case_err;
        begin
            sensor_ir = ir;
            nseg    = ir ? 28800 : 31680;
            nbucket = ir ? 511 : 1079;
            dir = {ROOT, "/sim/vectors/rp_tables_", (ir ? "ir_" : ""), name};
            $readmemh({dir, "/params.mem"}, exp_params);
            $readmemh({dir, "/bucket_end.mem"}, exp_bend);
            $readmemh({dir, "/row_ready.mem"}, exp_rr);
            $readmemh({dir, "/offsets.mem"}, exp_off);
            $readmemh({dir, "/meta.mem"}, exp_meta);
            for (k = 0; k < 6; k = k + 1) begin
                roll_cd[k*16 +: 16]  = exp_off[k];
                pitch_cd[k*16 +: 16] = exp_off[6 + k];
            end
            case_err = 0;
            @(negedge clk) build_req = 1'b1;
            @(negedge clk) build_req = 1'b0;
            cycles = 0;
            while (!tables_valid) begin
                @(negedge clk); cycles = cycles + 1;
                if (cycles > 3000000) begin
                    $display("FAIL %0s: build did not finish", name); $finish;
                end
            end
            if (ylo !== exp_meta[0][10:0] || yhi !== exp_meta[1][10:0] ||
                col_lo !== exp_meta[2][10:0] || col_hi !== exp_meta[3][10:0]) begin
                $display("FAIL %0s meta: Y=%0d..%0d cols=%0d..%0d expected Y=%0d..%0d cols=%0d..%0d",
                         name, ylo, yhi, col_lo, col_hi, exp_meta[0], exp_meta[1], exp_meta[2], exp_meta[3]);
                case_err = case_err + 1;
            end
            if (sat_seen) begin
                $display("FAIL %0s: saturation flagged", name); case_err = case_err + 1;
            end
            if (built_ir !== ir) begin
                $display("FAIL %0s: built_ir=%b", name, built_ir); case_err = case_err + 1;
            end
            for (k = 0; k < nbucket; k = k + 1) begin
                @(negedge clk) bend_raddr = k;
                @(negedge clk);
                if (bend_rdata !== exp_bend[k]) begin
                    if (case_err < 10)
                        $display("FAIL %0s bucket_end[%0d]=%0d expected %0d", name, k, bend_rdata, exp_bend[k]);
                    case_err = case_err + 1;
                end
            end
            for (k = 0; k < 480; k = k + 1) begin
                @(negedge clk) rready_raddr = k;
                @(negedge clk);
                if (rready_rdata !== exp_rr[k]) begin
                    if (case_err < 10)
                        $display("FAIL %0s row_ready[%0d]=%0d expected %0d", name, k, rready_rdata, exp_rr[k]);
                    case_err = case_err + 1;
                end
            end
            for (k = 0; k < nseg; k = k + 1) begin
                @(negedge clk) params_raddr = k;
                @(negedge clk); @(negedge clk);
                if (params_rdata !== exp_params[k]) begin
                    if (case_err < 10)
                        $display("FAIL %0s params[%0d]=%h expected %h", name, k, params_rdata, exp_params[k]);
                    case_err = case_err + 1;
                end
            end
            $display("%0s%0s: build %0d cycles, Y=%0d..%0d cols=%0d..%0d, %0d mismatches",
                     ir ? "ir_" : "eo_", name, cycles, ylo, yhi, col_lo, col_hi, case_err);
            errors = errors + case_err;
        end
    endtask

    initial begin
        $readmemh({ROOT, "/assets/rowruns/eo_v19_render_runs.mem"}, rom);
        $readmemh({ROOT, "/assets/rowruns/ir_v19_render_runs.mem"}, rom_ir);
        repeat (5) @(negedge clk);
        rst = 1'b0;
        run_case("zero", 1'b0);
        run_case("mixed", 1'b0);
        run_case("zero", 1'b1);
        run_case("mixed", 1'b1);
        run_case("roll10", 1'b0);
        run_case("roll10", 1'b1);
        run_case("alt10", 1'b0);
        run_case("alt10", 1'b1);
        if (errors == 0) $display("PASS tb_RpTables");
        else             $display("FAIL tb_RpTables: %0d mismatches", errors);
        $finish;
    end
endmodule
