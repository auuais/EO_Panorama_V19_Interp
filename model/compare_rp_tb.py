#!/usr/bin/env python3
"""Compare tb_RpRenderer output rasters with model/rpcore.py (EO and IR).

    python model/compare_rp_tb.py [--mark none|black|noise] [--interp-eo M]
                                  [--interp-ir M] [--grid-eo G] [--grid-ir G]
                                  [case ...]

Defaults match tb_RpRenderer's defaults.  --mark follows the bench's MARK
(RpRenderer OFFSET_CAM_MARK): what an EO camera carrying a roll/pitch offset
shows.  none and black are compared bit-exactly.  noise cannot be: the noise
comes from a free-running generator whose state depends on token timing, so
for noise the check is
  * every column outside the flagged cameras' strips (and their seams) is
    bit-exact -- the noise does not leak into other cameras
  * inside a flagged, present camera's body: pixels the model renders black
    (sample outside the source) stay black, every pixel is legal video
    (Y 16..235, C 16..240 -- 0x00/0xFF would read as SDI sync), and the mean
    |dY| against the clean render is large, i.e. the noise is really there
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rpcore as rc  # noqa: E402

MIX_R = [150, -300, 1000, -1000, 0, 777]
MIX_P = [-1000, 250, 0, 1000, -60, 5]
CASES = {
    "eo_zero": (rc.EO, [0] * 6, [0] * 6, (1,) * 6),
    "ir_zero": (rc.IR, [0] * 6, [0] * 6, (1,) * 6),
    "eo_mixed": (rc.EO, MIX_R, MIX_P, (1,) * 6),
    "ir_mixed": (rc.IR, MIX_R, MIX_P, (1,) * 6),
    "ir_alt10": (rc.IR, [1000, -1000] * 3, [1000, -1000] * 3, (1,) * 6),
    "ir_absent2": (rc.IR, [0] * 6, [0] * 6, (1, 1, 0, 1, 1, 1)),
    "ir_absent3": (rc.IR, MIX_R, MIX_P, (1, 1, 1, 0, 1, 1)),
    "eo_absent": (rc.EO, MIX_R, MIX_P, (1, 1, 1, 0, 1, 1)),
    # cameras 1, 3 and 5 carry an offset, 0, 2 and 4 do not
    "eo_partial": (rc.EO, [0, 600, 0, 0, 0, -250], [0, 0, 0, 300, 0, 0], (1,) * 6),
}
BLACK = rc.BLACK


def noise_check(s, got, clean, roll, pitch, present):
    """Checks for OFFSET_CAM_MARK 2 (noise).  Returns a list of problems."""
    got = got.reshape(clean.shape)
    flagged = [k for k in range(6) if roll[k] or pitch[k]]
    region = np.zeros(s.out_w, bool)
    for k in flagged:
        hi = s.cam_start[k + 1] + s.head_len[k] if k < 5 else s.out_w
        region[s.cam_start[k]:hi] = True
    problems = []
    out = ~region
    nbad = int((got[:, out] != clean[:, out]).sum())
    if nbad:
        problems.append(f"{nbad} mismatches outside the flagged cameras")
    for k in flagged:
        if not present[k]:
            continue
        lo = s.cam_start[k] + s.head_len[k - 1] if k > 0 else 0
        hi = s.cam_start[k] + s.tail_start[k] if k < 5 else s.out_w
        g, c = got[:, lo:hi], clean[:, lo:hi]
        blk = c == BLACK
        if (g[blk] != BLACK).any():
            problems.append(f"cam{k}: {int((g[blk] != BLACK).sum())} out-of-source pixels not black")
        y, cc = g >> 8, g & 0xFF
        if ((y < 16) | (y > 235) | (cc < 16) | (cc > 240)).any():
            problems.append(f"cam{k}: illegal levels")
        dy = np.abs((g[~blk] >> 8) - (c[~blk] >> 8)).mean() if (~blk).any() else 0.0
        if dy < 20:
            problems.append(f"cam{k}: mean |dY| {dy:.1f} -- no noise?")
        else:
            problems.append(f"(info) cam{k}: mean |dY| {dy:.1f}")
    return problems


def main():
    out = rc.ROOT / "sim_out"
    roms, frames = {}, {}
    ok = True
    args = sys.argv[1:]
    interp = {"eo": "vlerp", "ir": "vlerp"}  # tb_RpRenderer EO_INTERP / IR_INTERP
    grid = {"eo": 1, "ir": 1}                # tb_RpRenderer EO_GRID / IR_GRID
    mark = "noise"                           # tb_RpRenderer MARK 2
    while args[:1] and args[0].startswith("--"):
        if args[0] in ("--interp-eo", "--interp-ir"):
            interp[args[0][-2:]] = args[1]
        elif args[0] in ("--grid-eo", "--grid-ir"):
            grid[args[0][-2:]] = int(args[1])
        elif args[0] == "--mark":
            mark = args[1]
            assert mark in ("none", "black", "noise"), mark
        else:
            sys.exit(f"unknown option {args[0]}")
        args = args[2:]
    only = args
    for name, (s, r, p, present) in CASES.items():
        if only and name not in only:
            continue
        f = out / f"rp_{name}.hex"
        if not f.exists():
            print(f"{name}: no output file")
            ok = False
            continue
        if s.name not in roms:
            roms[s.name] = rc.load_rom(s)
            frames[s.name] = rc.coarsen(s, rc.synthetic_frames(s), grid["ir" if s.is_ir else "eo"])
        got = np.array([int(t, 16) for t in f.read_text().split()], np.int64)
        mode = interp["ir" if s.is_ir else "eo"]
        if mark == "noise" and not s.is_ir:
            clean = rc.render_offsets(s, roms[s.name], frames[s.name], r, p, present, mode, False)
            if got.size != clean.size:
                ok = False
                print(f"{name}: {got.size}/{clean.size} pixels")
                continue
            probs = noise_check(s, got, clean, r, p, present)
            errs = [x for x in probs if not x.startswith("(info)")]
            ok &= not errs
            print(f"{name}: {'noise checks pass' if not errs else 'FAIL'}; " + "; ".join(probs))
            continue
        exp = rc.render_offsets(s, roms[s.name], frames[s.name], r, p, present, mode,
                                mark == "black").reshape(-1)
        n = min(got.size, exp.size)
        bad = np.nonzero(got[:n] != exp[:n])[0]
        if got.size != exp.size or bad.size:
            ok = False
            print(f"{name}: {got.size}/{exp.size} pixels, {bad.size} mismatches",
                  [(int(i) // s.out_w, int(i) % s.out_w, f"{got[i]:04x}", f"{exp[i]:04x}") for i in bad[:6]])
        else:
            print(f"{name}: bit-exact ({got.size} pixels)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
