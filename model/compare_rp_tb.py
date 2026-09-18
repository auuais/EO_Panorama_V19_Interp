#!/usr/bin/env python3
"""Compare tb_RpRenderer output rasters with model/rpcore.py (EO and IR)."""
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
}


def main():
    out = rc.ROOT / "sim_out"
    roms, frames = {}, {}
    ok = True
    args = sys.argv[1:]
    interp = "none"                      # tb_RpRenderer INTERP 0
    grid = {"eo": 16, "ir": 8}           # tb_RpRenderer EO_GRID / IR_GRID
    blank = True                         # tb_RpRenderer BLANK 1
    while args[:1] and args[0].startswith("--"):
        if args[0] == "--no-blank":          # a flag, not a key/value pair
            blank = False
            args = args[1:]
            continue
        if args[0] == "--interp":
            interp = args[1]
        elif args[0] in ("--grid-eo", "--grid-ir"):
            grid[args[0][-2:]] = int(args[1])
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
        exp = rc.render_offsets(s, roms[s.name], frames[s.name], r, p, present, interp,
                                blank).reshape(-1)
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
