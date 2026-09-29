#!/usr/bin/env python3
"""Score a saved map against the world it was built from.

The robot spawns at the world origin facing +x, so slam_toolbox's map frame lines
up with world coordinates and occupied cells can be compared directly against the
real obstacle geometry. Reports how far each occupied cell sits from the nearest
real surface - a good map has almost all of them within a cell or two.

That distance alone only catches one kind of error. It asks "is what I drew
really a wall", and says nothing about walls that went missing, so a map with a
hole ray-traced straight through it still scores well. The second half of the
report covers the other direction: how much of the real surface got drawn at
all, and whether free space leaked outside the room, which is what a wall the
scan failed to report looks like in the saved grid.
"""
import math
import re
import pathlib
import sys

# Resolve the world generator relative to this file, so the checkout can live
# anywhere. tools/ -> unitree_go2_sim/ -> repo root -> unitree_go2_description/tools
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY, WALL_T  # noqa: E402

RECTS = [aabb(o) for o in OBSTACLES] + [
    (-RX-WALL_T, RY, RX+WALL_T, RY+WALL_T),      # north wall
    (-RX-WALL_T, -RY-WALL_T, RX+WALL_T, -RY),    # south wall
    (RX, -RY, RX+WALL_T, RY),                    # east wall
    (-RX-WALL_T, -RY, -RX, RY),                  # west wall
]


def dist_to_world(x, y):
    return min(math.hypot(max(x0-x, x-x1, 0.0), max(y0-y, y-y1, 0.0))
               for (x0, y0, x1, y1) in RECTS)


SAMPLE_STEP = 0.05    # spacing along a real surface when checking coverage
COVER_TOL = 0.15      # an occupied cell this close counts as covering the sample


def surface_samples():
    """Points along every real surface the lidar can actually see.

    Only the inner faces of the room walls are included. The outside of the room
    is unreachable, so scoring it would count a miss that no run could ever fix.
    """
    pts = []
    for o in OBSTACLES:
        x0, y0, x1, y1 = aabb(o)
        n = max(1, int((x1 - x0) / SAMPLE_STEP))
        for i in range(n + 1):
            x = x0 + (x1 - x0) * i / n
            pts += [(x, y0), (x, y1)]
        n = max(1, int((y1 - y0) / SAMPLE_STEP))
        for i in range(n + 1):
            y = y0 + (y1 - y0) * i / n
            pts += [(x0, y), (x1, y)]
    n = max(1, int(2 * RX / SAMPLE_STEP))
    for i in range(n + 1):
        x = -RX + 2 * RX * i / n
        pts += [(x, RY), (x, -RY)]
    n = max(1, int(2 * RY / SAMPLE_STEP))
    for i in range(n + 1):
        y = -RY + 2 * RY * i / n
        pts += [(RX, y), (-RX, y)]
    return pts


def main(pgm, yml):
    meta = open(yml).read()
    res = float(re.search(r'resolution:\s*([\d.]+)', meta).group(1))
    ox, oy = [float(v) for v in
              re.search(r'origin:\s*\[([-\d.eE]+),\s*([-\d.eE]+)', meta).groups()]

    d = open(pgm, 'rb').read()
    m = re.match(rb'P5\s+(?:#[^\n]*\n)?\s*(\d+)\s+(\d+)\s+(\d+)\s', d)
    w, h = int(m.group(1)), int(m.group(2))
    px = d[m.end():]

    errs = []
    occupied = set()
    leaked = []
    out_x, out_y = RX + WALL_T, RY + WALL_T
    for row in range(h):
        for col in range(w):
            v = px[row*w + col]
            # PGM rows run top-down; map rows run bottom-up
            x = ox + (col + 0.5) * res
            y = oy + (h - 1 - row + 0.5) * res
            if v < 100:
                errs.append(dist_to_world(x, y))
                occupied.add((col, h - 1 - row))
            elif v > 250 and (abs(x) > out_x or abs(y) > out_y):
                # Free space where no robot has ever been. Rays went through a
                # wall the scan did not report - the classic symptom of walking
                # closer to it than range_min.
                leaked.append((x, y))

    if not errs:
        print("dolu hucre yok"); return
    errs.sort()
    n = len(errs)
    def pct(p): return errs[min(n-1, int(n*p))]
    print(f"dolu hucre sayisi : {n}")
    print(f"gercek yuzeye uzaklik:")
    print(f"  medyan          : {pct(0.50):.3f} m")
    print(f"  %75             : {pct(0.75):.3f} m")
    print(f"  %90             : {pct(0.90):.3f} m")
    print(f"  en kotu         : {errs[-1]:.3f} m")
    good = sum(1 for e in errs if e <= 2*res)
    print(f"  2 hucre icinde  : {good}/{n}  (%{100*good/n:.1f})")
    print()
    if good/n > 0.85:
        print(">>> HARITA IYI: dolu hucrelerin buyuk cogunlugu gercek yuzeylerde")
    elif good/n > 0.6:
        print(">>> HARITA ORTA: gozle gorulur bulanikilik var")
    else:
        print(">>> HARITA KOTU: dolu hucreler gercek geometriyle ortusmuyor")

    # --- the other direction: what the map is missing ------------------------
    reach = int(math.ceil(COVER_TOL / res))
    samples = surface_samples()
    holes = []
    for (x, y) in samples:
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        if not any((c + dc, r + dr) in occupied
                   for dc in range(-reach, reach + 1)
                   for dr in range(-reach, reach + 1)):
            holes.append((x, y))
    covered = len(samples) - len(holes)
    print()
    print(f"yuzey kapsamasi   : {covered}/{len(samples)}  "
          f"(%{100*covered/len(samples):.1f})")
    if holes:
        xs = [p[0] for p in holes]
        ys = [p[1] for p in holes]
        print(f"  cizilmemis yuzey: x {min(xs):+.2f}..{max(xs):+.2f}  "
              f"y {min(ys):+.2f}..{max(ys):+.2f}")

    print(f"oda disi bos hucre: {len(leaked)}", end="")
    if leaked:
        xs = [p[0] for p in leaked]
        ys = [p[1] for p in leaked]
        print(f"  ({len(leaked)*res*res:.2f} m2, "
              f"x {min(xs):+.2f}..{max(xs):+.2f}  y {min(ys):+.2f}..{max(ys):+.2f})")
        print(">>> SIZINTI VAR: bir duvarin arkasi bos isaretlenmis. Robot o "
              "duvara range_min'den (0.5 m) yakin gecmis olabilir.")
    else:
        print("  - sizinti yok")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
