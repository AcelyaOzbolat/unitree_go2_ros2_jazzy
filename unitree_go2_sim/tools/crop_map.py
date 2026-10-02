"""Trim fully-unknown borders from a saved map and fix the yaml origin to match.

Cropping pixels without moving the origin would silently shift the map in world
coordinates, so the two must change together.
"""
import re, sys
src_pgm, src_yaml, out_base = sys.argv[1], sys.argv[2], sys.argv[3]
meta = open(src_yaml).read()
res = float(re.search(r'resolution:\s*([\d.]+)', meta).group(1))
ox, oy = [float(v) for v in re.search(r'origin:\s*\[([-\d.eE]+),\s*([-\d.eE]+)', meta).groups()]
d = open(src_pgm,'rb').read()
m = re.match(rb'P5\s+(?:#[^\n]*\n)?\s*(\d+)\s+(\d+)\s+(\d+)\s', d)
w, h, mx = int(m.group(1)), int(m.group(2)), int(m.group(3))
px = d[m.end():]
known = lambda v: v < 150 or v > 240
cols = [c for c in range(w) if any(known(px[r*w+c]) for r in range(h))]
rows = [r for r in range(h) if any(known(px[r*w+c]) for c in range(w))]
PAD = 4
c0, c1 = max(0, min(cols)-PAD), min(w-1, max(cols)+PAD)
r0, r1 = max(0, min(rows)-PAD), min(h-1, max(rows)+PAD)
nw, nh = c1-c0+1, r1-r0+1
out = bytearray()
for r in range(r0, r1+1):
    out += px[r*w+c0 : r*w+c1+1]
open(f"{out_base}.pgm","wb").write(b"P5\n%d %d\n%d\n" % (nw,nh,mx) + bytes(out))
# x grows right, y grows UP while pgm rows go down: the new bottom row is r1
nox = ox + c0*res
noy = oy + (h-1-r1)*res
open(f"{out_base}.yaml","w").write(
    f"image: {out_base.split('/')[-1]}.pgm\nmode: trinary\nresolution: {res:.3f}\n"
    f"origin: [{nox:.3f}, {noy:.3f}, 0]\nnegate: 0\n"
    f"occupied_thresh: 0.65\nfree_thresh: 0.196\n")
print(f"{w}x{h} -> {nw}x{nh} px  ({nw*res:.1f} x {nh*res:.1f} m, ratio {nw/nh:.2f})")
print(f"origin ({ox:.3f}, {oy:.3f}) -> ({nox:.3f}, {noy:.3f})")
