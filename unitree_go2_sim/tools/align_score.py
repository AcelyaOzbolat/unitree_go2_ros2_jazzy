import math,re,sys
# Resolve the world generator relative to this file, so the checkout can live
# anywhere. tools/ -> unitree_go2_sim/ -> repo root -> unitree_go2_description/tools
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY, WALL_T
RECTS=[aabb(o) for o in OBSTACLES]+[
 (-RX-WALL_T,RY,RX+WALL_T,RY+WALL_T),(-RX-WALL_T,-RY-WALL_T,RX+WALL_T,-RY),
 (RX,-RY,RX+WALL_T,RY),(-RX-WALL_T,-RY,-RX,RY)]
def dw(x,y): return min(math.hypot(max(x0-x,x-x1,0.0),max(y0-y,y-y1,0.0)) for (x0,y0,x1,y1) in RECTS)
pgm,yml=sys.argv[1],sys.argv[2]
meta=open(yml).read()
res=float(re.search(r'resolution:\s*([\d.]+)',meta).group(1))
ox,oy=[float(v) for v in re.search(r'origin:\s*\[([-\d.eE]+),\s*([-\d.eE]+)',meta).groups()]
d=open(pgm,'rb').read()
m=re.match(rb'P5\s+(?:#[^\n]*\n)?\s*(\d+)\s+(\d+)\s+(\d+)\s',d)
w,h=int(m.group(1)),int(m.group(2)); px=d[m.end():]
pts=[]
for r in range(h):
    for c in range(w):
        if px[r*w+c]<100:
            pts.append((ox+(c+0.5)*res, oy+(h-1-r+0.5)*res))
print(f"dolu hucre: {len(pts)}")
best=None
for deg in range(-20,21,2):
    th=math.radians(deg); ct,stt=math.cos(th),math.sin(th)
    for dx in [i*0.1 for i in range(-8,9)]:
        for dy in [i*0.1 for i in range(-8,9)]:
            good=0
            for (x,y) in pts[::7]:                    # alt orneklem, hiz icin
                X=ct*x-stt*y+dx; Y=stt*x+ct*y+dy
                if dw(X,Y)<=2*res: good+=1
            tot=len(pts[::7])
            if best is None or good>best[0]: best=(good,tot,deg,dx,dy)
g,t,deg,dx,dy=best
print(f"en iyi hizalama: donus {deg:+d} deg, kayma ({dx:+.1f}, {dy:+.1f}) m")
print(f"  o hizalamada 2 hucre icinde: {g}/{t}  (%{100*g/t:.1f})")
