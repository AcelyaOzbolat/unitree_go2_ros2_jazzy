#!/usr/bin/env python3
"""Compare the published /scan against a ray-cast of the known world.

Uses Gazebo ground truth for the robot pose, so any disagreement is the scan
pipeline's fault, not SLAM's. This separates "the sensor data is wrong" from
"SLAM is mishandling good data".
"""
import math
import pathlib
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

# Resolve the world generator relative to this file, so the checkout can live
# anywhere. tools/ -> unitree_go2_sim/ -> repo root -> unitree_go2_description/tools
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY, WALL_T  # noqa: E402

RECTS = [aabb(o) for o in OBSTACLES] + [
    (-RX-WALL_T, RY, RX+WALL_T, RY+WALL_T),
    (-RX-WALL_T, -RY-WALL_T, RX+WALL_T, -RY),
    (RX, -RY, RX+WALL_T, RY),
    (-RX-WALL_T, -RY, -RX, RY),
]


def ray_rect(px, py, dx, dy, r):
    """Slab method; returns entry distance or None."""
    x0, y0, x1, y1 = r
    tmin, tmax = 0.0, 1e9
    for p, d, lo, hi in ((px, dx, x0, x1), (py, dy, y0, y1)):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return None
        else:
            t1, t2 = (lo-p)/d, (hi-p)/d
            if t1 > t2: t1, t2 = t2, t1
            tmin = max(tmin, t1); tmax = min(tmax, t2)
            if tmin > tmax: return None
    return tmin if tmax >= 0 else None


def expected(px, py, ang):
    dx, dy = math.cos(ang), math.sin(ang)
    best = None
    for r in RECTS:
        t = ray_rect(px, py, dx, dy, r)
        if t is not None and t > 1e-6 and (best is None or t < best):
            best = t
    return best


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def main():
    rclpy.init()
    n = rclpy.create_node("check_scan")
    st = {}
    n.create_subscription(LaserScan, "/scan", lambda m: st.__setitem__("s", m),
                          qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom/ground_truth",
                          lambda m: st.__setitem__("g", m), 10)
    t0 = time.time()
    while len(st) < 2 and time.time()-t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(st) < 2:
        print("veri gelmedi"); return

    s, g = st["s"], st["g"]
    p = g.pose.pose.position
    th = yaw(g.pose.pose.orientation)
    print(f"gercek poz: x={p.x:.2f} y={p.y:.2f} yaw={math.degrees(th):+.1f} deg")

    errs = []
    used = 0
    for i, rng in enumerate(s.ranges):
        if not math.isfinite(rng) or rng < s.range_min or rng > s.range_max:
            continue
        a = th + s.angle_min + i*s.angle_increment
        e = expected(p.x, p.y, a)
        if e is None or e > 15:
            continue
        errs.append(rng - e)
        used += 1

    if not errs:
        print("karsilastirilacak isin yok"); return
    a = sorted(abs(e) for e in errs)
    nn = len(a)
    mean_signed = sum(errs)/len(errs)
    print(f"karsilastirilan isin : {used}")
    print(f"  ortalama isaretli sapma : {mean_signed:+.3f} m")
    print(f"  medyan |hata|           : {a[nn//2]:.3f} m")
    print(f"  %90 |hata|              : {a[int(nn*0.9)]:.3f} m")
    good = sum(1 for x in a if x <= 0.10)
    print(f"  10 cm icinde            : {good}/{nn} (%{100*good/nn:.1f})")
    print()
    # Three tiers, and the median leads. A hard pass/fail at 90 % sits right in the
    # run-to-run noise band - the same healthy scan measured 92.4 % and 89.9 % on two
    # consecutive runs - and would call a 2 cm-median scan broken. A handful of rays
    # always miss at obstacle silhouette edges, where the beam grazes past a near
    # object and the surface behind it falls outside the height band.
    med = a[nn//2]
    if med <= 0.05 and good/nn > 0.85:
        print(">>> TARAMA DOGRU - sorun SLAM tarafinda")
    elif med <= 0.15:
        print(">>> TARAMA SINIRDA - buyuk olcude dogru, kenarlarda kayip isinlar var")
    else:
        print(">>> TARAMA HATALI - sorun scan uretiminde")
        print("    ipucu: medyan metre mertebesindeyse once base_footprint -> base_link")
        print("    donusumunun yaw'ina bak; tarama o zincirle tasiniyor.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
