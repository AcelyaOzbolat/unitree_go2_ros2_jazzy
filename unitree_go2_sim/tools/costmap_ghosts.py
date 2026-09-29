#!/usr/bin/env python3
"""Count phantom obstacles: costmap cells marked lethal where the world is empty.

The global costmap is published in the map frame, which lines up with the Gazebo
world for this map, so every lethal cell can be checked against the real obstacle
geometry. A cell further than GHOST_DIST from anything real is a phantom.

Robot roll and pitch are sampled alongside, because the most likely source is the
laser slice tilting: /scan is cut in base_footprint, which base_to_footprint_ekf
keeps level using the IMU, and any lag there during a stride dips the slice into
the floor and paints a ring of floor points as obstacles.
"""
import math
import pathlib
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY, WALL_T  # noqa: E402

GHOST_DIST = 0.35          # further than this from anything real = phantom
# In the published OccupancyGrid nav2 maps costmap 254 -> 100 (a real obstacle)
# and 253 -> 99 (inscribed inflation). Counting 99 as an obstacle makes the normal
# inflation shell look like a field of phantoms - measured 1044 "ghosts" while the
# robot stood still, all of them 0.35-0.44 m out, i.e. exactly the shell.
LETHAL = 100               # only true obstacle cells
RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0

RECTS = [aabb(o) for o in OBSTACLES] + [
    (-RX-WALL_T, RY, RX+WALL_T, RY+WALL_T),
    (-RX-WALL_T, -RY-WALL_T, RX+WALL_T, -RY),
    (RX, -RY, RX+WALL_T, RY),
    (-RX-WALL_T, -RY, -RX, RY),
]


def dist_real(x, y):
    return min(math.hypot(max(x0-x, x-x1, 0.0), max(y0-y, y-y1, 0.0))
               for (x0, y0, x1, y1) in RECTS)


def rp(q):
    sr = 2*(q.w*q.x + q.y*q.z)
    cr = 1 - 2*(q.x*q.x + q.y*q.y)
    sp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
    return math.degrees(math.atan2(sr, cr)), math.degrees(math.asin(sp))


def main():
    rclpy.init()
    n = rclpy.create_node("costmap_ghosts")
    st = {}
    n.create_subscription(Odometry, "/odom/ground_truth",
                          lambda m: st.__setitem__("g", m), 10)
    q = QoSProfile(depth=1)
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    q.reliability = QoSReliabilityPolicy.RELIABLE
    n.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                          lambda m: st.__setitem__("c", m), q)

    t0 = time.time()
    while len(st) < 2 and time.time()-t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(st) < 2:
        print("costmap veya odometri gelmedi - yigin calisiyor mu?")
        return 2

    samples = []
    tilts = []
    seen = set()
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        rclpy.spin_once(n, timeout_sec=0.2)
        c = st["c"]
        key = (c.header.stamp.sec, c.header.stamp.nanosec)
        if key in seen:
            continue
        seen.add(key)

        i = c.info
        ghosts = []
        for idx, v in enumerate(c.data):
            if v < LETHAL:
                continue
            cx = i.origin.position.x + (idx % i.width + 0.5) * i.resolution
            cy = i.origin.position.y + (idx // i.width + 0.5) * i.resolution
            d = dist_real(cx, cy)
            if d > GHOST_DIST:
                ghosts.append((cx, cy, d))

        p = st["g"].pose.pose
        roll, pitch = rp(p.orientation)
        tilts.append(max(abs(roll), abs(pitch)))
        if ghosts:
            dists_to_robot = [math.hypot(gx-p.position.x, gy-p.position.y)
                              for gx, gy, _ in ghosts]
            samples.append((len(ghosts), min(dists_to_robot),
                            max(g[2] for g in ghosts), max(abs(roll), abs(pitch))))
        else:
            samples.append((0, None, 0.0, max(abs(roll), abs(pitch))))

    if not samples:
        print("costmap guncellemesi alinmadi")
        return 2

    counts = [s[0] for s in samples]
    withg = [s for s in samples if s[0] > 0]
    print(f"{len(samples)} costmap guncellemesi, {RUN_SECONDS:.0f} sn")
    print(f"hayalet hucre sayisi : ort {statistics.mean(counts):.1f}  "
          f"max {max(counts)}  medyan {statistics.median(counts):.0f}")
    print(f"hayaletli guncelleme : {len(withg)}/{len(samples)} "
          f"(%{100*len(withg)/len(samples):.0f})")
    if withg:
        near = [s[1] for s in withg if s[1] is not None]
        print(f"robota en yakin hayalet: medyan {statistics.median(near):.2f} m  "
              f"min {min(near):.2f} m")
        print(f"gercek yuzeye uzaklik  : max {max(s[2] for s in withg):.2f} m")
        tw = [s[3] for s in withg]
        tn = [s[3] for s in samples if s[0] == 0]
        print(f"govde egimi, hayaletliyken : ort {statistics.mean(tw):.1f} deg")
        if tn:
            print(f"govde egimi, temizken      : ort {statistics.mean(tn):.1f} deg")
    print(f"govde egimi genel      : ort {statistics.mean(tilts):.1f}  "
          f"max {max(tilts):.1f} deg")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
