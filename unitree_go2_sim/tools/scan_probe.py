#!/usr/bin/env python3
"""Report what actually survives the pointcloud -> laserscan conversion."""
import math
import time

import rclpy
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = rclpy.create_node("scan_probe")
st = {"scan": None, "cloud": None, "map": None}
from rclpy.qos import qos_profile_sensor_data
node.create_subscription(LaserScan, "/scan", lambda m: st.__setitem__("scan", m), qos_profile_sensor_data)
node.create_subscription(PointCloud2, "/velodyne_points/points",
                         lambda m: st.__setitem__("cloud", m), 10)
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
mq = QoSProfile(depth=1)
mq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
mq.reliability = QoSReliabilityPolicy.RELIABLE
node.create_subscription(OccupancyGrid, "/map", lambda m: st.__setitem__("map", m), mq)

t0 = time.time()
while time.time() - t0 < 20 and not all(st.values()):
    rclpy.spin_once(node, timeout_sec=0.2)

c = st["cloud"]
if c is None:
    print("BULUT YOK")
else:
    print(f"bulut     : {c.width} x {c.height} nokta, frame={c.header.frame_id}")

s = st["scan"]
if s is None:
    print("SCAN YOK")
else:
    r = list(s.ranges)
    fin = [x for x in r if math.isfinite(x) and s.range_min <= x <= s.range_max]
    print(f"scan      : {len(r)} isin, frame={s.header.frame_id}")
    print(f"  gecerli : {len(fin)} (%{100*len(fin)/max(1,len(r)):.1f})")
    if fin:
        print(f"  mesafe  : min {min(fin):.2f} m / max {max(fin):.2f} m / ort {sum(fin)/len(fin):.2f} m")
    print(f"  limitler: range_min={s.range_min:.2f} range_max={s.range_max:.2f}")

m = st["map"]
if m is None:
    print("HARITA YOK")
else:
    i = m.info
    print(f"harita    : {i.width} x {i.height} hucre = "
          f"{i.width*i.resolution:.1f} x {i.height*i.resolution:.1f} m")
    known = sum(1 for v in m.data if v >= 0)
    occ = sum(1 for v in m.data if v > 50)
    print(f"  bilinen : {known} hucre, dolu {occ}")

rclpy.shutdown()
