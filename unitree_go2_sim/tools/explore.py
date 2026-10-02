#!/usr/bin/env python3
"""Drive the room for mapping without ever touching an obstacle.

Walks forward while there is room ahead, and turns away as soon as the clearance
in front drops below SAFE. Clearance is read from the world definition rather
than from sensors, so a bump is a bug in this script, not bad luck.
"""
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Resolve the world generator relative to this file, so the checkout can live
# anywhere. tools/ -> unitree_go2_sim/ -> repo root -> unitree_go2_description/tools
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY  # noqa: E402

SAFE = 1.0        # stop approaching once this close to anything
LOOK = 0.8        # look this far ahead of the body
RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0


def clearance(x, y):
    best = min(math.hypot(max(x0-x, x-x1, 0.0), max(y0-y, y-y1, 0.0))
               for o in OBSTACLES for (x0, y0, x1, y1) in [aabb(o)])
    return min(best, RX-abs(x), RY-abs(y))


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def main():
    rclpy.init()
    n = rclpy.create_node("explore")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    st = {"g": None}
    n.create_subscription(Odometry, "/odom/ground_truth",
                          lambda m: st.__setitem__("g", m), 10)
    t0 = time.time()
    while st["g"] is None and time.time()-t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if st["g"] is None:
        print("no odometry"); return

    def pose():
        p = st["g"].pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    start = time.time()
    turning = 0.0
    bumped = False
    min_seen = 99.0
    msg = Twist()
    while time.time() - start < RUN_SECONDS:
        x, y, th = pose()
        here = clearance(x, y)
        min_seen = min(min_seen, here)
        if here < 0.35:
            bumped = True
        ahead = clearance(x + LOOK*math.cos(th), y + LOOK*math.sin(th))

        if turning > 0:
            msg.linear.x = 0.0
            msg.angular.z = 0.30
            turning -= 0.05
        elif ahead < SAFE:
            # rotate away; direction chosen by whichever side is more open
            lx, ly = x + LOOK*math.cos(th+1.2), y + LOOK*math.sin(th+1.2)
            rx, ry = x + LOOK*math.cos(th-1.2), y + LOOK*math.sin(th-1.2)
            turning = 4.0
            msg.linear.x = 0.0
            msg.angular.z = 0.30 if clearance(lx, ly) > clearance(rx, ry) else -0.30
        else:
            msg.linear.x = 0.15
            msg.angular.z = 0.0
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.05)

    pub.publish(Twist())
    x, y, _ = pose()
    q = st["g"].pose.pose.orientation
    roll = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
    print(f"done: x={x:.2f} y={y:.2f}  roll={roll:+.0f} "
          f"({'FELL OVER' if abs(roll) > 60 else 'upright'})")
    print(f"smallest clearance seen: {min_seen:.2f} m  "
          f"({'HIT SOMETHING' if bumped else 'no contact'})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
