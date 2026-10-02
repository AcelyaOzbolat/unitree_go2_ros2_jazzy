#!/usr/bin/env python3
"""Measure how far the leg odometry drifts from Gazebo ground truth.

Obstacle-aware: the straight leg stops as soon as the robot gets within STOP_DIST
of anything. An earlier version drove blind, walked into the loop block, and the
resulting "36 % odometry error" was really just the robot being stuck against a
wall while the odometry kept counting.
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

STOP_DIST = 0.9   # keep the body well clear; the Go2 is 0.7 m long


def clearance(x, y):
    best = min(math.hypot(max(x0-x, x-x1, 0.0), max(y0-y, y-y1, 0.0))
               for o in OBSTACLES for (x0, y0, x1, y1) in [aabb(o)])
    return min(best, RX-abs(x), RY-abs(y))


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def dyaw(a, b):
    d = b - a
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return math.degrees(d)


def main():
    rclpy.init()
    n = rclpy.create_node("odom_quality")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    st = {}
    n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("o", m), 10)
    n.create_subscription(Odometry, "/odom/ground_truth", lambda m: st.__setitem__("g", m), 10)
    t0 = time.time()
    while len(st) < 2 and time.time()-t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(st) < 2:
        print("no odometry received"); return

    def snap():
        o, g = st["o"].pose.pose, st["g"].pose.pose
        return ((o.position.x, o.position.y, yaw(o.orientation)),
                (g.position.x, g.position.y, yaw(g.orientation)))

    def settle(secs=3):
        end = time.time() + secs
        while time.time() < end:
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)

    p = st["g"].pose.pose.position
    print(f"starting clearance: {clearance(p.x, p.y):.2f} m")

    # --- straight leg, aborted before touching anything -------------------
    settle()
    o0, g0 = snap()
    msg = Twist(); msg.linear.x = 0.15
    end = time.time() + 30
    stopped = "time limit"
    while time.time() < end:
        pub.publish(msg); rclpy.spin_once(n, timeout_sec=0.05)
        p = st["g"].pose.pose.position
        if clearance(p.x, p.y) < STOP_DIST:
            stopped = "obstacle ahead"; break
    settle()
    o1, g1 = snap()
    od = math.hypot(o1[0]-o0[0], o1[1]-o0[1])
    gd = math.hypot(g1[0]-g0[0], g1[1]-g0[1])
    print("--- 1) walking straight ---")
    print(f"  stopped because: {stopped}")
    print(f"  odometry : {od:.3f} m")
    print(f"  truth    : {gd:.3f} m")
    if gd > 0.2:
        print(f"  ERROR    : {abs(od-gd):.3f} m  ({100*abs(od-gd)/gd:.1f} %)")
    else:
        print("  (moved too little for a percentage to mean anything)")

    # --- turn in place --------------------------------------------------
    # Yaw is accumulated step by step rather than compared end to end: a 20 s
    # turn covers ~344 deg, which wraps past +/-180 and made the previous
    # endpoint comparison meaningless.
    settle()
    o_prev, g_prev = snap()
    o_acc = g_acc = 0.0
    msg = Twist(); msg.angular.z = 0.30
    end = time.time() + 20
    while time.time() < end:
        pub.publish(msg); rclpy.spin_once(n, timeout_sec=0.05)
        o_now, g_now = snap()
        o_acc += dyaw(o_prev[2], o_now[2])
        g_acc += dyaw(g_prev[2], g_now[2])
        o_prev, g_prev = o_now, g_now
    settle()
    print("--- 2) turning in place (accumulated) ---")
    print(f"  odometry : {o_acc:+.1f} deg")
    print(f"  truth    : {g_acc:+.1f} deg")
    print(f"  ERROR    : {abs(o_acc-g_acc):.1f} deg  ({100*abs(o_acc-g_acc)/max(1,abs(g_acc)):.1f} %)")
    p = st["g"].pose.pose.position
    print(f"  final clearance: {clearance(p.x, p.y):.2f} m")

    pub.publish(Twist())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
