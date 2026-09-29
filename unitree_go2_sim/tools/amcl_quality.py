#!/usr/bin/env python3
"""Measure how well AMCL knows where the robot is.

Drives a short obstacle-aware course and compares AMCL's estimate against Gazebo
ground truth throughout. Reports both the position error and how tight the
particle cloud is - a confident-but-wrong filter and an uncertain one need
different fixes, and the covariance is what separates them.

The map frame is assumed to line up with the Gazebo world, which holds when the
map was built by slam_toolbox from a robot spawned at the world origin facing +x
(verified for simple_room: best alignment was 0 deg, 0 m offset).
"""
import math
import pathlib
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry

# Resolve the world generator relative to this file, so the checkout can live
# anywhere. tools/ -> unitree_go2_sim/ -> repo root -> unitree_go2_description/tools
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY  # noqa: E402

SAFE = 1.0
LOOK = 0.8
RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0


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
    n = rclpy.create_node("amcl_quality")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    init_pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    st = {}
    n.create_subscription(Odometry, "/odom/ground_truth",
                          lambda m: st.__setitem__("g", m), 10)
    n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                          lambda m: st.__setitem__("a", m), 10)

    t0 = time.time()
    while "g" not in st and time.time()-t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if "g" not in st:
        print("ground truth odometri yok - simulasyon calisiyor mu?")
        return 2

    # Seed AMCL from the true pose. In RViz this is the "2D Pose Estimate" button;
    # doing it here keeps the run reproducible.
    g = st["g"].pose.pose
    seed = PoseWithCovarianceStamped()
    seed.header.frame_id = "map"
    seed.header.stamp = n.get_clock().now().to_msg()
    seed.pose.pose = g
    seed.pose.covariance[0] = 0.25
    seed.pose.covariance[7] = 0.25
    seed.pose.covariance[35] = 0.07

    # Wait for AMCL's subscription to match before sending, then keep resending
    # until it answers. A single publish straight after creating the publisher is
    # dropped - DDS discovery has not finished - and AMCL then sits there logging
    # "Please set the initial pose" forever.
    t0 = time.time()
    while init_pub.get_subscription_count() == 0 and time.time()-t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    if init_pub.get_subscription_count() == 0:
        print("/initialpose'u dinleyen yok - amcl calisiyor mu?")
        return 2

    print(f"baslangic tahmini veriliyor: x={g.position.x:.2f} y={g.position.y:.2f} "
          f"yaw={math.degrees(yaw(g.orientation)):+.1f} deg")
    t0 = time.time()
    while "a" not in st and time.time()-t0 < 40:
        seed.header.stamp = n.get_clock().now().to_msg()
        init_pub.publish(seed)
        for _ in range(10):
            rclpy.spin_once(n, timeout_sec=0.05)
            if "a" in st:
                break
    if "a" not in st:
        print("/amcl_pose gelmedi - amcl aktif mi? (lifecycle_manager loguna bak)")
        return 2
    print(f"amcl yanit verdi ({time.time()-t0:.1f} sn sonra)")

    pos_err, yaw_err, spread = [], [], []
    turning = 0.0
    msg = Twist()
    start = time.time()
    while time.time() - start < RUN_SECONDS:
        gp = st["g"].pose.pose
        x, y, th = gp.position.x, gp.position.y, yaw(gp.orientation)
        ahead = clearance(x + LOOK*math.cos(th), y + LOOK*math.sin(th))

        if turning > 0:
            msg.linear.x, msg.angular.z = 0.0, 0.25
            turning -= 0.05
        elif ahead < SAFE:
            lx, ly = x + LOOK*math.cos(th+1.2), y + LOOK*math.sin(th+1.2)
            rx, ry = x + LOOK*math.cos(th-1.2), y + LOOK*math.sin(th-1.2)
            turning = 4.0
            msg.linear.x = 0.0
            msg.angular.z = 0.25 if clearance(lx, ly) > clearance(rx, ry) else -0.25
        else:
            msg.linear.x, msg.angular.z = 0.15, 0.0
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.05)

        ap = st["a"].pose.pose
        cov = st["a"].pose.covariance
        pos_err.append(math.hypot(ap.position.x - x, ap.position.y - y))
        yaw_err.append(abs(dyaw(th, yaw(ap.orientation))))
        spread.append(math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0)))

    pub.publish(Twist())

    def pct(v, p):
        v = sorted(v)
        return v[min(len(v)-1, int(len(v)*p))]

    print(f"\n{len(pos_err)} ornek, {RUN_SECONDS:.0f} sn")
    print("konum hatasi (AMCL vs gercek)")
    print(f"  medyan : {statistics.median(pos_err):.3f} m")
    print(f"  %90    : {pct(pos_err, 0.90):.3f} m")
    print(f"  en kotu: {max(pos_err):.3f} m")
    print("yon hatasi")
    print(f"  medyan : {statistics.median(yaw_err):.1f} deg")
    print(f"  en kotu: {max(yaw_err):.1f} deg")
    print("parcacik bulutu yayilimi (kovaryans)")
    print(f"  basta  : {spread[0]:.3f}")
    print(f"  sonda  : {spread[-1]:.3f}")

    med = statistics.median(pos_err)
    print()
    if med < 0.15 and statistics.median(yaw_err) < 5:
        print(">>> KONUMLANDIRMA IYI")
    elif med < 0.40:
        print(">>> KONUMLANDIRMA ORTA - takip ediyor ama gevsek")
    else:
        print(">>> KONUMLANDIRMA KOTU - filtre robotu kaybediyor")
        if spread[-1] < spread[0]:
            print("    bulut daralmis ama yanlis yerde: alpha degerlerini yukselt,")
            print("    filtre odometriye oldugundan fazla guveniyor.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
