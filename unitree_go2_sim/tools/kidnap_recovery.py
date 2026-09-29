#!/usr/bin/env python3
"""Kidnap AMCL and see whether it can find itself again.

Three phases, each measured against Gazebo ground truth:

  1. seeded correctly        - baseline, should track
  2. seeded at a wrong pose  - the "kidnapped" state
  3. global localization      - /reinitialize_global_localization, then drive

Phase 2 is expected to stay lost: plain AMCL is a tracking filter and its
recovery_alpha injection only trickles in a few random particles. Phase 3 is the
mechanism actually designed for this - it scatters particles across the whole map
and lets the scan matching vote them down.

The robot is kidnapped by lying to AMCL rather than by teleporting the model,
which keeps odometry continuous and isolates the filter's behaviour.
"""
import math
import pathlib
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY  # noqa: E402

SAFE, LOOK = 1.0, 0.8


def clearance(x, y):
    best = min(math.hypot(max(x0-x, x-x1, 0.0), max(y0-y, y-y1, 0.0))
               for o in OBSTACLES for (x0, y0, x1, y1) in [aabb(o)])
    return min(best, RX-abs(x), RY-abs(y))


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Runner:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("kidnap_recovery")
        self.cmd = self.n.create_publisher(Twist, "/cmd_vel", 10)
        self.init = self.n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.st = {}
        self.n.create_subscription(Odometry, "/odom/ground_truth",
                                   lambda m: self.st.__setitem__("g", m), 10)
        self.n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                   lambda m: self.st.__setitem__("a", m), 10)
        self.global_cli = self.n.create_client(Empty, "/reinitialize_global_localization")
        self.turning = 0.0

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def wait_data(self):
        t0 = time.time()
        while "g" not in self.st and time.time()-t0 < 25:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        return "g" in self.st

    def true_pose(self):
        p = self.st["g"].pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    def seed(self, x, y, th, label):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th/2)
        msg.pose.pose.orientation.w = math.cos(th/2)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        t0 = time.time()
        while self.init.get_subscription_count() == 0 and time.time()-t0 < 15:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        self.st.pop("a", None)
        t0 = time.time()
        while "a" not in self.st and time.time()-t0 < 30:
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.init.publish(msg)
            self.spin(0.5)
        print(f"  [{label}] tohum verildi: x={x:.2f} y={y:.2f} "
              f"yaw={math.degrees(th):+.0f} deg")

    def drive_and_measure(self, secs, label):
        errs = []
        msg = Twist()
        end = time.time() + secs
        while time.time() < end:
            x, y, th = self.true_pose()
            ahead = clearance(x + LOOK*math.cos(th), y + LOOK*math.sin(th))
            if self.turning > 0:
                msg.linear.x, msg.angular.z = 0.0, 0.25
                self.turning -= 0.05
            elif ahead < SAFE:
                lx, ly = x + LOOK*math.cos(th+1.2), y + LOOK*math.sin(th+1.2)
                rx, ry = x + LOOK*math.cos(th-1.2), y + LOOK*math.sin(th-1.2)
                self.turning = 4.0
                msg.linear.x = 0.0
                msg.angular.z = 0.25 if clearance(lx, ly) > clearance(rx, ry) else -0.25
            else:
                msg.linear.x, msg.angular.z = 0.15, 0.0
            self.cmd.publish(msg)
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if "a" in self.st:
                ap = self.st["a"].pose.pose
                errs.append(math.hypot(ap.position.x - x, ap.position.y - y))
        self.cmd.publish(Twist())
        if not errs:
            print(f"  [{label}] /amcl_pose gelmedi")
            return None
        first = statistics.median(errs[:max(1, len(errs)//5)])
        last = statistics.median(errs[-max(1, len(errs)//5):])
        print(f"  [{label}] konum hatasi  ilk %20: {first:.2f} m   "
              f"son %20: {last:.2f} m   en iyi: {min(errs):.2f} m")
        return last


def main():
    r = Runner()
    if not r.wait_data():
        print("ground truth odometri yok - simulasyon calisiyor mu?")
        return 2

    print("\n--- 1) dogru tohum (temel) ---")
    x, y, th = r.true_pose()
    r.seed(x, y, th, "dogru")
    base = r.drive_and_measure(45, "dogru")

    print("\n--- 2) kacirildi: yanlis tohum, kurtarma yok ---")
    x, y, th = r.true_pose()
    # Somewhere else in the room, rotated - a plausible wrong guess
    r.seed(-x - 2.5, -y - 1.5, th + math.pi/2, "yanlis")
    lost = r.drive_and_measure(60, "yanlis")

    print("\n--- 3) kuresel konumlandirma cagrildi ---")
    if not r.global_cli.wait_for_service(timeout_sec=10.0):
        print("  /reinitialize_global_localization yok")
        return 2
    r.global_cli.call_async(Empty.Request())
    r.spin(2.0)
    print("  parcaciklar tum haritaya dagitildi, suruluyor...")
    recovered = r.drive_and_measure(150, "kuresel")

    print("\n=== SONUC ===")
    print(f"  dogru tohumla       : {base:.2f} m")
    print(f"  kacirildiktan sonra : {lost:.2f} m")
    print(f"  kuresel kurtarmadan : {recovered:.2f} m")
    if recovered is not None and recovered < 0.4:
        print("\n>>> KURTARMA BASARILI - kuresel konumlandirma robotu buldu")
    elif recovered is not None and lost is not None and recovered < lost * 0.5:
        print("\n>>> KISMEN TOPARLADI - daha uzun surus veya daha cok parcacik gerek")
    else:
        print("\n>>> KURTARAMADI")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
