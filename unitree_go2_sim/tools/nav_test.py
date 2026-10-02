#!/usr/bin/env python3
"""Send Nav2 a goal and report whether the robot actually got there.

Checks the goal is in free space before sending it - a goal inside an obstacle
fails in a way that looks like a navigation bug but isn't - then tracks the run
against Gazebo ground truth: distance travelled, closest approach to anything,
whether the robot stayed upright, and the final error against the goal.
"""
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY  # noqa: E402

GOALS = [(4.5, 3.5, 0.0), (-3.0, -1.0, math.pi)]


def clearance(x, y):
    best = min(math.hypot(max(x0-x, x-x1, 0.0), max(y0-y, y-y1, 0.0))
               for o in OBSTACLES for (x0, y0, x1, y1) in [aabb(o)])
    return min(best, RX-abs(x), RY-abs(y))


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def roll(q):
    return math.degrees(math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y)))


def main():
    rclpy.init()
    n = rclpy.create_node("nav_test")
    st = {"g": None}
    plans = {"len": [], "n": 0}

    def on_plan(m):
        # Plan length tells planner quality apart from controller quality: a short
        # plan the robot wanders off is a controller problem, a long plan it
        # follows faithfully is a planner or costmap problem.
        d = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                           b.pose.position.y - a.pose.position.y)
                for a, b in zip(m.poses, m.poses[1:]))
        plans["len"].append(d)
        plans["n"] += 1

    n.create_subscription(Odometry, "/odom/ground_truth",
                          lambda m: st.__setitem__("g", m), 10)
    n.create_subscription(Path, "/plan", on_plan, 10)
    client = ActionClient(n, NavigateToPose, "navigate_to_pose")

    t0 = time.time()
    while st["g"] is None and time.time()-t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if st["g"] is None:
        print("no ground-truth odometry - is the simulation running?")
        return 2

    if not client.wait_for_server(timeout_sec=20.0):
        print("no navigate_to_pose action server - is bt_navigator active?")
        return 2

    ok_count = 0
    for gx, gy, gyaw in GOALS:
        c = clearance(gx, gy)
        print(f"\n=== goal ({gx:+.1f}, {gy:+.1f})  clearance {c:.2f} m ===")
        if c < 0.45:
            print("  goal sits too close to an obstacle, skipping")
            continue

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = n.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(gyaw/2)
        goal.pose.pose.orientation.w = math.cos(gyaw/2)

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, send, timeout_sec=15.0)
        if not send.done() or not send.result().accepted:
            print("  goal REJECTED")
            continue
        handle = send.result()
        result_fut = handle.get_result_async()

        plans["len"].clear(); plans["n"] = 0
        start = time.time()
        p0 = st["g"].pose.pose.position
        travelled = 0.0
        prev = (p0.x, p0.y)
        min_clear = 99.0
        fell = False
        while not result_fut.done() and time.time()-start < 180:
            rclpy.spin_once(n, timeout_sec=0.1)
            p = st["g"].pose.pose
            travelled += math.hypot(p.position.x-prev[0], p.position.y-prev[1])
            prev = (p.position.x, p.position.y)
            min_clear = min(min_clear, clearance(p.position.x, p.position.y))
            if abs(roll(p.orientation)) > 60:
                fell = True
                break

        p = st["g"].pose.pose
        err = math.hypot(p.position.x-gx, p.position.y-gy)
        dt = time.time()-start
        if fell:
            print(f"  FELL OVER  x={p.position.x:.2f} y={p.position.y:.2f}")
            continue
        if not result_fut.done():
            print(f"  TIMED OUT ({dt:.0f} s), {err:.2f} m short of the goal")
            handle.cancel_goal_async()
            rclpy.spin_once(n, timeout_sec=2.0)
            continue

        status = result_fut.result().status
        print(f"  time          : {dt:.0f} s")
        print(f"  distance      : {travelled:.2f} m "
              f"(kus ucusu {math.hypot(gx-p0.x, gy-p0.y):.2f} m)")
        if plans["len"]:
            print(f"  first plan    : {plans['len'][0]:.2f} m")
            print(f"  mean plan     : {sum(plans['len'])/len(plans['len']):.2f} m")
            print(f"  replans       : {plans['n']}")
        print(f"  goal error    : {err:.2f} m")
        print(f"  nearest obst. : {min_clear:.2f} m "
              f"({'COLLISION' if min_clear < 0.30 else 'no contact'})")
        names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        print(f"  action status : {status} = {names.get(status, '?')}")
        if status == 4 and err < 0.4 and min_clear >= 0.30:
            ok_count += 1

    print(f"\n=== {ok_count}/{len(GOALS)} goals reached ===")
    rclpy.shutdown()
    return 0 if ok_count == len(GOALS) else 1


if __name__ == "__main__":
    sys.exit(main())
