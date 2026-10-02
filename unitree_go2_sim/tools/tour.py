#!/usr/bin/env python3
"""Drive a planned coverage tour of the course, for building a complete map.

explore.py is a reactive wanderer: it drives forward and turns away from whatever
is in front of it. That is enough to exercise the gait, but it does not cover a
room - measured over two runs totalling 12 minutes it mapped 70% of the real
surface and both runs ended within a metre of where they started, orbiting the
middle. Coverage needs a plan, not a reflex.

So this walks a lawnmower pattern instead. Waypoints are laid on a grid, ordered
boustrophedon (alternate rows reversed, so the tour never jumps across the room),
and each leg is planned with A* over the world geometry inflated by CLEAR_R.

That inflation is the whole point of planning it this way. pointcloud_to_laserscan
drops returns closer than range_min (0.5 m), and slam_toolbox marks the space a ray
travelled through as free - so a robot that brushes a wall erases it and paints the
far side as open floor. Keeping every metre of the tour CLEAR_R away from geometry
makes that impossible by construction rather than by luck.

    python3 tour.py            # full tour
    python3 tour.py 300        # give up after 300 s wherever it has got to
"""
import heapq
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY  # noqa: E402

# 0.70 was not enough. Two tours planned entirely on cells at least 0.70 m clear
# still recorded a closest approach of 0.39 m: a trotting quadruped slides
# sideways through turns, so where the body actually goes is not where the plan
# put it. The margin has to absorb that drift, not just clear the geometry.
CLEAR_R = 0.85      # keep this far from every surface - well outside range_min
RES = 0.10          # planning grid
STEP = 1.00         # spacing between lawnmower waypoints
REACH = 0.28        # a waypoint counts as reached within this
# Commanded speed is not achieved speed, and the gap is enormous at the low end.
# Measured against /odom/ground_truth on this robot:
#
#     commanded    achieved     body tilt
#     0.15 m/s     0.021 m/s    -
#     0.25 m/s     0.085 m/s    -
#     0.35 m/s     0.141 m/s    12 deg
#     0.50 m/s     0.149 m/s    13 deg
#     0.70 m/s     0.203 m/s    15 deg
#
# At 0.15 the gait shuffles in place and the body barely translates - 14% of what
# was asked. That is what made the first tours look like a coverage failure: the
# robot was not exploring badly, it was hardly moving.
#
# 0.50 is not the answer either: it went over on its back 22 waypoints in. Note
# how little 0.35 gives up - 0.141 against 0.149 m/s, a 5% difference in ground
# speed for a much calmer gait. The straight-line tilt figures above understate
# the risk anyway, because what actually tips the robot is the step change when a
# turn ends and full forward speed is commanded in one tick, which is why RAMP
# exists below.
V_LIN = 0.35        # forward speed command (~0.14 m/s on the ground)
V_ANG = 0.40        # turn rate command (~0.26 rad/s on the ground)
ALIGN = 0.35        # turn in place while heading error exceeds this (rad)
ABORT = 0.35        # stop everything if we ever get this close to something
RAMP = 0.04         # most the linear command may change per control tick

# --fill threads gaps where the planned margin is barely above range_min, so what
# matters there is not speed but how far the body slides off the plan. Measured:
# a tour planned at 0.65 m still recorded 0.35 m, i.e. 0.30 m of drift, and that
# is below range_min - exactly the condition that erases a wall. Halving the
# command roughly halves the slide, and these gaps are a few metres long, so the
# extra time is worth it.
V_FILL = 0.22       # forward speed command while filling (~0.07 m/s)

RECTS = [aabb(o) for o in OBSTACLES]
NX = int(2 * RX / RES)
NY = int(2 * RY / RES)


def clearance(x, y):
    """Distance to the nearest real surface, walls included."""
    best = min((math.hypot(max(x0 - x, x - x1, 0.0), max(y0 - y, y - y1, 0.0))
                for (x0, y0, x1, y1) in RECTS), default=99.0)
    return min(best, RX - abs(x), RY - abs(y))


def cell(x, y):
    return int((x + RX) / RES), int((y + RY) / RES)


def centre(i, j):
    return -RX + (i + 0.5) * RES, -RY + (j + 0.5) * RES


# The north-west alcove is the reason this exists and the reason it is 0.60.
# A partition at x=-3.0 runs from y=+2.4 to the north wall, so the alcove is wide
# inside (0.90-1.00 m of clearance) but its doorway is not: 0.51-0.58 m. Anything
# above 0.60 cannot plan through that gap, which is how a room the robot can
# physically walk into stays a grey wedge on the map. Measured connectivity: at
# 0.60 nothing in this world is cut off, and it still leaves 0.10 m over
# range_min so the walls keep reporting.
FILL_R = 0.60       # tighter margin used only by --fill, still above range_min


def free_grid(margin):
    return [[clearance(*centre(i, j)) >= margin for j in range(NY)]
            for i in range(NX)]


def reachable(grid):
    """Cells connected to the spawn point through the given grid."""
    from collections import deque
    s = cell(0.0, 0.0)
    if not grid[s[0]][s[1]]:
        return set()
    seen = {s}
    q = deque([s])
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if (0 <= ni < NX and 0 <= nj < NY and grid[ni][nj]
                    and (ni, nj) not in seen):
                seen.add((ni, nj))
                q.append((ni, nj))
    return seen


FREE = free_grid(CLEAR_R)


def use_margin(margin, only_outside=None):
    """Rebuild the planning grid at this margin, keeping reachable cells only.

    A margin seals the narrower gaps, which leaves pockets - the north-west
    corner is one - that are open floor in reality but unreachable under it. A*
    searches the whole grid before failing, so a waypoint in a pocket stalls the
    tour and then skips. Removing them up front is cheaper.

    only_outside, when given, additionally drops every cell in that set. --fill
    passes the cells the main tour could already reach, so what is left is
    exactly the pockets it had to skip.
    """
    grid = free_grid(margin)
    keep = reachable(grid)
    for i in range(NX):
        for j in range(NY):
            FREE[i][j] = grid[i][j] and (i, j) in keep
    if only_outside is not None:
        return {c for c in keep if c not in only_outside}
    return keep


use_margin(CLEAR_R)


def nearest_free(x, y):
    """Snap a point onto the free grid, for when the robot starts off it."""
    ci, cj = cell(x, y)
    best, bd = None, 1e9
    for i in range(NX):
        for j in range(NY):
            if not FREE[i][j]:
                continue
            d = (i - ci) ** 2 + (j - cj) ** 2
            if d < bd:
                best, bd = (i, j), d
    return best


def plan(a, b):
    """A* from grid cell a to grid cell b. Returns world points, or None."""
    if not FREE[b[0]][b[1]]:
        return None
    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def h(c):
        return math.hypot(c[0] - b[0], c[1] - b[1])

    openq = [(h(a), 0.0, a)]
    came, gscore = {}, {a: 0.0}
    seen = set()
    while openq:
        _, g, cur = heapq.heappop(openq)
        if cur == b:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return [centre(*c) for c in reversed(path)]
        if cur in seen:
            continue
        seen.add(cur)
        for dx, dy in nbrs:
            ni, nj = cur[0] + dx, cur[1] + dy
            if not (0 <= ni < NX and 0 <= nj < NY) or not FREE[ni][nj]:
                continue
            ng = g + math.hypot(dx, dy) * RES
            if ng < gscore.get((ni, nj), 1e9):
                gscore[(ni, nj)] = ng
                came[(ni, nj)] = cur
                heapq.heappush(openq, (ng + h((ni, nj)) * RES, ng, (ni, nj)))
    return None


def waypoints(step=STEP):
    """Lawnmower over the free space, alternate rows reversed."""
    pts, row, y = [], 0, -RY + step
    while y <= RY - step:
        line, x = [], -RX + step
        while x <= RX - step:
            i, j = cell(x, y)
            if 0 <= i < NX and 0 <= j < NY and FREE[i][j]:
                line.append((x, y))
            x += step
        if row % 2:
            line.reverse()
        pts += line
        y += step
        row += 1
    return pts


def safest_waypoints(cells, spacing=1.5):
    """Pick waypoints down the middle of whatever these cells form.

    A lattice cannot cover a narrow corridor: the only safe line through the gap
    south of the east crate is y=-4.40, and no lattice that also has to land
    somewhere sensible elsewhere puts a point there. At 1.00 m spacing the
    nearest candidates are 0.20 m from the crate, at 0.50 m they are 0.20 m from
    the crate or 0.50 m from the wall, so the corridor simply never gets a
    waypoint and its wall never gets mapped.

    So choose by clearance instead of by position: take the roomiest cell, drop
    everything within spacing of it, repeat. On a corridor that walks the centre
    line, which is exactly where the robot should be.
    """
    cand = sorted(((clearance(*centre(i, j)), (i, j)) for (i, j) in cells),
                  reverse=True)
    picked = []
    for cl, (i, j) in cand:
        x, y = centre(i, j)
        if all(math.hypot(x - px, y - py) >= spacing for px, py in picked):
            picked.append((x, y))
    # Visit them nearest-first from the spawn, so the tour does not criss-cross.
    order, cur, left = [], (0.0, 0.0), list(picked)
    while left:
        nxt = min(left, key=lambda p: math.hypot(p[0]-cur[0], p[1]-cur[1]))
        left.remove(nxt)
        order.append(nxt)
        cur = nxt
    return order


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    args = [a for a in sys.argv[1:] if a != "--fill"]
    fill = "--fill" in sys.argv
    budget = float(args[0]) if args else 1e9

    pockets = None
    if fill:
        # Everything the normal tour could reach, then the same question at the
        # tighter margin. The difference is what it never visited.
        main_reach = reachable(free_grid(CLEAR_R))
        pockets = use_margin(FILL_R, only_outside=main_reach)
        print(f"--fill: {len(pockets)} cells skipped at {CLEAR_R} m, "
              f"visiting them at {FILL_R} m")
    rclpy.init()
    n = rclpy.create_node("tour")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    st = {"g": None}
    n.create_subscription(Odometry, "/odom/ground_truth",
                          lambda m: st.__setitem__("g", m), 10)
    t0 = time.time()
    while st["g"] is None and time.time() - t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if st["g"] is None:
        print("no odometry - is the simulation running?")
        return

    def pose():
        p = st["g"].pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    # A previous run may have left the robot parked in a tight spot - the abort
    # check would then fire on the first tick and the tour would end having done
    # nothing, reporting the standing clearance as though it had driven there.
    # Walk out of the corner first.
    x, y, th = pose()
    if clearance(x, y) < CLEAR_R:
        print(f"starting clearance {clearance(x, y):.2f} m - backing out first")
        t_esc = time.time()
        msg = Twist()
        while time.time() - t_esc < 40:
            x, y, th = pose()
            if clearance(x, y) >= CLEAR_R:
                break
            # steer towards whichever heading opens up fastest
            best = max(((clearance(x + 0.9 * math.cos(th + a),
                                   y + 0.9 * math.sin(th + a)), a)
                        for a in [i * math.pi / 8 for i in range(-8, 8)]))[1]
            if abs(best) > ALIGN:
                msg.linear.x = 0.0
                msg.angular.z = V_ANG if best > 0 else -V_ANG
            else:
                msg.linear.x = min(V_FILL, msg.linear.x + RAMP)
                msg.angular.z = max(-0.2, min(0.2, 1.2 * best))
            pub.publish(msg)
            rclpy.spin_once(n, timeout_sec=0.05)
        pub.publish(Twist())
        x, y, _ = pose()
        print(f"  clear now, clearance {clearance(x, y):.2f} m")

    if pockets is not None:
        wps = safest_waypoints(pockets)
        if not wps:
            print("no skipped area left - nothing to do")
            rclpy.shutdown()
            return
        print(f"{len(wps)} waypoints (chosen at the widest points), "
              f"margin {FILL_R} m")
    else:
        wps = waypoints()
        print(f"{len(wps)} waypoints, spacing {STEP} m, margin {CLEAR_R} m")

    start = time.time()
    msg = Twist()
    min_seen = 99.0   # measured over the tour only, after any escape above
    done = 0
    aborted = False

    for wi, target in enumerate(wps):
        if time.time() - start > budget:
            print("out of time")
            break
        x, y, _ = pose()
        route = plan(nearest_free(x, y), cell(*target))
        if route is None:
            continue
        # Thin the cell-by-cell path down to something worth steering at, but
        # not far: the robot drives straight between consecutive points, so
        # every skipped cell is a chord across whatever the path was curving
        # around. At every 6th cell (0.6 m) that cost 0.36 m of the planned
        # margin on a 90-degree corner. Every 3rd is cheap and much tighter.
        route = route[::3] + [target]

        for gx, gy in route:
            while time.time() - start <= budget:
                x, y, th = pose()
                here = clearance(x, y)
                min_seen = min(min_seen, here)
                if here < ABORT:
                    aborted = True
                    break
                d = math.hypot(gx - x, gy - y)
                if d < REACH:
                    break
                err = wrap(math.atan2(gy - y, gx - x) - th)
                if abs(err) > ALIGN:
                    want_lin, want_ang = 0.0, (V_ANG if err > 0 else -V_ANG)
                else:
                    want_lin = V_FILL if fill else V_LIN
                    want_ang = max(-0.2, min(0.2, 1.2 * err))
                # Ease into the new linear command. Going from a standstill to
                # full speed in one tick is what put the robot on its back.
                msg.linear.x += max(-RAMP, min(RAMP, want_lin - msg.linear.x))
                msg.angular.z = want_ang
                pub.publish(msg)
                rclpy.spin_once(n, timeout_sec=0.05)
            if aborted:
                break
        if aborted:
            break
        done += 1
        if done % 5 == 0:
            print(f"  {done}/{len(wps)} waypoints, {time.time()-start:.0f} s")

    pub.publish(Twist())
    x, y, _ = pose()
    q = st["g"].pose.pose.orientation
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z),
                                   1 - 2 * (q.x * q.x + q.y * q.y)))
    print(f"done: {done}/{len(wps)} waypoints, {time.time()-start:.0f} s")
    print(f"final pose x={x:.2f} y={y:.2f}  roll={roll:+.0f} "
          f"({'FELL OVER' if abs(roll) > 60 else 'upright'})")
    print(f"smallest clearance seen: {min_seen:.2f} m  "
          f"({'CAME TOO CLOSE' if aborted else 'safe'})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
