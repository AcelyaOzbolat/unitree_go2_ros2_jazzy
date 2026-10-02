#!/usr/bin/env python3
"""Generate simple_room.sdf and check the layout is actually walkable.

Everything is a box or a cylinder: no meshes, no textures, all <static>true</static>,
so the world costs almost nothing to simulate compared to the TI building.

Design constraints, all enforced by check() below:
  * MIN_GAP between any two obstacles, so the Go2 (0.70 x 0.31 m body) can pass
    anywhere it can fit at all rather than wedging itself into a slot.
  * A clear disc around the spawn point, so the robot does not materialise inside
    an obstacle.
  * Every obstacle at least MIN_H tall, so it appears in the 0.15-0.70 m band that
    pointcloud_to_laserscan slices for /scan. A shorter prop would be invisible to
    the 2D map while still blocking the robot.
  * The layout must not be mirror-symmetric, or AMCL cannot tell which way round
    the robot is once it is placed.
"""
import math

# Room interior half-extents.
RX, RY = 6.0, 5.0
# Walls are 3 m so they sit comfortably above the laser slice's 1.6 m ceiling.
# (Measured: raising them from 1.5 m did not change the residual scan gaps, so
# this is headroom, not a fix.)
WALL_T, WALL_H = 0.2, 3.0

MIN_GAP = 1.20     # metres of free floor between obstacles (Go2 body is 0.31 m wide)
SPAWN_CLEAR = 1.20  # free radius around the spawn point
MIN_H = 0.80       # every obstacle must reach into the laser slice
SPAWN = (0.0, 0.0)

# name, shape, x, y, sx/radius, sy, height, colour
OBSTACLES = [
    # Central block: walking a full loop around it is what gives slam_toolbox a
    # loop closure to work with.
    ("loop_block",   "box", 3.0,  1.5, 2.0, 2.0, 1.00, (0.30, 0.45, 0.70)),

    # Two wall-attached partitions. They are deliberately NOT mirror images:
    # the room has to look different from every heading or AMCL can converge
    # to the wrong pose.
    ("part_north",   "box", -3.0,  3.7, 0.2, 2.6, 1.30, (0.65, 0.35, 0.30)),
    ("part_south",   "box",  1.0, -3.7, 0.2, 2.6, 1.30, (0.65, 0.35, 0.30)),
    ("part_east",    "box",  4.6, -1.6, 2.8, 0.2, 1.30, (0.65, 0.35, 0.30)),

    # Pillars: round returns look different from flat walls in a scan, which
    # helps scan matching.
    ("pillar_a",     "cyl", -4.4, -3.2, 0.30, 0.0, 1.20, (0.55, 0.55, 0.58)),
    ("pillar_b",     "cyl", -4.4,  1.2, 0.30, 0.0, 1.20, (0.55, 0.55, 0.58)),
    ("pillar_c",     "cyl",  0.2,  3.4, 0.25, 0.0, 1.10, (0.55, 0.55, 0.58)),

    # Loose crates.
    ("crate_a",      "box", -1.8, -2.8, 1.0, 1.0, 0.90, (0.72, 0.60, 0.35)),
    ("crate_b",      "box", -1.7,  0.8, 0.8, 1.2, 0.95, (0.72, 0.60, 0.35)),
    ("crate_c",      "box",  4.0, -3.4, 1.2, 0.8, 0.90, (0.72, 0.60, 0.35)),
]


def aabb(o):
    _, shape, x, y, a, b, _, _ = o
    hx = a if shape == "cyl" else a / 2.0
    hy = a if shape == "cyl" else b / 2.0
    return (x - hx, y - hy, x + hx, y + hy)


def gap(o1, o2):
    ax0, ay0, ax1, ay1 = aabb(o1)
    bx0, by0, bx1, by1 = aabb(o2)
    dx = max(bx0 - ax1, ax0 - bx1, 0.0)
    dy = max(by0 - ay1, ay0 - by1, 0.0)
    return math.hypot(dx, dy)


def check():
    errs = []
    for o in OBSTACLES:
        name, _, _, _, _, _, h, _ = o
        x0, y0, x1, y1 = aabb(o)

        if h < MIN_H:
            errs.append(f"{name}: height {h:.2f} < {MIN_H} (invisible to /scan)")

        if x0 < -RX or x1 > RX or y0 < -RY or y1 > RY:
            errs.append(f"{name}: sticks out of the room")

        # Distance to each wall. Touching a wall on purpose is fine (partitions);
        # a near-miss that leaves an unusable slot is not.
        for wname, d in (("west", x0 + RX), ("east", RX - x1),
                         ("south", y0 + RY), ("north", RY - y1)):
            if 0.01 < d < MIN_GAP:
                errs.append(f"{name}: {d:.2f} m slot against {wname} wall "
                            f"(either touch it or leave {MIN_GAP} m)")

        # Spawn clearance.
        cx = min(max(SPAWN[0], x0), x1)
        cy = min(max(SPAWN[1], y0), y1)
        d = math.hypot(SPAWN[0] - cx, SPAWN[1] - cy)
        if d < SPAWN_CLEAR:
            errs.append(f"{name}: only {d:.2f} m from spawn (need {SPAWN_CLEAR})")

    for i, o1 in enumerate(OBSTACLES):
        for o2 in OBSTACLES[i + 1:]:
            g = gap(o1, o2)
            if g < MIN_GAP:
                errs.append(f"{o1[0]} <-> {o2[0]}: {g:.2f} m gap (need {MIN_GAP})")
    return errs


def box_model(name, x, y, sx, sy, h, c, static=True):
    r, g, b = c
    return f"""
    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{x} {y} {h/2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material>
            <ambient>{r*0.5:.3f} {g*0.5:.3f} {b*0.5:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def cyl_model(name, x, y, rad, h, c):
    r, g, b = c
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {h/2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{rad}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{rad}</radius><length>{h}</length></cylinder></geometry>
          <material>
            <ambient>{r*0.5:.3f} {g*0.5:.3f} {b*0.5:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def build():
    parts = []
    wc = (0.78, 0.78, 0.74)
    # Perimeter, overlapping at the corners so there is no pinhole gap for a ray.
    parts.append(box_model("wall_north", 0.0,  RY + WALL_T / 2, 2 * RX + 2 * WALL_T, WALL_T, WALL_H, wc))
    parts.append(box_model("wall_south", 0.0, -RY - WALL_T / 2, 2 * RX + 2 * WALL_T, WALL_T, WALL_H, wc))
    parts.append(box_model("wall_east",  RX + WALL_T / 2, 0.0, WALL_T, 2 * RY, WALL_H, wc))
    parts.append(box_model("wall_west", -RX - WALL_T / 2, 0.0, WALL_T, 2 * RY, WALL_H, wc))

    for o in OBSTACLES:
        name, shape, x, y, a, b, h, c = o
        if shape == "box":
            parts.append(box_model(name, x, y, a, b, h, c))
        else:
            parts.append(cyl_model(name, x, y, a, h, c))
    return "".join(parts)


HEADER = f"""<?xml version="1.0" ?>
<!-- Generated by unitree_go2_description/tools/gen_simple_room.py - edit that script, not this file. -->
<sdf version="1.8">
  <world name="simple_room">
    <!-- bullet-featherstone handles articulated bodies far better than the default
         engine; a quadruped's legs jitter noticeably without it. -->
    <physics name="1ms" type="bullet-featherstone">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <!-- Required for gpu_lidar and camera sensors to render. Without it the
         Velodyne and the RGB camera stay silent and only the IMU works, because
         the IMU system plugin is declared inside the robot's own xacro. -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- Parked well outside the room. A directional light's pose does not affect
         the lighting at all - only <direction> does - but Gazebo still draws a
         marker at the pose, and at the origin that marker sits directly over the
         robot in any top-down view and looks like a bug in the robot model. -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>-22 -18 25 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <surface>
            <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

FOOTER = """
  </world>
</sdf>
"""

if __name__ == "__main__":
    import sys
    errs = check()
    if errs:
        print("LAYOUT REJECTED:")
        for e in errs:
            print("  -", e)
        sys.exit(1)
    print(f"Layout OK: {len(OBSTACLES)} obstacles, min gap {MIN_GAP} m, "
          f"room {2*RX:.0f} x {2*RY:.0f} m")
    out = sys.argv[1] if len(sys.argv) > 1 else "simple_room.sdf"
    with open(out, "w") as f:
        f.write(HEADER + build() + FOOTER)
    print("written:", out)
