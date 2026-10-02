# Troubleshooting map

Every problem hit while building this stack, with the symptom you actually see,
the command that identifies it, and the fix.

These are written symptom-first because that is how you meet them. Several of
these bugs point at the wrong subsystem entirely — a broken transform looks like
a SLAM failure, a display bug looks like a dead node — so the "how to check"
step matters more than the fix.

**A note on method.** Nearly every entry here was found by comparing against
Gazebo ground truth rather than by reading code or guessing. The simulator knows
where the robot really is; `/odom/ground_truth` is never fed into the stack, only
used to score it. If you take one thing from this page, take that.

---

## Index

| # | Symptom | Section |
|---|---|---|
| 1 | No LiDAR or camera data, but the topics exist | [Sensors silent](#1-sensors-produce-no-data-at-all) |
| 2 | Launch dies with "Unable to parse the value of parameter robot_description" | [robot_description](#2-unable-to-parse-the-value-of-parameter-robot_description) |
| 3 | Two nodes publishing `/odom` | [Duplicate odom](#3-two-publishers-on-odom) |
| 4 | Map never grows; map seems attached to the robot | [base_footprint below the floor](#4-the-map-never-grows-and-travels-with-the-robot) |
| 5 | Star-shaped spokes punching through walls | [Scan angle aliasing](#5-star-shaped-spokes-through-the-walls) |
| 6 | Map internally distorted; no alignment fixes it | [Scan rotated ~100°](#6-the-map-is-distorted-not-just-shifted) |
| 7 | SLAM never sees the robot turn | [Leg odometry yaw](#7-slam-never-sees-the-robot-turn) |
| 8 | Robot falls over while walking | [Gait tuning](#8-the-robot-falls-over) |
| 9 | slam_toolbox starts but publishes nothing | [Lifecycle nodes](#9-a-node-starts-but-publishes-nothing) |
| 10 | Particle cloud spreads instead of converging | [AMCL motion noise](#10-the-particle-cloud-spreads-instead-of-converging) |
| 11 | Particle cloud invisible in RViz | [Message type mismatch](#11-the-particle-cloud-is-invisible) |
| 12 | AMCL logs "Please set the initial pose" forever | [Initial pose](#12-amcl-waits-forever-for-an-initial-pose) |
| 13 | Robot cannot find itself after being moved | [Kidnapped robot](#13-the-robot-cannot-find-itself-after-being-moved) |
| 14 | Robot grazes obstacles; planner then cannot escape | [Local costmap static layer](#14-the-robot-grazes-obstacles-and-the-planner-gets-stuck) |
| 15 | Phantom obstacles flicker in open floor | [Tilted laser slice](#15-phantom-obstacles-in-open-floor) |
| 16 | Nav2 aborts with "Failed to bring up all requested nodes" | [Launch order](#16-failed-to-bring-up-all-requested-nodes) |
| 17 | RViz goal button does nothing | [Wrong RViz tool](#17-the-goal-button-does-nothing) |
| 18 | Measurements contradict each other between runs | [Stale processes](#18-measurements-that-contradict-each-other) |
| 19 | Open floor mapped on the far side of a wall | [Seeing through a wall](#19-open-floor-on-the-far-side-of-a-wall) |
| 20 | The robot barely moves although `/cmd_vel` is accepted | [Commanded vs achieved speed](#20-the-robot-barely-moves) |

---

## 1. Sensors produce no data at all

**Symptom.** The IMU works. The LiDAR and camera publish nothing, yet
`ros2 topic list` shows their topics, so everything looks wired up.

**How to check.** Ask Gazebo, not ROS:

```bash
gz topic -i -t /velodyne_points/points
```

If the publisher and subscriber are at the **same address**, that topic is being
advertised only by the ROS–Gazebo bridge. Nothing in Gazebo is producing data.

**Cause.** The world file is missing the sensors system plugin. Without it Gazebo
never renders `gpu_lidar` or `camera` sensors. The IMU still works because its
plugin is declared inside the robot's own xacro, which is what makes this
confusing.

**Fix.** Add to the world's `<world>` block:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

---

## 2. "Unable to parse the value of parameter robot_description"

**Symptom.** The launch aborts immediately. Gazebo may open but no robot spawns,
and every downstream symptom follows — no TF, no sensors.

```
[ERROR] [launch]: Caught exception in launch: Unable to parse the value of
parameter robot_description as yaml. If the parameter is meant to be a string,
try wrapping it in launch_ros.parameter_descriptions.ParameterValue(...)
```

**Cause.** `launch` runs `yaml.safe_load()` on the xacro output to decide whether
it is a string or a structure. Any **colon followed by a space** in the URDF —
*including inside an XML comment* — makes YAML read that line as a mapping key and
the whole launch fails.

**Fix.** Two layers. Remove the colon-space from the URDF text, and wrap every
xacro command so it can never happen again:

```python
from launch_ros.parameter_descriptions import ParameterValue

robot_description = {"robot_description": ParameterValue(
    Command(["xacro ", LaunchConfiguration("model_path")]),
    value_type=str,
)}
```

---

## 3. Two publishers on `/odom`

**Symptom.** Odometry behaves erratically; consumers see inconsistent
`child_frame_id`s.

**How to check.**

```bash
ros2 topic info /odom --verbose | grep -c "Endpoint type: PUBLISHER"
```

Anything other than `1` is a problem.

**Cause.** Gazebo's `OdometryPublisher` plugin and `robot_localization`'s EKF were
both writing to `/odom`, with different child frames. The bridge was also
**bidirectional** (`@`), so the EKF's output was being pushed back into Gazebo.

**Fix.** Give the ground-truth odometry its own topic and bridge it one-way:

```
/odom/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

Note the `[` — one-way, Gazebo to ROS. Keep ground truth off the control path; it
is for measurement only.

---

## 4. The map never grows, and travels with the robot

The single most instructive bug in this project, and two symptoms with one cause.

**Symptom.** SLAM runs without errors but the map stays small — about 3.5 × 2.7 m
in a 12 × 10 m room — and it appears *attached to the robot*, moving along as the
robot walks.

**How to check.** Look at the scan's range distribution:

```bash
python3 unitree_go2_sim/tools/scan_probe.py
```

If the ranges cluster tightly around one value instead of reaching the walls,
you are mapping the floor. In this case **79 %** of rays fell between 1.0 and
1.5 m.

Then check where `base_footprint` actually sits:

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

The z should equal the robot's standing height (0.23 m here). It read **0.631 m**.

**Cause.** `base_footprint` was 0.4 m *below* the floor, so the height band that
defines the laser slice — nominally 0.15–0.70 m above ground — was cutting
through the ground itself.

The LiDAR sits 0.35 m up with a ±15° vertical spread, so its downward rays meet
the floor at `0.35 / tan(15°) ≈ 1.31 m`. Those floor returns became a **ring of
obstacles 1.31 m around the robot**. That ring was the entire map — which is why
it never grew, and why it followed the robot.

Three faults stacked up:

1. **Foot contacts were never published.** CHAMP's `state_estimation` uses a
   synchroniser over `joint_states` **and** `foot_contacts`. With no contacts the
   callback never fired, `updateJointPositions()` was never called, and the leg
   kinematics ran on zero joint angles — giving a fully-extended-leg height of
   `thigh + calf = 0.213 + 0.213 = 0.426 m` instead of 0.233 m.
2. **Contacts were gated off in simulation.** `quadruped_controller.cpp` published
   them only `if (publish_foot_contacts_ && !in_gazebo_)`, expecting a Gazebo
   contact sensor that this robot does not have.
3. **The EKF integrated gravity.** `base_to_footprint.yaml` fed IMU linear
   acceleration into position without `remove_gravitational_acceleration`, so a
   constant 9.81 m/s² was integrated too, inflating 0.426 m to 0.631 m.

**Fix.** Enable `publish_foot_contacts`, drop the `!in_gazebo_` gate on the
contact publisher only (leave it on joint states — `joint_state_broadcaster`
already publishes those in Gazebo), and turn off the linear-acceleration terms in
the base-to-footprint EKF.

| | before | after | truth |
|---|---|---|---|
| `base_footprint → base_link` z | 0.631 m ±0.062 | **0.214 m ±0.000** | 0.233 m |
| mean scan range | 1.33 m | 4.40 m | — |
| map size | 3.5 × 2.7 m | **12.1 × 10.1 m** | room is 12 × 10 m |

**What it teaches.** A frame that is silently wrong produces symptoms that look
like a mapping bug. Check your TF tree's *values*, not just its shape.

---

## 5. Star-shaped spokes through the walls

**Symptom.** Triangular fans radiate out from the robot in the map, carving free
space straight through walls.

**Cause.** The scan's `angle_increment` exactly matched the sensor's own
horizontal spacing. Quantisation then leaves some angular bins with no sample at
all; with `use_inf: true` those become `+inf`, and SLAM reads `+inf` as *free all
the way to `range_max`*.

**Fix.** Make the output bin **wider** than the sensor's sample spacing, so every
bin is guaranteed to contain at least one sample. The VLP-16 here gives 0.818°, so
1.0° bins work:

```yaml
angle_increment: 0.01745   # 1.0 deg, sensor gives 0.818 deg
```

Raising `max_height` also helps: a beam that grazes over a near obstacle is
already high by the time it reaches a far wall, and a low ceiling discards it.

Invalid rays went from 14/440 to 4/361.

---

## 6. The map is distorted, not just shifted

**Symptom.** The map is clearly wrong but not in a way that a rotation or offset
explains.

**How to check.** Score it, then search for the best rigid alignment:

```bash
python3 unitree_go2_sim/tools/score_map.py  map.pgm map.yaml
python3 unitree_go2_sim/tools/align_score.py map.pgm map.yaml
```

If the best alignment still scores badly, the map is **internally distorted** —
a different problem from a shifted one.

Then test the scan itself against the known world from the robot's true pose:

```bash
python3 unitree_go2_sim/tools/check_scan.py
```

**Cause.** The scan was rotated by roughly 100°. `base_footprint → base_link`
carried a yaw of −102.7°, when by definition it should be zero — `base_footprint`
is `base_link` projected onto the ground and shares its heading. The laser cloud
reaches `base_footprint` through that link, so it rotated with the error.

The yaw drifted because *every* yaw measurement had been disabled for that filter
while chasing a different bug. An unobserved state in an EKF does not stay at
zero — it random-walks.

**Fix.** Anchor the yaw to a source that is correct by construction. CHAMP's
`base_to_footprint_pose` publishes exactly 0.00°, which is the right value, so
re-enabling yaw from **that pose only** (not from the IMU) pins it.

Scan median error: **2.213 m → 0.021 m**.

**What it teaches.** Turning a measurement off is not the same as forcing a value.
If a state should be a constant, feed it that constant.

---

## 7. SLAM never sees the robot turn

**Symptom.** Mapping works while driving straight, then the robot turns near a
wall and the pose jumps — in RViz it can appear *behind* the wall.

**How to check.** Compare all three sources during one turn in place:

```bash
python3 unitree_go2_sim/tools/odom_quality.py
```

Measured over a 20 s turn:

```
IMU        +121.9°     correct
/odom/raw  −166.6°     CHAMP leg odometry, wrong sign and magnitude
/odom        −0.3°     EKF output: the two cancelled
ground truth +122.2°
```

**Cause.** CHAMP's leg odometry reports a badly wrong yaw rate. Fused against a
correct IMU, the two averaged out to nothing, so `/odom` reported no rotation at
all. Scan matching then had to find the whole rotation unaided — which fails next
to a flat wall, where the geometry is degenerate.

**Fix.** Stop fusing the leg odometry's yaw rate, and take yaw from the gyro:

```python
# [x y z  roll pitch yaw  vx vy vz  vroll vpitch vyaw  ax ay az]
odom0_config: [F,F,F, F,F,F, T,T,F, F,F,F, F,F,F]   # vx, vy only
imu0_config:  [F,F,F, F,F,F, F,F,F, F,F,T, F,F,F]   # yaw RATE only
```

Note that absolute yaw is off as well. The `odom` frame is meant to be locally
smooth and free to drift; world-referenced heading belongs in `map`, which is
SLAM's job. Feeding absolute yaw here produced nonsense (−243° over a turn the
IMU itself measured correctly as +303°).

Result: **0.3 %** error over a full turn.

---

## 8. The robot falls over

**Symptom.** The robot topples during normal walking, especially when a forward
command steps straight into a turn.

**How to check.**

```bash
python3 unitree_go2_sim/tools/gait_stability.py
```

It reports *where* the robot fell and how far it was from the nearest obstacle —
which separates a gait problem (falls in open ground) from a collision.

**Causes and fixes, in order of impact.**

1. **`stance_duration`.** A 2×2 experiment isolated it: 0.16 s fell within 0.3 m
   on a demanding course, 0.25 s walked 3.5 m upright. `update_rate` turned out
   not to matter.
2. **Abrupt velocity steps.** Stepping from 0.22 m/s straight into 0.45 rad/s puts
   the robot on its back. Under Nav2 this is handled by
   `nav2_velocity_smoother`; by hand, stop before turning.
3. **Commands above the gait limits.** `gait.yaml` caps 0.3 m/s and 0.5 rad/s.
   Teleop's defaults (0.5 / 1.0) exceed both and CHAMP clips them.

At 0.15 m/s with gentle turns the robot walked 216 s without falling, with or
without SLAM running.

---

## 9. A node starts but publishes nothing

**Symptom.** `slam_toolbox` (or `map_server`, or `amcl`) starts, logs no error,
and publishes nothing. `ros2 node list` shows it running.

**How to check.**

```bash
ros2 lifecycle get /slam_toolbox
```

If it says `unconfigured` or `inactive`, that is the whole story.

**Cause.** These are **lifecycle nodes**. Launched as a plain `Node` they load
their parameters and then wait. Something has to drive them through
*configure* → *activate*.

**Fix.** Either launch them under `nav2_lifecycle_manager` with
`autostart: True`, or drive the transitions from the launch file with
`LifecycleNode` plus `EmitEvent`/`RegisterEventHandler`. Both patterns are used in
this repo — see `slam_2d.launch.py` and `localization_2d.launch.py`.

---

## 10. The particle cloud spreads instead of converging

**Symptom.** AMCL tracks the robot but loosely, and the cloud gets *wider* the
longer it runs.

**How to check.**

```bash
python3 unitree_go2_sim/tools/amcl_quality.py 120
```

Watch the spread figure, not just the error. It should shrink. Ours went
0.63 → 1.52.

**Cause.** The motion-model noise terms (`alpha1`–`alpha4`) had been set by
reasoning — "a legged robot slips more than a wheeled one, so double the
defaults" — rather than by measurement. Measuring the odometry showed the
opposite shape:

- rotation: **0.3–0.5 %** error — nearly perfect
- translation: **2–18 %** — poor, and growing with distance

So the largest noise was being injected into the one quantity the odometry gets
right.

**Fix.** Set the alphas from measured odometry:

```yaml
alpha1: 0.1   # rotation from rotation      - heading is accurate
alpha2: 0.2   # rotation from translation
alpha3: 0.5   # translation from translation - the weak axis
alpha4: 0.1   # translation from rotation   - heading is accurate
max_beams: 180
```

| | before | after |
|---|---|---|
| position error | 0.178 m | **0.112 m** |
| heading error | 6.3° | **4.2°** |
| cloud spread | 0.63 → 1.52 | 0.62 → **0.32** |

**What it teaches.** The alphas are a *description of your odometry's real error*,
not free tuning knobs. Measure before setting them.

---

## 11. The particle cloud is invisible

**Symptom.** Localization works, the robot is in the right place, but RViz shows
no particle arrows — and no error.

**How to check.**

```bash
ros2 topic info /particle_cloud
```

**Cause.** `nav2_amcl` publishes `nav2_msgs/msg/ParticleCloud`. The stock
`rviz_default_plugins/PoseArray` display expects `geometry_msgs/PoseArray`. On a
type mismatch RViz renders nothing and reports nothing.

**Fix.** Use Nav2's own display, and match the QoS — AMCL publishes best-effort:

```yaml
- Class: nav2_rviz_plugins/ParticleCloud
  Topic:
    Reliability Policy: Best Effort
    Value: /particle_cloud
```

The cloud was there all along: 1354 particles.

---

## 12. AMCL waits forever for an initial pose

**Symptom.**

```
[amcl]: AMCL cannot publish a pose or update the transform. Please set the initial pose...
```

repeating, and no `map → odom` transform.

**Cause.** AMCL is a *tracking* filter and will not start without a starting
guess.

**Fix, interactive.** RViz's **2D Pose Estimate** button.

**Fix, automatic.** If the robot always starts at a known pose, seed it from the
launch file:

```python
{"set_initial_pose": True},
{"initial_pose.x": 0.0}, {"initial_pose.y": 0.0}, {"initial_pose.yaw": 0.0},
```

**If you publish `/initialpose` from your own script**, wait for AMCL's
subscription to match first and resend until it answers. A single publish right
after creating the publisher is dropped — DDS discovery has not finished — and
AMCL sits there logging the same line forever.

---

## 13. The robot cannot find itself after being moved

**Symptom.** Move the robot in Gazebo, or give a wrong pose with 2D Pose Estimate.
AMCL locks onto the wrong place and driving around does not fix it.

**This is by design, not a bug.** AMCL only spreads particles around its *current*
belief. If the robot is really 5 m away, there is no candidate there to score, so
every particle is equally wrong and the filter stays stuck. `recovery_alpha_slow`
and `recovery_alpha_fast` trickle in a few random particles, which is nowhere near
enough.

**Fix.** Scatter particles across the whole map:

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

Then drive a few metres. Measured:

| | position error |
|---|---|
| correctly seeded | 0.11 m |
| after being moved | **2.35 m** — stays lost |
| after global relocalization | **0.11 m** — recovers fully |

**Design note.** This only works if the map is **not symmetric**. A mirror-image
room gives global localization two equally good hypotheses and it cannot choose.
The course in this repo is deliberately asymmetric for that reason.

Raising the `recovery_alpha` values to get automatic recovery is possible but
degrades normal tracking — the filter keeps injecting noise even when it is doing
fine. Calling the service explicitly is the usual practice.

---

## 14. The robot grazes obstacles and the planner gets stuck

**Symptom.** The robot shaves past an obstacle corner, then the planner fails:

```
[planner_server]: GridBased plugin failed to plan from (1.90, 2.50) to (-3.00, -1.00):
"Failed to create plan with tolerance of: 0.500000"
```

**Cause.** The local costmap's only source was the live scan, and the LiDAR cannot
see closer than 0.5 m. An obstacle **disappears from the local costmap exactly as
the robot closes on it**. Once the robot is inside inflated space, the planner
cannot plan its way out.

**Fix.** Give the local costmap the saved map as well — it already knows where
every wall is:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
```

Then stop the planner cutting corners. `cost_scaling_factor` controls how fast
inflated cost decays: at 3.0 it collapses quickly and hugging edges stays cheap.

```yaml
inflation_radius: 0.75
cost_scaling_factor: 2.0
robot_radius: 0.30        # larger than the 0.24 m geometry, on purpose
```

The oversized radius is deliberate for a legged robot: a wheeled base that brushes
a wall keeps rolling, but a quadruped that catches a swinging leg trips and the
run is over.

Closest approach: **0.21 m → 0.70 m**.

**Related.** If the controller aborts with `Controller patience exceeded` while the
robot is in free space, raise `failure_tolerance` (0.3 s is tight — RPP flags
transient "collision ahead" while rotating in place to pick up a path behind it).

---

## 15. Phantom obstacles in open floor

**Symptom.** While the robot walks, costmap blobs appear in empty floor, the
planner detours around them, then they vanish and it goes back. Several times per
goal.

**How to check.**

```bash
# with the robot driving in another terminal
python3 unitree_go2_sim/tools/costmap_ghosts.py 120
```

Measured: **0** while standing still, **61 per costmap update** while walking.

> Count only cells with value `100`. In Nav2's published `OccupancyGrid`, `99` is
> the inscribed inflation shell and `100` is a real obstacle — counting 99 makes
> the ordinary inflation ring look like a thousand phantoms.

**Cause.** The same family as [#4](#4-the-map-never-grows-and-travels-with-the-robot),
but milder. `base_footprint` is kept level by the IMU, yet the correction lags
during a stride. A couple of degrees of residual tilt lifts *distant* floor
returns into the slice: at 2.5 m, 3.7° of tilt puts the floor at 0.16 m — just
over a 0.15 m threshold.

**Fix.** Raising `min_height` only pushes the onset further out (2.48 m → 3.37 m);
on a tilted slice the floor always catches the band eventually. Limit the
**range** instead, and only for the costmap:

```yaml
obstacle_max_range: 2.5    # was 10.0
raytrace_max_range: 8.0    # clearing still reaches further
```

AMCL keeps using the full scan for localization; the costmap marks only what is
close, and everything further out is already in the static layer.

| | phantoms per update | updates affected |
|---|---|---|
| original | 61 | 98 % |
| after | **1.1** | **10 %** |

Navigation got faster as a side effect — the controller was reacting to the
flicker.

**Trade-off.** Obstacles that are *not* on the map now go unmarked until the robot
is within 2.5 m. Fine for a static world; raise the range if you add moving
obstacles.

---

## 16. "Failed to bring up all requested nodes"

**Symptom.**

```
[global_costmap]: Timed out waiting for transform from base_footprint to map,
  tf error: Invalid frame ID "map" ... frame does not exist
[local_costmap]: Can't update static costmap layer, no map received
[lifecycle_manager]: Failed to bring up all requested nodes. Aborting bringup.
```

**Cause.** Navigation was started without localization. With no `map_server` there
is no `/map` and no `map` frame, so every costmap waits on a transform that never
arrives. Nothing in the error says "localization is not running".

**Fix.** Launch them together, localization first, with a delay before navigation
so `/map` and `map → odom` exist before the costmaps activate. See
`bringup_2d.launch.py`.

**Two launch-file traps found while building that file:**

- **Arguments leak between includes.** `launch_arguments` passed to
  `IncludeLaunchDescription` are plain `SetLaunchConfiguration` actions that write
  into the *parent* scope. Passing `rviz: "false"` to one include silently
  overwrote the top-level `rviz` argument for every later include. Wrap each
  include in `GroupAction(..., scoped=True)`.
- **Startup load can stall the bringup.** RViz loading its shaders and map at the
  same moment the lifecycle manager configures `controller_server` made the
  `change_state` service response time out, leaving `planner_server` unconfigured.
  Start RViz last, on its own timer, and raise `bond_timeout`.

---

## 17. The goal button does nothing

**Symptom.** Clicking the goal tool in RViz and dragging produces nothing, but
publishing the same pose from the terminal works:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: -2.5}, orientation: {w: 1.0}}}"
```

**Cause.** `nav2_rviz_plugins/GoalTool` does not publish to a topic at all — it
emits a Qt signal intended for `nav2_rviz_plugins`' Nav2 panel. Without that panel
loaded the button is inert.

**Fix.** Use the stock tool, which writes a `PoseStamped` straight to `/goal_pose`
and carries the familiar **2D Goal Pose** label:

```yaml
- Class: rviz_default_plugins/SetGoal
  Topic:
    Value: /goal_pose
```

`bt_navigator` subscribes to `/goal_pose` and starts navigating on receipt.

---

## 18. Measurements that contradict each other

**Symptom.** The same test gives different answers on consecutive runs. A topic
reports two different message rates. A node appears twice in `ros2 node list`.

**Cause.** Leftover processes from a previous launch. At one point this project
had **five `slam_toolbox` instances** running simultaneously, two of them
publishing `/scan` at different resolutions.

**Fix.**

```bash
bash unitree_go2_sim/tools/stop_sim.sh
```

It prints how many processes are left; you want `0`.

> The cleanup lives in a **file**, not inline. `pkill -f "ros2 launch"` matches the
> command line of the shell running it, so it kills the cleanup halfway through and
> leaves half the stack alive — which is exactly how those five instances
> accumulated.

**Other measurement traps met along the way**, all of which produced confident
wrong answers:

- **Driving blind into obstacles.** A test robot pressed against a block while
  odometry kept counting produced a fictitious "36 % odometry error". Tests now
  read clearance from the world definition and stop before contact.
- **Angle wraparound.** A 20 s turn covers 344°, which wraps past ±180°, making
  end-to-end comparison meaningless. Accumulate step by step instead.
- **Reading a map by eye.** A text preview first showed unknown space as free, then
  showed single pixels as walls. Score every occupied cell against the real
  geometry instead.
- **Wrong QoS on a probe.** Subscribing reliably to a best-effort publisher
  receives nothing, which looks exactly like a dead publisher.
- **Hypotheses that felt certain and were wrong.** The physics engine was blamed;
  it was innocent. A robot was assumed to have fallen; it was standing. Both were
  settled by measuring, which is the only reason they cost little.

---

## 19. Open floor on the far side of a wall

**Symptom.** The map is good everywhere except one patch, where free space
continues past a wall into ground the robot has never been able to reach. Often
it appears right after the robot bumps into something.

**Cause.** `pointcloud_to_laserscan` drops returns closer than `range_min`,
which is **0.5 m** here. Walk nearer than that and the wall stops being reported
at all, while `slam_toolbox` goes on marking the space each ray travelled through
as free. The wall is erased and the far side is painted as floor. Bumping the
wall is only how you get that close; the blind zone is what does the damage.

**Detect it.** The room is closed, so nothing outside it is reachable and every
cell out there should stay unknown:

```bash
python3 unitree_go2_sim/tools/score_map.py <map>.pgm <map>.yaml
```

Read the last line. `free cells outside: 0` is clean; anything else names the
leak's extent. Measured on the first hand-driven map: **91 cells, 0.23 m²**,
just outside the west wall.

Accuracy alone will not catch this. It asks "is what I drew really a wall" and
scores a leaking map at 96 %, because the leak adds free cells, not wrong ones.
That is why the same tool also reports surface coverage and the leak count.

**Fix.** Keep **0.6 m** or more between the robot and every surface while
mapping. `tour.py` enforces that by construction — it plans over the world
geometry inflated by `CLEAR_R`, so no part of the route can pass closer.

> Margin is not the same as plan. A tour planned at 0.85 m recorded a closest
> approach of **0.39 m**: a trotting quadruped slides sideways through turns, so
> the body does not go where the plan put it. Budget for roughly 0.3 m of drift.

---

## 20. The robot barely moves

**Symptom.** `/cmd_vel` is accepted, the legs step, the gait looks normal — and
the robot crosses a room in several minutes. Coverage tools look broken because
so little ground gets covered.

**Cause.** Commanded velocity and achieved velocity are very far apart at the low
end of this gait. Measured against `/odom/ground_truth`:

| Commanded | Achieved | Body tilt |
|---|---|---|
| 0.15 m/s | **0.021 m/s** | — |
| 0.25 m/s | 0.085 m/s | — |
| 0.35 m/s | 0.141 m/s | 12° |
| 0.50 m/s | 0.149 m/s | 13°, **fell over** |
| 0.70 m/s | 0.203 m/s | 15° |

At 0.15 the gait shuffles in place and delivers 14 % of what was asked.

**Fix.** Command **0.35**. It is the knee of the curve: 0.50 buys 5 % more ground
speed and put the robot on its back, and anything below 0.25 barely moves at all.
Ramp the command rather than stepping it — going from a standstill to full speed
in one tick is itself what tipped the robot.

> This one masqueraded as a different bug entirely. Two exploration runs totalling
> 12 minutes mapped only 70 % of the room and both ended a metre from where they
> started, which read as a broken exploration strategy. The strategy was fine; at
> 0.15 m/s the robot had travelled about six metres in total.
