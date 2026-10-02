# Diagnostic tools

These are **not automated tests**. They are diagnostics you run by hand. `colcon
test` does not run them, every one of them needs a **running simulation**, and
some of them drive the robot.

```bash
# start the simulation first
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false
# wait for the controllers to activate (~35 s), then:
python3 unitree_go2_sim/tools/<tool>.py
```

## World dependency — important

`odom_quality`, `check_scan`, `score_map`, `align_score`, `explore`, `tour` and
`gait_stability` know the geometry of the `simple_room.sdf` course. They read it
**live** from the definition in `unitree_go2_description/tools/gen_simple_room.py`,
so if you change the course through that generator, the tools follow by themselves.

But **in a different world** — an SDF you wrote, an environment you downloaded —
these tools quietly produce meaningless numbers, because they go on measuring
against simple_room's obstacles. Only `scan_probe` and `crop_map` stay valid in
that case.

---

## The tools

### `tour.py` — planned coverage tour, for mapping
Walks the room end to end in a lawnmower pattern. It inflates the world geometry
by 0.85 m, lays waypoints on the result, and plans between them with A*.

Use this rather than `explore.py` when you are building a map. `explore` works by
reflex and **cannot close a room**: over two measured runs totalling twelve
minutes it mapped only 70 % of the real surface, and both runs ended within a
metre of where they started.

The margin is not only about avoiding collisions. `range_min` is 0.5 m, so returns
from anything closer are dropped and slam_toolbox goes on clearing the rays that
travelled through it — the wall is erased and the far side is painted as floor.
Building the margin into the plan makes that impossible by construction rather
than by luck.

```bash
python3 tour.py          # full tour (~13 min)
python3 tour.py 600      # stop after 600 s, wherever it has got to
python3 tour.py --fill   # only the pockets the full tour had to skip
```

**The speeds were measured, not guessed:**

| Commanded | Achieved | Body tilt |
|---|---|---|
| 0.15 m/s | 0.021 m/s | — |
| 0.25 m/s | 0.085 m/s | — |
| **0.35 m/s** | **0.141 m/s** | **12°** |
| 0.50 m/s | 0.149 m/s | 13° — *fell over* |
| 0.70 m/s | 0.203 m/s | 15° |

At 0.15 the gait shuffles in place and delivers 14 % of what was asked. `explore.py`
used that speed, which is why 300 s of exploring covered about six metres of
ground — what looked like a coverage problem was a speed problem. 0.50 is not the
answer either: it put the robot on its back. 0.35 gives 95 % of the ground speed
with a much calmer gait. There is also a `RAMP`: stepping from a standstill to
full speed in one tick was itself what tipped the robot.

Measured: 36 waypoints, 779 s, no contact, no falls, closest approach 0.49 m.

### `explore.py` — wander without touching anything
Drives the robot around for gait testing. It reads the clearance ahead from the
world definition and turns towards whichever side is more open. At the end it
reports the smallest clearance it saw and whether anything was hit.

```bash
python3 explore.py 250      # seconds
```

### `odom_quality.py` — odometry against truth
Measures the gap between `/odom` (the EKF output) and `/odom/ground_truth`
(Gazebo's truth). Straight walking and turning in place are reported separately,
because a legged robot behaves very differently in the two.

Guarded against two traps: it stops 1 m short of an obstacle (a robot leaning on
something while odometry keeps counting invents an error) and it accumulates the
turn step by step (a 20 s turn covers 344°, which wraps past ±180° and ruins any
start-to-end comparison).

Typical measured values: **0.3–0.5 % turning**, **2–18 % walking straight**. The
translation error grows with distance — 1.5 % at 1.1 m, 18 % at 1.9 m — and varies
a lot between runs depending on foot slip, so do not read one run as a verdict.

A turning error above 5 % is a real problem: compare `/odom/raw` (CHAMP's leg
odometry), `/imu/data` and `/odom` together and find where it starts. Translation
error is correctable by SLAM; `odom_scaler` in `gait.yaml` is the knob if you want
to reduce it at the source.

### `amcl_quality.py` — localization against truth
Compares AMCL's estimate with Gazebo's truth. It drives the robot without hitting
anything and reports position and heading error alongside the **spread of the
particle cloud**.

The spread is the most important signal: the cloud should **tighten** over time. If
it widens the filter cannot gain confidence, and the `alpha` values are usually too
high. If it tightens but the position error stays large, the filter is confident
and wrong — the opposite problem.

```bash
python3 amcl_quality.py 120      # seconds
```

Measured on this robot: position median **0.11–0.19 m**, heading median **4°**,
spread 0.63 → 0.37. The `alpha` values were set from measured odometry — low on
the rotation terms (heading is accurate), high on the translation term (the weak
axis).

### `kidnap_recovery.py` — the kidnapped robot experiment
Tests AMCL in three stages: a correct seed, a wrong seed (kidnapped), then
`/reinitialize_global_localization`. It measures the position error at each stage.

Measured on this robot: **0.11 m** with a correct seed, **2.35 m** after the kidnap
(it does not recover on its own), **0.11 m** again after global localization.

```bash
python3 kidnap_recovery.py
```

### `nav_test.py` — does Nav2 reach the goal
Sends two goals and measures each against truth: time, distance travelled, plan
length, how many times it replanned, goal error, **distance to the nearest
obstacle**, and whether the robot stayed upright.

Reporting plan length separately from distance travelled is deliberate: if it
cannot follow a short plan the problem is in the controller, and if it follows a
long plan faithfully the problem is in the planner or the costmap.

It checks that a goal is in open space before sending it — a goal inside an
obstacle produces a failure that looks like a navigation fault and is not.

```bash
python3 nav_test.py
```

Measured on this robot: both goals SUCCEEDED, nearest obstacle **0.70–0.75 m**,
goal error 0.20–0.56 m. The path is still about three times the straight line.

### `costmap_ghosts.py` — counting phantom obstacles
Counts cells marked occupied in the costmap where the world is actually empty.
The measurable form of those cyan patches that appear and vanish in open floor.

It samples body tilt too, because the likeliest cause is the laser slice tipping
and catching distant floor returns.

```bash
# run it while driving the robot from another terminal
python3 costmap_ghosts.py 120
```

Measured on this robot: **0** standing still; **61** on average while walking (in
98 % of updates) before tuning, **1.1** (in 10 %) after `obstacle_max_range` was
brought down to 2.5 m.

Note: the threshold must be `100`, not `99`. In Nav2's published costmap 99 is the
inflation shell and 100 is a real obstacle. Counting 99 reports ordinary inflation
as thousands of phantoms.

### `check_scan.py` — the scan against the known world
Ray-casts the known obstacle geometry from the robot's **true** pose and compares
it with `/scan`. It settles the difference between "the sensor data is wrong" and
"SLAM is mishandling good data", which is the first thing to establish when
mapping misbehaves.

Healthy: more than 90 % of rays within 10 cm.

### `score_map.py` — a saved map against truth
Scores a saved map three ways. Far more reliable than looking at it; an ASCII
preview misleads easily.

```bash
python3 score_map.py <map>.pgm <map>.yaml
```

**1. Accuracy** — how far each occupied cell sits from a real surface.
Healthy: more than 85 % of cells within 2 cells (10 cm).

**2. Surface coverage** — how much of the real surface got drawn at all.
Accuracy alone was not enough: it asks "is what I drew really a wall", not "did I
draw all of the wall". A map with a hole ray-traced straight through it still
scores well on the first measure alone.

**3. Free cells outside the room** — space marked free behind a wall.
The room is closed and nothing outside it is reachable, so every free cell out
there means a wall was erased. The likeliest cause is `range_min`: pass closer
than 0.5 m and those returns are dropped while SLAM keeps clearing the rays that
went through, so the wall disappears and the far side becomes floor. The fix is
to keep away from walls while mapping — `tour.py` already leaves a 1 m margin.

Measured: the first hand-driven map leaked 91 cells (0.23 m²).

### `align_score.py` — is the map rotated?
Run it when `score_map` gives a low score. It tries small rotations and shifts
looking for the best alignment. If the score rises at the best alignment the map
is **shifted**; if it does not, the map is **internally distorted** — two very
different problems.

```bash
python3 align_score.py <map>.pgm <map>.yaml
```

### `gait_stability.py` — the falling-over test
Drives a fixed course and reports whether the robot went over, and if so **where**
and **how far from the nearest obstacle**. That distinction matters: falling in
open ground is a gait tuning problem, falling within 0.4 m of an obstacle is a
collision.

### `scan_probe.py` — quick health check
A one-shot summary of the cloud, the scan and the map: point count, valid-ray
ratio, range limits, map size. Works independently of the world.

### `crop_map.py` — trim the map
Removes the completely unexplored border from a saved map. It shifts `origin` as
it crops pixels — which is essential, or the map silently moves in the world.

```bash
python3 crop_map.py input.pgm input.yaml output_basename
```

### `stop_sim.sh` — stop everything
Kills every process belonging to the simulation. It lives in an executable file so
that it does not match its own command line — doing the same job inline with
`pkill -f` kills the shell running it and leaves the cleanup half finished.

The need for it is real: leftover processes quietly corrupt measurements. At one
point five `slam_toolbox` instances were running at once, publishing to `/scan` at
two different resolutions.

```bash
bash unitree_go2_sim/tools/stop_sim.sh
```

---

## Order to work through when something is wrong

1. `stop_sim.sh` — make sure nothing is left over, then start one stack
2. `scan_probe.py` — is data flowing, is the map growing
3. `check_scan.py` — is the scan correct? If not, do not look at SLAM, look at the
   sensor chain
4. `odom_quality.py` — is odometry correct? Turning error comes after scan error
5. drive with `tour.py`, save, and score it with `score_map.py`
6. if the score is low, `align_score.py` — shifted, or distorted?
