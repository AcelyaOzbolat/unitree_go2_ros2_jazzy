import math, pathlib, sys, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# Resolve the world generator relative to this file, so the checkout can live
# anywhere. tools/ -> unitree_go2_sim/ -> repo root -> unitree_go2_description/tools
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "unitree_go2_description" / "tools"))
from gen_simple_room import OBSTACLES, aabb, RX, RY

def clearance(x, y):
    """Distance to the nearest obstacle or wall, and its name."""
    best, who = min(
        ((math.hypot(max(x0-x, x-x1, 0), max(y0-y, y-y1, 0)), o[0])
         for o in OBSTACLES for (x0, y0, x1, y1) in [aabb(o)]),
        key=lambda t: t[0])
    wall = min(RX-abs(x), RY-abs(y))
    return (best, who) if best < wall else (wall, "duvar")

rclpy.init(); n = rclpy.create_node("falltest")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"m": None}
n.create_subscription(Odometry, "/odom/ground_truth", lambda m: st.__setitem__("m", m), 10)
t0 = time.time()
while st["m"] is None and time.time()-t0 < 20: rclpy.spin_once(n, timeout_sec=0.2)
if st["m"] is None:
    print("no ground-truth odometry - is the simulation running?")
    rclpy.shutdown(); sys.exit(2)

COURSE = []
for _ in range(6):
    COURSE += [(0.15, 0.0, 22), (0.0, 0.0, 3), (0.0, 0.25, 8), (0.0, 0.0, 3)]

start = time.time()
for lin, ang, secs in COURSE:
    msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
    t_end = time.time() + secs
    while time.time() < t_end:
        pub.publish(msg); rclpy.spin_once(n, timeout_sec=0.05)
        p = st["m"].pose.pose.position; q = st["m"].pose.pose.orientation
        roll = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
        if abs(roll) > 60:
            d, who = clearance(p.x, p.y)
            print(f"FELL OVER  t={time.time()-start:.0f}s  x={p.x:.2f} y={p.y:.2f} roll={roll:.0f}")
            print(f"  nearest obstacle: '{who}' at {d:.2f} m")
            print(f"  >>> {'COLLISION (within 0.4 m)' if d < 0.4 else 'FELL IN OPEN GROUND'}")
            pub.publish(Twist()); rclpy.shutdown(); sys.exit(1)
p = st["m"].pose.pose.position
print(f"STAYED UP  t={time.time()-start:.0f}s  x={p.x:.2f} y={p.y:.2f}")
rclpy.shutdown()
