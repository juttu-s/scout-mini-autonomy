"""Offline scenario tests for scout_safety/safety_node.py, no camera or robot needed.

    roscore &
    rosrun tf2_ros static_transform_publisher 0.3 0 0.2 -1.5708 0 -1.5708 base_link camera_color_optical_frame &
    rosrun scout_safety safety_node.py _max_speed:=0.5 &
    python3 test/scenario_test.py        # prints PASS/FAIL per scenario
Use a separate master if a real robot is running: export ROS_MASTER_URI=http://localhost:11312 and
start that roscore with -p 11312 (the node publishes /cmd_vel).
"""
import json, time, numpy as np, rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from rs_object_detection.msg import TrackedObjectArray, TrackedObject

rospy.init_node("safety_test")
K = np.array([[600, 0, 424], [0, 600, 240], [0, 0, 1.0]])
pub3 = rospy.Publisher("/rs_detector/detections", Detection3DArray, queue_size=1)
pub2 = rospy.Publisher("/rs_detector/detections_2d", Detection2DArray, queue_size=1)
pubt = rospy.Publisher("/rs_detector/tracks", TrackedObjectArray, queue_size=1)
pubi = rospy.Publisher("/camera/camera/color/camera_info", CameraInfo, queue_size=1, latch=True)
tele = rospy.Publisher("/teleop/cmd_vel", Twist, queue_size=1)
nav = rospy.Publisher("/nav/cmd_vel", Twist, queue_size=1)
pubi.publish(CameraInfo(header=rospy.Header(frame_id="camera_color_optical_frame"), height=480, width=848, K=list(K.ravel())))
out, state = [None], [None]
rospy.Subscriber("/cmd_vel", Twist, lambda m: out.__setitem__(0, m))
rospy.Subscriber("/scout_safety/state", String, lambda m: state.__setitem__(0, json.loads(m.data)))

sc = dict(detect=False, people=[], nodepth=[], tracks=[], tele=None, nav=None)
def base_to_opt(x, y, z=0.0):  # base_link point -> camera optical (camera at x=0.3, z=0.2, facing forward)
    return np.array([-(y - 0.0), -(z - 0.2), x - 0.3])
def tick(_):
    now = rospy.Time.now()
    if sc["tele"] is not None: tele.publish(Twist(linear=type(Twist().linear)(sc["tele"][0], 0, 0), angular=type(Twist().angular)(0, 0, sc["tele"][1])))
    if sc["nav"] is not None: nav.publish(Twist(linear=type(Twist().linear)(sc["nav"][0], 0, 0), angular=type(Twist().angular)(0, 0, sc["nav"][1])))
    if not sc["detect"]: return
    if sc.get("flicker"):  # person jumps in and out of the corridor every 0.1 s
        sc["people"] = [(1.81, 0.0)] if int(time.time() * 10) % 2 else [(1.81, 1.0)]
    h = rospy.Header(stamp=now, frame_id="camera_color_optical_frame")
    a3, a2 = Detection3DArray(header=h), Detection2DArray(header=h)
    for (x, y) in sc["people"]:
        q = base_to_opt(x, y, 0.9)
        d = Detection3D(header=h); hyp = ObjectHypothesisWithPose(id=0, score=0.9)
        d.results.append(hyp); d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = q
        a3.detections.append(d)
        d2 = Detection2D(header=h); d2.results.append(ObjectHypothesisWithPose(id=0, score=0.9))
        d2.bbox.center.x, d2.bbox.center.y = K[0, 0] * q[0] / q[2] + K[0, 2], K[1, 1] * q[1] / q[2] + K[1, 2]; d2.bbox.size_y = 200
        a2.detections.append(d2)
    for hfrac in sc["nodepth"]:
        d2 = Detection2D(header=h); d2.results.append(ObjectHypothesisWithPose(id=0, score=0.9))
        d2.bbox.center.x, d2.bbox.center.y, d2.bbox.size_y = 100, 240, hfrac * 480
        a2.detections.append(d2)
    ta = TrackedObjectArray(); ta.header.stamp, ta.header.frame_id = now, "base_link"
    for i, (x, y, vx, vy) in enumerate(sc["tracks"]):
        o = TrackedObject(id=i + 1, class_id=0, has_position=True); o.position.x, o.position.y = x, y; o.velocity.x, o.velocity.y = vx, vy
        ta.objects.append(o)
    pub3.publish(a3); pub2.publish(a2); pubt.publish(ta)
rospy.Timer(rospy.Duration(1 / 30.0), tick)

results = []
def check(name, wait, exp_state, exp_lin, exp_ang=None, tol=0.02, **changes):
    sc.update(changes); time.sleep(wait)
    o, st = out[0], state[0]
    ok = st["state"] == exp_state and abs(o.linear.x - exp_lin) <= tol and (exp_ang is None or abs(o.angular.z - exp_ang) <= tol)
    results.append(ok)
    print("%-4s %-46s state %-5s lin %+.2f ang %+.2f  (want %s %+.2f%s)  %s" % ("PASS" if ok else "FAIL", name, st["state"], o.linear.x, o.angular.z,
          exp_state, exp_lin, "" if exp_ang is None else " ang %+.2f" % exp_ang, st["reason"]))

time.sleep(1.0)
check("1 no detector output -> FAULT", 1.0, "FAULT", 0.0, 0.0, tele=(0.4, 0.3))
check("2 detector alive, nobody -> full speed", 1.6, "OK", 0.4, 0.3, detect=True)
check("3 person 3 m ahead -> full speed", 0.5, "OK", 0.4, people=[(3.31, 0.0)])
check("4 person 1.5 m ahead -> scaled", 0.5, "SLOW", 0.25, tol=0.04, people=[(1.81, 0.0)])
check("5 person 0.6 m ahead -> stop, turning ok", 0.5, "STOP", 0.0, 0.3, people=[(0.91, 0.0)])
check("6 reverse away while stopped -> capped -0.2", 0.5, "STOP", -0.2, tele=(-0.5, 0.0))
check("7 person beside the path -> no limit", 1.0, "OK", 0.4, people=[(1.81, 1.0)], tele=(0.4, 0.3))
check("8 person next to robot -> no turning either", 0.5, "STOP", 0.0, 0.0, people=[(0.5, 0.3)])
check("9 nav 0.3 + teleop 0.1 -> teleop wins", 1.0, "OK", 0.1, people=[], nav=(0.3, 0.0), tele=(0.1, 0.0))
check("10 teleop goes silent -> nav after timeout", 1.0, "OK", 0.3, tele=None)
check("11 big person box without depth -> stop", 0.5, "STOP", 0.0, nodepth=[0.62])
check("12 small person box without depth -> slow", 1.0, "SLOW", 0.1, nodepth=[0.3])
check("13 person 3 m ahead running at robot -> TTC stop", 0.5, "STOP", 0.0, nodepth=[], people=[(3.31, 0.0)], tracks=[(3.31, 0.0, -2.0, 0.0)])
check("14 STOP is held for clear_time after clearing", 0.25, "STOP", 0.0, people=[], tracks=[])
check("15 ...then released", 0.6, "OK", 0.3)
check("16 person standing 1.5 m to the side, not on path -> no TTC stop", 0.5, "OK", 0.3, people=[(3.0, 1.5)], tracks=[(3.0, 1.5, 0.0, 0.0)])
check("17 person flickering in/out of the corridor -> steady SLOW", 1.0, "SLOW", 0.25, tol=0.04, people=[], tracks=[], flicker=True)
states = []
sub = rospy.Subscriber("/scout_safety/state", String, lambda m: states.append(json.loads(m.data)["state"]))
time.sleep(1.0); sub.unregister()
ok = len(states) > 10 and all(x == "SLOW" for x in states)
results.append(ok); print("%-4s %-46s %d state msgs, all SLOW: %s" % ("PASS" if ok else "FAIL", "18 ...and it never toggles back to OK", len(states), ok))
check("19 detector stalls -> FAULT", 0.4, "FAULT", 0.0, flicker=False, detect=False)
rogue = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # a node driving the robot around the safety layer
time.sleep(2.6)
ok = "/safety_test" in (state[0].get("bypass") or [])
results.append(ok); print("%-4s %-46s bypass reported: %s" % ("PASS" if ok else "FAIL", "20 rogue /cmd_vel publisher is reported", state[0].get("bypass")))
if False: check("17 detector stalls -> FAULT", 0.4, "FAULT", 0.0, detect=False)
print("%d/%d passed" % (sum(results), len(results)))
