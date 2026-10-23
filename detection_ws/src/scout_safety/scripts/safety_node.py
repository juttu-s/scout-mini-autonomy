#!/usr/bin/env python3
"""Person-aware speed and separation monitor for the Scout Mini.

Sits between the command sources (teleop, navigation) and the robot: it is the only publisher of
~output_topic (/cmd_vel). Each cycle it picks the highest-priority fresh input command and limits it using
the people seen by rs_object_detection:

  OK     no person limits the motion
  SLOW   a person is in the slow zone ahead (forward speed scaled down), or seen without depth
  STOP   a person is inside the stop distance ahead, closing in fast (time to collision), or very close
         without depth: forward motion blocked; turning allowed unless someone is next to the robot
  FAULT  detector output is stale or missing, or the camera transform is unknown: all motion zero

Distances are clearances from the robot's front (base_link x minus ~front_offset). The stop distance grows
with speed: stop_distance + v * reaction_time + v^2 / (2 * decel).

This is a monitoring layer, not a certified safety function: keep the hardware e-stop within reach.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rs_object_detection.msg import TrackedObjectArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf.transformations import quaternion_matrix
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

OK, SLOW, STOP, FAULT = "OK", "SLOW", "STOP", "FAULT"
SEVERITY = {OK: 0, SLOW: 1, STOP: 2, FAULT: 3}


class Heartbeat:
    """Touches ~heartbeat_file at most once a second while the control loop runs: a container health check reads
    the file's age, which is far cheaper than spawning a ROS node per probe. Empty path (default): off."""

    def __init__(self, path):
        self.path, self.last = path, 0.0
        if path:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

    def beat(self):
        now = time.monotonic()
        if self.path and now - self.last >= 1.0:
            self.last = now
            try:
                with open(self.path, "a"):
                    os.utime(self.path, None)
            except OSError:
                pass


class CommandInput:
    """One velocity command source with a priority and a timeout."""

    def __init__(self, topic, priority, timeout):
        self.topic, self.priority, self.timeout = topic, priority, timeout
        self.msg, self.stamp = None, None
        rospy.Subscriber(topic, Twist, self._cb, queue_size=1)

    def _cb(self, msg):
        self.msg, self.stamp = msg, rospy.get_time()

    def fresh(self, now):
        return self.msg is not None and now - self.stamp <= self.timeout


class SafetyMonitor:
    def __init__(self):
        p = rospy.get_param
        self.base_frame = p("~base_frame", "base_link")
        self.classes = set(p("~classes", [0]))
        self.rate = p("~rate", 50.0)
        self.heartbeat = Heartbeat(p("~heartbeat_file", ""))
        # Speed caps
        self.max_speed = p("~max_speed", 0.5)
        self.max_reverse_speed = p("~max_reverse_speed", 0.2)
        self.max_angular = p("~max_angular", 1.0)
        self.min_slow_speed = p("~min_slow_speed", 0.1)
        # Zones (clearances from the robot's front, metres)
        self.front_offset = p("~front_offset", 0.31)
        self.half_width = p("~half_width", 0.30)
        self.lateral_margin = p("~lateral_margin", 0.30)
        self.stop_distance = p("~stop_distance", 0.8)
        self.slow_distance = p("~slow_distance", 2.0)
        self.footprint_radius = p("~footprint_radius", 0.42)
        self.turn_margin = p("~turn_margin", 0.30)
        self.reaction_time = p("~reaction_time", 0.3)
        self.decel = p("~decel", 1.0)
        self.ttc_stop = p("~ttc_stop", 1.5)
        self.no_depth_stop_height = p("~no_depth_stop_height", 0.5)
        # Health / hysteresis
        self.max_detection_age = p("~max_detection_age", 0.3)
        self.recover_time = p("~recover_time", 1.0)
        self.clear_time = p("~clear_time", 0.5)

        self.inputs = [CommandInput(i["topic"], i.get("priority", 0), i.get("timeout", 0.5))
                       for i in p("~inputs", [{"topic": "/teleop/cmd_vel", "priority": 10, "timeout": 0.5},
                                             {"topic": "/nav/cmd_vel", "priority": 5, "timeout": 0.5}])]
        self.inputs.sort(key=lambda i: -i.priority)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.T_base_cam = None  # camera mount is fixed: looked up once
        self.K, self.img_h = None, None
        self.det3d = self.det2d = self.tracks = None
        self.odom_speed = 0.0
        self.last_out = Twist()
        self.state, self.reason = FAULT, "starting"
        self.recent = deque()  # (time, state, reason, limit, allow_turn) of the last clear_time seconds
        self.healthy_since = None

        det_ns = p("~detector_ns", "/rs_detector")
        rospy.Subscriber(det_ns + "/detections", Detection3DArray, lambda m: setattr(self, "det3d", m), queue_size=1)
        rospy.Subscriber(det_ns + "/detections_2d", Detection2DArray, lambda m: setattr(self, "det2d", m),
                         queue_size=1)
        rospy.Subscriber(det_ns + "/tracks", TrackedObjectArray, lambda m: setattr(self, "tracks", m), queue_size=1)
        rospy.Subscriber(p("~camera_info_topic", "/camera/camera/color/camera_info"), CameraInfo, self._info_cb,
                         queue_size=1)
        rospy.Subscriber(p("~odom_topic", "/odom"), Odometry, self._odom_cb, queue_size=1)

        self.output_topic = rospy.resolve_name(p("~output_topic", "/cmd_vel"))
        self.pub_cmd = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.bypass = []  # other nodes publishing output_topic directly (they bypass this node)
        rospy.Timer(rospy.Duration(2.0), self.check_bypass)
        self.pub_state = rospy.Publisher("~state", String, queue_size=5)
        self.pub_markers = rospy.Publisher("~markers", MarkerArray, queue_size=2)
        rospy.loginfo("Safety monitor: inputs %s -> %s", [(i.topic, i.priority) for i in self.inputs],
                      p("~output_topic", "/cmd_vel"))
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.step)

    def check_bypass(self, _event):
        """Find nodes other than this one that publish the robot command topic directly."""
        try:
            pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception:  # master unreachable: nothing to report
            return
        me = rospy.get_name()
        self.bypass = sorted(n for t, nodes in pubs if t == self.output_topic for n in nodes if n != me)
        if self.bypass:
            rospy.logerr("%s also publishes %s directly and BYPASSES the safety layer; remap it to one of %s",
                         ", ".join(self.bypass), self.output_topic, [i.topic for i in self.inputs])

    def _info_cb(self, msg):
        self.K = np.array(msg.K).reshape(3, 3)
        self.img_h = float(msg.height)

    def _odom_cb(self, msg):
        self.odom_speed = msg.twist.twist.linear.x

    # ------------------------------------------------------------------ perception
    def camera_tf(self, cam_frame):
        if self.T_base_cam is None:
            try:
                tr = self.tf_buffer.lookup_transform(self.base_frame, cam_frame, rospy.Time(0)).transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return None
            T = quaternion_matrix([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
            T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            self.T_base_cam = T
        return self.T_base_cam

    def people(self, T):
        """[(x, y) in base_frame] for people with depth, and [box height fraction] for people without."""
        with_depth, centers = [], []
        for d in self.det3d.detections:
            if d.results and d.results[0].id in self.classes:
                q = d.bbox.center.position
                x, y, _ = T[:3, :3] @ np.array([q.x, q.y, q.z]) + T[:3, 3]
                with_depth.append((x, y))
                if self.K is not None and q.z > 0:  # pixel center this 3D position came from
                    centers.append((self.K[0, 0] * q.x / q.z + self.K[0, 2], self.K[1, 1] * q.y / q.z + self.K[1, 2]))
        no_depth = []
        if self.det2d is not None and self.det2d.header.stamp == self.det3d.header.stamp and self.K is not None:
            img_h = self.img_h
            for d in self.det2d.detections:
                if not d.results or d.results[0].id not in self.classes:
                    continue
                c = d.bbox.center
                if not any(abs(c.x - u) < 1.0 and abs(c.y - v) < 1.0 for u, v in centers):
                    no_depth.append(d.bbox.size_y / img_h)
        return with_depth, no_depth

    def min_ttc(self, robot_speed):
        """Smallest time until a tracked person's path, relative to the robot, enters the safety circle
        (radius footprint_radius + turn_margin). People whose relative path misses the circle are ignored,
        so walking past the robot doesn't stop it."""
        if self.tracks is None or self.tracks.header.frame_id != self.base_frame:
            return float("inf"), None
        if rospy.get_time() - self.tracks.header.stamp.to_sec() > self.max_detection_age:
            return float("inf"), None
        R = self.footprint_radius + self.turn_margin
        best, who = float("inf"), None
        for o in self.tracks.objects:
            if o.class_id not in self.classes or not o.has_position:
                continue
            p = np.array([o.position.x, o.position.y])
            v = np.array([o.velocity.x - robot_speed, o.velocity.y])  # person velocity relative to the robot
            # Solve |p + v t| = R for the first t >= 0
            a, b, c = float(v @ v), float(p @ v), float(p @ p) - R * R
            if c <= 0.0:
                ttc = 0.0
            elif a < 1e-6 or b >= 0.0 or b * b - a * c < 0.0:
                continue  # not moving relative to the robot, moving away, or passing outside the circle
            else:
                ttc = (-b - math.sqrt(b * b - a * c)) / a
            if ttc < best:
                best, who = ttc, o.id
        return best, who

    # ------------------------------------------------------------------ control loop
    def step(self, _event):
        self.heartbeat.beat()
        now = rospy.get_time()
        cmd = next((i.msg for i in self.inputs if i.fresh(now)), None)
        source = next((i.topic for i in self.inputs if i.fresh(now)), None)
        # Signed forward speed: measured, or what was last commanded if that is larger (not yet reached)
        robot_speed = max(self.odom_speed, self.last_out.linear.x, key=abs)

        state, reason, fwd_limit, allow_turn, info = self.evaluate(now, robot_speed)
        # Apply the most restrictive result of the last clear_time seconds, so a person flickering in and out
        # of a zone (box dropped for a frame, position jittering at the corridor edge) doesn't toggle the speed
        if state == FAULT:
            self.recent.clear()
        else:
            self.recent.append((now, state, reason, fwd_limit, allow_turn))
            while self.recent and now - self.recent[0][0] > self.clear_time:
                self.recent.popleft()
            held = min(self.recent, key=lambda r: (r[3], r[4], -SEVERITY[r[1]]))
            if held[3] < fwd_limit or (allow_turn and not held[4]):
                state, reason = held[1], "holding (%s)" % held[2]
                fwd_limit, allow_turn = held[3], held[4] and allow_turn

        out = Twist()
        if cmd is not None and state != FAULT:
            out.linear.x = min(cmd.linear.x, fwd_limit) if cmd.linear.x > 0 else max(cmd.linear.x,
                                                                                    -self.max_reverse_speed)
            out.angular.z = float(np.clip(cmd.angular.z, -self.max_angular, self.max_angular)) if allow_turn else 0.0
        self.pub_cmd.publish(out)
        self.last_out = out

        if state != self.state:
            log = rospy.logwarn if state in (STOP, FAULT) else rospy.loginfo
            log("Safety %s -> %s: %s", self.state, state, reason)
        self.state, self.reason = state, reason
        info.update(state=state, reason=reason, source=source, bypass=self.bypass, limit=round(fwd_limit, 3),
                    cmd_in=[round(cmd.linear.x, 3), round(cmd.angular.z, 3)] if cmd else None,
                    cmd_out=[round(out.linear.x, 3), round(out.angular.z, 3)])
        self.pub_state.publish(String(data=json.dumps(info)))
        self.publish_markers(state, info.get("d_stop", self.stop_distance))

    def evaluate(self, now, v):
        """(state, reason, forward speed limit, turning allowed, info) for the current perception."""
        info = {}
        if self.det3d is None:
            return self._fault(now, "no detector output yet")
        age = now - self.det3d.header.stamp.to_sec()
        info["det_age"] = round(age, 3)
        if age > self.max_detection_age:
            return self._fault(now, "detections %.2f s old" % age, info)
        T = self.camera_tf(self.det3d.header.frame_id)
        if T is None:
            return self._fault(now, "no TF %s -> %s" % (self.det3d.header.frame_id, self.base_frame), info)
        if self.healthy_since is None:
            self.healthy_since = now
        if now - self.healthy_since < self.recover_time:
            return FAULT, "recovering", 0.0, False, info

        vf = max(v, 0.0)  # braking distance only matters when driving forward
        d_stop = self.stop_distance + vf * self.reaction_time + vf * vf / (2.0 * self.decel)
        info["d_stop"] = round(d_stop, 3)
        with_depth, no_depth = self.people(T)

        state, reason, limit, allow_turn = OK, "clear", self.max_speed, True
        nearest = float("inf")
        for x, y in with_depth:
            r = math.hypot(x, y)
            if r - self.footprint_radius < self.turn_margin:
                return STOP, "person %.2f m from the robot" % r, 0.0, False, dict(info, nearest=round(r, 2))
            if x > 0 and abs(y) < self.half_width + self.lateral_margin:
                nearest = min(nearest, x - self.front_offset)
        if nearest < float("inf"):
            info["nearest"] = round(nearest, 2)
            if nearest <= d_stop:
                state, reason, limit = STOP, "person %.2f m ahead (stop %.2f m)" % (nearest, d_stop), 0.0
            elif nearest < self.slow_distance:
                frac = (nearest - d_stop) / max(self.slow_distance - d_stop, 1e-3)
                limit = max(self.min_slow_speed, self.max_speed * min(frac, 1.0))
                state, reason = SLOW, "person %.2f m ahead" % nearest

        if no_depth:
            info["no_depth"] = [round(h, 2) for h in no_depth]
            if max(no_depth) >= self.no_depth_stop_height:
                return STOP, "person very close without depth", 0.0, False, info
            if state == OK:
                state, reason, limit = SLOW, "person without depth", self.min_slow_speed

        ttc, who = self.min_ttc(v)
        if ttc < float("inf"):
            info["ttc"] = round(ttc, 2)
        if ttc < self.ttc_stop and state != STOP:
            state, reason, limit = STOP, "person #%d closing, time to collision %.1f s" % (who, ttc), 0.0
        return state, reason, min(limit, self.max_speed), allow_turn, info

    def _fault(self, now, reason, info=None):
        self.healthy_since = None
        return FAULT, reason, 0.0, False, info or {}

    def publish_markers(self, state, d_stop):
        col = {OK: (0.1, 0.8, 0.1), SLOW: (1.0, 0.8, 0.0), STOP: (1.0, 0.1, 0.1), FAULT: (0.6, 0.0, 0.8)}[state]
        w = self.half_width + self.lateral_margin
        arr = MarkerArray()
        for i, (dist, alpha) in enumerate([(d_stop, 0.9), (self.slow_distance, 0.5)]):
            m = Marker(ns="zones", id=i, type=Marker.LINE_STRIP, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.base_frame, rospy.Time.now()
            x0, x1 = self.front_offset, self.front_offset + dist
            m.points = [Point(x0, -w, 0), Point(x1, -w, 0), Point(x1, w, 0), Point(x0, w, 0), Point(x0, -w, 0)]
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = col[0], col[1], col[2], alpha
            m.pose.orientation.w = 1.0
            arr.markers.append(m)
        self.pub_markers.publish(arr)


def main():
    rospy.init_node("scout_safety")
    SafetyMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
