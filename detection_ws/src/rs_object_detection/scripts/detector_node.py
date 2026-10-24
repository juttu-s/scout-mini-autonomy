#!/usr/bin/env python3
"""YOLO object detection on a RealSense color stream with 3D localization from aligned depth.

Subscribes to the color image and a depth image, runs YOLO (~backend: "ultralytics" with a .pt or
TensorRT .engine, or "deepstream": NVIDIA DeepStream nvinfer + nvtracker, see ds_pipeline.py), and for each detection estimates the object's 3D position in the color
optical frame from the median depth inside the box. With ~tracker set, detections are tracked
across frames (ByteTrack) and each track gets a stable id and a smoothed 3D velocity. With
~reid_model set, people who leave the view (or the camera turns away) get their old id back when
they reappear, by matching appearance embeddings against recently lost identities.

Depth comes either from the raw depth stream, registered into the color frame on the GPU here
(~register_depth: true, camera launched with align_depth:=false), or from the driver's
aligned_depth_to_color topic (~register_depth: false). The driver's CPU alignment in
librealsense 2.50 throttles and bursts the color stream, so in-node registration is the default.

Publishes (all in the node's private namespace):
  ~detections      vision_msgs/Detection3DArray
  ~detections_2d   vision_msgs/Detection2DArray
  ~image_annotated sensor_msgs/Image
  ~image_annotated/compressed sensor_msgs/CompressedImage (JPEG; use this over a network, e.g. Foxglove)
  ~markers         visualization_msgs/MarkerArray
  ~tracks          rs_object_detection/TrackedObjectArray (tracking only)
  ~depth_points    sensor_msgs/PointCloud2 (registered depth, decimated, color optical frame; for costmaps)
  <aligned_depth_topic>/image_raw + /camera_info  (~publish_aligned_depth): the GPU-registered depth in the
                   driver's aligned_depth_to_color format (16UC1 mm, color frame and stamp) for other nodes
"""
import copy
import json
import os
import sys
import threading
import time
from collections import defaultdict, deque

import cv2
import numpy as np
import rospy
import tf2_ros
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import String
from vision_msgs.msg import (BoundingBox2D, BoundingBox3D, Detection2D, Detection2DArray,
                             Detection3D, Detection3DArray, ObjectHypothesisWithPose)
from rs_object_detection.msg import TrackedObject, TrackedObjectArray
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray


class Heartbeat:
    """Touches ~heartbeat_file at most once a second while detections are published: a container health check
    reads the file's age, which is far cheaper than spawning a ROS node per probe. Empty path (default): off."""

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


def class_color(cls_id):
    """Deterministic BGR color per class (or track) id."""
    rng = np.random.RandomState((cls_id * 7 + 3) % 2 ** 32)
    return tuple(int(c) for c in rng.randint(60, 255, size=3))


class TrackState:
    """Per-track history kept by the node: first-seen time, 2D trail and 3D velocity."""

    def __init__(self, stamp, history):
        self.first_seen = stamp
        self.last_seen = stamp
        self.trail = deque(maxlen=history)  # recent pixel centers, for drawing
        self.samples = deque()  # (stamp, xyz) of recent 3D positions
        self.votes = defaultdict(float)  # class id -> summed score, so the label doesn't flicker
        self.vel = np.zeros(3)
        self.has_vel = False

    def vote(self, cls_id, score):
        """Add this frame's class and return the track's score-weighted majority class."""
        self.votes[cls_id] += score
        return max(self.votes, key=self.votes.get)

    def reset_motion(self):
        """Forget trail and positions, e.g. after re-identification across a gap."""
        self.trail.clear()
        self.samples.clear()
        self.vel = np.zeros(3)
        self.has_vel = False

    def update(self, stamp, uv, pos, window, min_samples, outlier_dist):
        """Velocity = least-squares slope of the positions in the last `window` seconds.

        A robust Theil-Sen line (median of pairwise slopes) is fitted first, and positions farther
        than `outlier_dist` from it are dropped: they come from the box median switching between
        foreground and background, not from real motion.
        """
        self.last_seen = stamp
        self.trail.append((int(uv[0]), int(uv[1])))
        if pos is not None:
            self.samples.append((stamp, pos))
        while self.samples and stamp - self.samples[0][0] > window:
            self.samples.popleft()
        self.has_vel = False
        if len(self.samples) >= min_samples:
            t = np.array([s[0] for s in self.samples]) - stamp
            p = np.array([s[1] for s in self.samples])
            i, j = np.triu_indices(len(t), 1)
            dt = t[j] - t[i]
            ok = dt > 1e-6
            slope = np.median((p[j] - p[i])[ok] / dt[ok, None], axis=0)
            offset = np.median(p - t[:, None] * slope, axis=0)
            keep = np.linalg.norm(p - (offset + t[:, None] * slope), axis=1) < outlier_dist
            t, p = t[keep], p[keep]
            if len(t) >= min_samples and t.max() - t.min() > 0.5 * window:
                tc = t - t.mean()
                self.vel = (tc[:, None] * (p - p.mean(axis=0))).sum(0) / (tc ** 2).sum()
                self.has_vel = True
        if not self.has_vel:
            self.vel = np.zeros(3)


def register_gated_bytetrack():
    """Register tracker_type "bytetrack_gated" with ultralytics: ByteTrack that never matches a track
    of a `gate_classes` class (e.g. person) to a detection of another class, or the reverse.

    Plain ByteTrack ignores class, so a person's track can jump onto an overlapping bed box and back.
    Classes outside the gate still match each other, so an object the detector labels remote in one
    frame and laptop in the next keeps its track.
    """
    from ultralytics.trackers import track as ul_track
    from ultralytics.trackers.basetrack import TrackState as BTState
    from ultralytics.trackers.byte_tracker import BYTETracker
    from ultralytics.trackers.utils import matching

    class GatedBYTETracker(BYTETracker):
        def update(self, results, img=None, feats=None, **kwargs):
            # Keep this frame's raw detections: the node publishes them on ~detections so a new object is
            # reported at once, not only after its track is confirmed on the next frame.
            self.raw = (results.xyxy.copy(), results.conf.copy(), results.cls.copy())
            return super().update(results, img, feats, **kwargs)

        def _gate(self, dists, tracks, dets):
            gate = list(getattr(self.args, "gate_classes", None) or [])
            if dists.size == 0 or not gate:
                return dists
            tc = np.array([int(t.cls) for t in tracks])
            dc = np.array([int(d.cls) for d in dets])
            cross = (tc[:, None] != dc[None, :]) & (np.isin(tc, gate)[:, None] | np.isin(dc, gate)[None, :])
            dists[cross] = 1.0
            return dists

        def get_dists(self, tracks, detections):
            return self._gate(super().get_dists(tracks, detections), tracks, detections)

        def _second_association(self, strack_pool, u_track, detections_second, activated, refind, lost):
            # Same as BYTETracker's (IoU-only, threshold 0.5), with the class gate applied
            r_tracked = [strack_pool[i] for i in u_track if strack_pool[i].state == BTState.Tracked]
            if r_tracked and detections_second:
                dists = self._gate(matching.iou_distance(r_tracked, detections_second), r_tracked, detections_second)
                matches, u_track, _ = matching.linear_assignment(dists, thresh=0.5)
                self._apply_matches(matches, r_tracked, detections_second, activated, refind)
            else:
                u_track = list(range(len(r_tracked)))
            for it in u_track:
                r_tracked[it].mark_lost()
                lost.append(r_tracked[it])

    ul_track.TRACKER_MAP["bytetrack_gated"] = GatedBYTETracker


class ReIDEncoder:
    """Appearance embeddings from a TensorRT ReID engine (input "images" Bx3xSxS RGB in [0, 1],
    output "embeddings" BxD), e.g. ultralytics yolo26n-reid trained on MSMT17 person ReID."""

    def __init__(self, path, device, max_batch=8):
        import tensorrt as trt
        self.dev = torch.device("cuda:%d" % device if isinstance(device, int) else device)
        with open(path, "rb") as f:
            self.engine = trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.inp = self.engine.get_binding_index("images")
        self.out = self.engine.get_binding_index("embeddings")
        self.size = self.engine.get_binding_shape(self.inp)[2]
        self.size = self.size if self.size > 0 else 448
        self.dim = self.engine.get_binding_shape(self.out)[1]
        self.max_batch = max_batch

    @torch.no_grad()
    def __call__(self, img_bgr, boxes):
        """L2-normalised embeddings (N x D numpy) for the given xyxy pixel boxes."""
        frame = torch.from_numpy(img_bgr).to(self.dev)  # uint8 HxWx3; only the crops are converted
        h, w = img_bgr.shape[:2]
        crops = []
        for x1, y1, x2, y2 in boxes:
            x1, y1 = min(max(int(x1), 0), w - 1), min(max(int(y1), 0), h - 1)
            x2, y2 = max(min(int(np.ceil(x2)), w), x1 + 1), max(min(int(np.ceil(y2)), h), y1 + 1)
            c = frame[y1:y2, x1:x2, [2, 1, 0]].permute(2, 0, 1)[None].float().div_(255.0)
            crops.append(torch.nn.functional.interpolate(c, size=(self.size, self.size), mode="bilinear",
                                                         align_corners=False))
        feats = []
        for s in range(0, len(crops), self.max_batch):
            x = torch.cat(crops[s:s + self.max_batch]).contiguous()
            y = torch.empty((x.shape[0], self.dim), device=self.dev, dtype=torch.float32)
            if hasattr(self.ctx, "set_input_shape"):  # TensorRT >= 8.5
                self.ctx.set_input_shape("images", tuple(x.shape))
            else:
                self.ctx.set_binding_shape(self.inp, tuple(x.shape))
            bindings = [0, 0]
            bindings[self.inp], bindings[self.out] = int(x.data_ptr()), int(y.data_ptr())
            self.ctx.execute_async_v2(bindings, torch.cuda.current_stream().cuda_stream)
            feats.append(y)
        return torch.nn.functional.normalize(torch.cat(feats), dim=1).cpu().numpy()


class Identity:
    """A persistent object id, with a gallery of recent appearance embeddings."""

    def __init__(self, pid, cls_id, stamp, gallery_size):
        self.pid = pid
        self.cls_id = cls_id
        self.last_seen = stamp
        self.last_feat = -1e9  # time the gallery was last updated
        self.gallery = deque(maxlen=gallery_size)


class Reidentifier:
    """Maps ByteTrack ids to persistent ids that survive the object leaving the view.

    ByteTrack keeps a lost track for ~1.5 s and re-associates it only near its motion-predicted
    position, so turning the camera away and back gives the same person a new track. Here a new
    ByteTrack track of a re-identified class is first *pending* (reported with a key >= PENDING,
    shown as "#?"): its embeddings, from crops not cut off at the image edge, are compared with the
    galleries of identities of the same class that are not visible right now. If the best match
    reaches `threshold`, the old id is reused; after `pending_tries` crops (or `pending_time`)
    without a match, it gets a new id.
    """

    PENDING = 1 << 30  # keys at or above this are pending tracks (PENDING + ByteTrack id)

    def __init__(self, encoder, classes, threshold, memory, gallery_size, update_period, pending_tries,
                 pending_time, border):
        self.encoder = encoder
        self.classes = set(classes) if classes else None  # None = all classes
        self.threshold = threshold
        self.memory = memory
        self.gallery_size = gallery_size
        self.update_period = update_period
        self.pending_tries = pending_tries
        self.pending_time = pending_time
        self.border = border
        self.bt2pid = {}  # ByteTrack id -> persistent id
        self.pending = {}  # ByteTrack id -> {"cls", "first", "seen", "tries", "feats"}
        self.ids = {}  # persistent id -> Identity
        self.next_pid = 1

    def _new_identity(self, cls_id, stamp, feats=()):
        ident = Identity(self.next_pid, cls_id, stamp, self.gallery_size)
        ident.gallery.extend(feats)
        if feats:
            ident.last_feat = stamp
        self.ids[ident.pid] = ident
        self.next_pid += 1
        return ident.pid

    def _good_crop(self, i, boxes, scores, shape):
        """Crop usable for appearance: confident, not occluded, and not cut off at the left/right edge.

        Only the side edges are checked: an object entering the view as the camera pans is cut off
        there and looks different, while a person close to the camera is routinely cut off at the
        top or bottom and still has to be matched.
        """
        x1, y1, x2, y2 = boxes[i]
        w = shape[1]
        if x1 < self.border or x2 > w - self.border or scores[i] < 0.5:
            return False
        for j in range(len(boxes)):
            if j != i:
                ix = max(0.0, min(x2, boxes[j][2]) - max(x1, boxes[j][0]))
                iy = max(0.0, min(y2, boxes[j][3]) - max(y1, boxes[j][1]))
                if ix * iy > 0.2 * (x2 - x1) * (y2 - y1):
                    return False
        return True

    def _best_match(self, cls_id, feat, exclude):
        best, best_sim = None, self.threshold
        for cand in self.ids.values():
            if cand.pid in exclude or cand.cls_id != cls_id or not cand.gallery:
                continue
            sim = float(np.max(np.stack(cand.gallery) @ feat))
            if sim >= best_sim:
                best, best_sim = cand, sim
        return best, best_sim

    def assign(self, stamp, img, bt_ids, boxes, clss, scores):
        """Keys for this frame's tracks (persistent id, or >= PENDING while undecided), plus
        {old key: persistent id} for tracks that were decided this frame."""
        keys, decided = [], {}
        for bt, c in zip(bt_ids, clss):
            if bt not in self.bt2pid and bt not in self.pending:
                if self.classes is None or int(c) in self.classes:
                    self.pending[bt] = {"cls": int(c), "first": stamp, "tries": 0, "feats": []}
                else:
                    self.bt2pid[bt] = self._new_identity(int(c), stamp)
            if bt in self.pending:
                self.pending[bt]["seen"] = stamp
                keys.append(self.PENDING + bt)
            else:
                keys.append(self.bt2pid[bt])
                self.ids[keys[-1]].last_seen = stamp
        visible = set(k for k in keys if k < self.PENDING)

        # Embeddings: every usable crop of a pending track, and periodic gallery updates otherwise
        todo = []
        for i, (bt, k) in enumerate(zip(bt_ids, keys)):
            if k >= self.PENDING:
                need = True
            else:
                ident = self.ids[k]
                need = ((self.classes is None or ident.cls_id in self.classes)
                        and stamp - ident.last_feat >= self.update_period)
            if need and self._good_crop(i, boxes, scores, img.shape[:2]):
                todo.append(i)
        feats = self.encoder(img, [boxes[i] for i in todo]) if todo else []

        for i, f in zip(todo, feats):
            bt, k = bt_ids[i], keys[i]
            if k < self.PENDING:
                self.ids[k].gallery.append(f)
                self.ids[k].last_feat = stamp
                continue
            pend = self.pending[bt]
            pend["tries"] += 1
            pend["feats"].append(f)
            best, sim = self._best_match(pend["cls"], f, visible)
            if best is not None:
                rospy.loginfo("Re-identified a new track as #%d (similarity %.2f, gone %.1f s)",
                              best.pid, sim, stamp - best.last_seen)
                best.gallery.extend(pend["feats"])
                best.last_seen = best.last_feat = stamp
                self.bt2pid[bt] = keys[i] = decided[k] = best.pid
                visible.add(best.pid)
                del self.pending[bt]

        for bt in list(self.pending):
            pend = self.pending[bt]
            if pend["tries"] >= self.pending_tries or stamp - pend["first"] > self.pending_time:
                pid = self._new_identity(pend["cls"], stamp, pend["feats"])  # no match: a new identity
                self.bt2pid[bt] = decided[self.PENDING + bt] = pid
                keys = [pid if k == self.PENDING + bt else k for k in keys]
                del self.pending[bt]
            elif stamp - pend["seen"] > self.pending_time:
                del self.pending[bt]  # ByteTrack dropped it before it was decided
        # Forget identities not seen for `memory` seconds, and ByteTrack ids that point to them
        for pid in [p for p, ident in self.ids.items() if stamp - ident.last_seen > self.memory]:
            del self.ids[pid]
        for bt in [b for b, p in self.bt2pid.items() if p not in self.ids]:
            del self.bt2pid[bt]
        return keys, decided


class DepthRegistrar:
    """Reprojects a raw depth image into the color camera's pixel grid on the GPU."""

    def __init__(self, device):
        self.device = torch.device("cuda:%d" % device if isinstance(device, int) else device)
        self.rays = None  # (x/z, y/z) for every depth pixel, cached per depth intrinsics

    def register(self, depth_m, Kd, Kc, T_c_d, color_shape):
        """depth_m: HxW meters in the depth frame -> color_shape depth map in meters (0 = no data)."""
        dev = self.device
        h, w = depth_m.shape
        if self.rays is None or self.rays[0].shape != (h, w):
            v, u = torch.meshgrid(torch.arange(h, device=dev, dtype=torch.float32),
                                  torch.arange(w, device=dev, dtype=torch.float32), indexing="ij")
            self.rays = ((u - Kd[0, 2]) / Kd[0, 0], (v - Kd[1, 2]) / Kd[1, 1])
        z = torch.from_numpy(depth_m).to(dev).view(-1)
        pts = torch.stack([self.rays[0].view(-1) * z, self.rays[1].view(-1) * z, z])
        R = torch.as_tensor(T_c_d[:3, :3], dtype=torch.float32, device=dev)
        t = torch.as_tensor(T_c_d[:3, 3:], dtype=torch.float32, device=dev)
        pc = R @ pts + t
        zc = pc[2]
        hc, wc = color_shape
        zsafe = torch.where(z > 0, zc, torch.ones_like(zc))
        uc = torch.round(pc[0] / zsafe * Kc[0, 0] + Kc[0, 2]).long()
        vc = torch.round(pc[1] / zsafe * Kc[1, 1] + Kc[1, 2]).long()
        ok = (z > 0) & (zc > 0) & (uc >= 0) & (uc < wc) & (vc >= 0) & (vc < hc)
        # Invalid points go to a dummy slot at the end, avoiding boolean-index compaction (a GPU sync).
        # Nearest surface wins when several depth pixels land on the same color pixel.
        dummy = hc * wc
        out = torch.full((dummy + 1,), float("inf"), device=dev)
        out.scatter_reduce_(0, torch.where(ok, vc * wc + uc, torch.full_like(uc, dummy)), zc, reduce="amin")
        out = out[:dummy].view(1, 1, hc, wc)
        # When color has a higher resolution than depth, each depth pixel covers several color
        # pixels; fill the resulting pinholes with the nearest neighbouring surface.
        k = 2 * int(np.ceil(max(Kc[0, 0] / Kd[0, 0], Kc[1, 1] / Kd[1, 1]) - 1e-3)) - 1
        if k > 1:
            filled = -torch.nn.functional.max_pool2d(-out, k, stride=1, padding=k // 2)
            out = torch.where(torch.isinf(out), filled, out)
        out = torch.where(torch.isinf(out), torch.zeros_like(out), out)
        return out.view(hc, wc).cpu().numpy()


class RealSenseDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.heartbeat = Heartbeat(rospy.get_param("~heartbeat_file", ""))

        # Model / inference params
        model_path = os.path.expanduser(rospy.get_param("~model", "yolo11n.pt"))
        self.conf = rospy.get_param("~conf", 0.4)
        self.iou = rospy.get_param("~iou", 0.5)
        self.imgsz = rospy.get_param("~imgsz", 640)
        # FP16 for .pt models; a TensorRT .engine keeps the precision it was exported with
        self.quantize = 16 if rospy.get_param("~half", True) else None
        self.device = rospy.get_param("~device", 0)
        classes = rospy.get_param("~classes", [])
        self.classes = list(classes) if classes else None

        # Depth params
        self.depth_scale = rospy.get_param("~depth_scale", 0.001)  # 16UC1 units -> meters
        self.min_depth = rospy.get_param("~min_depth", 0.2)
        self.max_depth = rospy.get_param("~max_depth", 8.0)
        self.roi_fraction = rospy.get_param("~roi_fraction", 0.5)  # central part of box used for depth
        self.min_valid_pixels = rospy.get_param("~min_valid_pixels", 10)

        self.publish_annotated = rospy.get_param("~publish_annotated", True)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.3)
        self.log_every = rospy.get_param("~log_every", 60)

        # Inference backend. "deepstream" runs detection + tracking in a DeepStream pipeline (NvSORT by default);
        # everything after that (depth, 3D, re-ID, topics) is the same code for both backends.
        self.backend = rospy.get_param("~backend", "ultralytics")
        # Tracking: ultralytics tracker yaml (e.g. config/tracker.yaml), or with the deepstream backend an
        # nvtracker config (e.g. config/deepstream/tracker_NvSORT.yml); empty disables tracking
        self.tracker = os.path.expanduser(rospy.get_param("~tracker", ""))
        self.track_history = rospy.get_param("~track_history", 30)  # trail length in frames
        self.velocity_window = rospy.get_param("~velocity_window", 0.5)  # s of positions fitted for velocity
        self.velocity_min_samples = rospy.get_param("~velocity_min_samples", 5)
        self.velocity_outlier = rospy.get_param("~velocity_outlier", 0.25)  # m from the window median
        self.track_timeout = rospy.get_param("~track_timeout", 3.0)  # s before a vanished track's state is freed
        self.tracks = {}
        # Frames for ~tracks: positions are published in output_frame, velocity is fitted in velocity_frame
        # (e.g. odom, so the robot's own motion is removed). Empty = the color camera frame.
        self.output_frame = rospy.get_param("~output_frame", "")
        self.velocity_frame = rospy.get_param("~velocity_frame", "")
        self.static_tf = {}  # cached camera -> output_frame transform (the camera mount is fixed)
        # Re-identification across gaps (needs tracking); empty reid_model disables it
        reid_model = os.path.expanduser(rospy.get_param("~reid_model", ""))
        self.reid = None
        if self.tracker and reid_model:
            self.reid = Reidentifier(
                ReIDEncoder(reid_model, self.device),
                classes=list(rospy.get_param("~reid_classes", [0])),
                threshold=rospy.get_param("~reid_threshold", 0.6),
                memory=rospy.get_param("~reid_memory", 300.0),
                gallery_size=rospy.get_param("~reid_gallery", 20),
                update_period=rospy.get_param("~reid_update_period", 0.5),
                pending_tries=rospy.get_param("~reid_pending_tries", 5),
                pending_time=rospy.get_param("~reid_pending_time", 2.0),
                border=rospy.get_param("~reid_border", 8))
        self.predict_conf = self.conf
        if self.tracker and self.backend == "ultralytics":
            import yaml
            register_gated_bytetrack()
            with open(self.tracker) as f:
                tcfg = yaml.safe_load(f)
            # ByteTrack needs the weak detections too; its own thresholds decide what becomes a track
            self.predict_conf = min(self.conf, float(tcfg.get("track_low_thresh", 0.1)))

        color_topic = rospy.get_param("~color_topic", "/camera/camera/color/image_raw")
        info_topic = rospy.get_param("~info_topic", "/camera/camera/color/camera_info")
        self.register_depth = rospy.get_param("~register_depth", True)
        if self.register_depth:
            depth_topic = rospy.get_param("~depth_topic", "/camera/camera/depth/image_rect_raw")
            depth_info_topic = rospy.get_param("~depth_info_topic", "/camera/camera/depth/camera_info")
        else:
            depth_topic = rospy.get_param("~depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")

        if self.backend == "deepstream":
            self.ds = None  # pipeline is built on the first frame, when the image size is known
            self.ds_infer_config = os.path.expanduser(rospy.get_param("~ds_infer_config"))
            self.names = self.read_labels(self.ds_infer_config)
            rospy.loginfo("DeepStream backend: %s", self.ds_infer_config)
        else:
            rospy.loginfo("Loading model %s", model_path)
            from ultralytics import YOLO  # imported here so rospy logging is up first
            self.model = YOLO(model_path, task="detect")
            self.names = self.model.names
            self._warmup()

        self.K = None
        self.Kd = None  # depth intrinsics (register_depth only)
        self.T_c_d = None  # depth optical frame -> color optical frame
        self.registrar = DepthRegistrar(self.device) if self.register_depth else None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lock = threading.Lock()
        self.latest = None  # newest color frame; older unprocessed frames are dropped
        self.event = threading.Event()

        self.pub_det3d = rospy.Publisher("~detections", Detection3DArray, queue_size=5)
        self.pub_det2d = rospy.Publisher("~detections_2d", Detection2DArray, queue_size=5)
        self.pub_img = rospy.Publisher("~image_annotated", Image, queue_size=2)
        self.pub_jpg = rospy.Publisher("~image_annotated/compressed", CompressedImage, queue_size=2)
        # Safety monitor state drawn as a banner on the annotated image (empty topic disables it)
        self.safety_state, self.safety_stamp = None, 0.0
        safety_topic = rospy.get_param("~safety_state_topic", "/scout_safety/state")
        if safety_topic:
            rospy.Subscriber(safety_topic, String, self._safety_cb, queue_size=1)
        # Decimated point cloud of the registered depth for obstacle avoidance (move_base costmaps)
        self.pub_cloud = rospy.Publisher("~depth_points", PointCloud2, queue_size=1)
        self.cloud_step = rospy.get_param("~cloud_step", 4)            # every Nth pixel in u and v
        # m; D455 depth noise grows ~quadratically. Under ~0.5 m it reads phantom patches (0.33 m with nothing there,
        # lab2 2026-10-24), which sat on the costmap footprint and blocked every motion
        self.cloud_range = rospy.get_param("~cloud_range", [0.5, 3.5])
        self.cloud_period = 1.0 / rospy.get_param("~cloud_rate", 10.0)
        self.cloud_last = 0.0
        # Republish the registered depth like the driver's align_depth output (camera must run align_depth:=false)
        self.pub_adepth = self.pub_adepth_info = None
        if rospy.get_param("~publish_aligned_depth", False) and self.register_depth:
            ns = rospy.get_param("~aligned_depth_topic", "/camera/camera/aligned_depth_to_color")
            self.pub_adepth = rospy.Publisher(ns + "/image_raw", Image, queue_size=2)
            self.pub_adepth_info = rospy.Publisher(ns + "/camera_info", CameraInfo, queue_size=2)
        self.jpeg_quality = rospy.get_param("~jpeg_quality", 75)
        self.pub_markers = rospy.Publisher("~markers", MarkerArray, queue_size=5)
        self.pub_tracks = rospy.Publisher("~tracks", TrackedObjectArray, queue_size=5) if self.tracker else None

        # Detection runs on every color frame; the newest depth frame is attached if it is recent
        # enough (depth may run slower than color, so strict sync would cap the FPS).
        self.depth_max_age = rospy.get_param("~depth_max_age", 0.25)
        self.latest_depth = None
        self.n_rx = 0  # color frames received, for the FPS log
        rospy.Subscriber(info_topic, CameraInfo, self._info_cb, queue_size=1)
        rospy.Subscriber(color_topic, Image, self._color_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(depth_topic, Image, self._depth_cb, queue_size=1, buff_size=2 ** 24)
        if self.register_depth:
            rospy.Subscriber(depth_info_topic, CameraInfo, self._depth_info_cb, queue_size=1)

        if self.tracker:
            rospy.loginfo("Tracking with %s%s", self.tracker, ", re-identification with %s" % reid_model if self.reid else "")
        rospy.loginfo("Waiting for %s, %s and %s", color_topic, depth_topic, info_topic)

    def _warmup(self):
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        for _ in range(2):
            self.model.predict(dummy, imgsz=self.imgsz, quantize=self.quantize, device=self.device, verbose=False)
        rospy.loginfo("Model warmed up (%d classes)", len(self.names))

    def _safety_cb(self, msg):
        try:
            self.safety_state, self.safety_stamp = json.loads(msg.data), time.time()
        except ValueError:
            pass

    SAFETY_COLORS = {"OK": (60, 170, 60), "SLOW": (0, 190, 255), "STOP": (0, 0, 230), "FAULT": (200, 0, 160)}

    def draw_safety(self, img):
        """Banner across the top: safety state, its reason and the speed limit."""
        if self.safety_state is None:
            return
        st = self.safety_state
        if time.time() - self.safety_stamp > 1.0:
            state, text = "FAULT", "SAFETY: no state from the safety monitor"
        else:
            state = st.get("state", "?")
            label = {"STOP": "SAFETY STOP", "SLOW": "SAFETY SLOW", "OK": "SAFETY OK", "FAULT": "SAFETY FAULT"}
            text = "%s - %s" % (label.get(state, state), st.get("reason", ""))
            if state in ("OK", "SLOW"):
                text += "  (max %.2f m/s)" % st.get("limit", 0.0)
        h, w = img.shape[:2]
        bar = max(28, h // 16)
        strip = img[:bar]  # blend only the banner strip
        color = np.array(self.SAFETY_COLORS.get(state, (128, 128, 128)), np.float32)
        strip[:] = (strip.astype(np.float32) * 0.25 + color * 0.75).astype(np.uint8)
        scale = bar / 40.0
        cv2.putText(img, text, (8, int(bar * 0.72)), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255),
                    max(1, int(round(scale * 2))), cv2.LINE_AA)
        if state == "STOP":  # red frame around the whole image
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), self.SAFETY_COLORS["STOP"], max(4, h // 80))

    def _info_cb(self, msg):
        self.color_info = msg
        self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.color_frame = msg.header.frame_id

    def _depth_info_cb(self, msg):
        self.Kd = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.depth_frame = msg.header.frame_id

    def lookup_extrinsics(self):
        """Static depth->color transform from TF; None until it is available."""
        if self.T_c_d is None and self.Kd is not None and self.K is not None:
            try:
                tr = self.tf_buffer.lookup_transform(self.color_frame, self.depth_frame, rospy.Time(0)).transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(5, "Waiting for TF %s -> %s", self.depth_frame, self.color_frame)
                return None
            T = quaternion_matrix([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
            T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            self.T_c_d = T
            rospy.loginfo("Registering depth into color frame (baseline %.3f m)", np.linalg.norm(T[:3, 3]))
        return self.T_c_d

    def _depth_cb(self, msg):
        self.latest_depth = msg

    def _color_cb(self, msg):
        self.n_rx += 1
        with self.lock:
            self.latest = msg
        self.event.set()

    # ------------------------------------------------------------------ processing
    def spin(self):
        n, t_acc, t_last = 0, 0.0, time.time()
        while not rospy.is_shutdown():
            if not self.event.wait(timeout=0.5):
                continue
            self.event.clear()
            with self.lock:
                color_msg, self.latest = self.latest, None
            if color_msg is None or self.K is None:
                continue
            depth_msg = self.latest_depth
            if depth_msg is not None and abs((color_msg.header.stamp - depth_msg.header.stamp).to_sec()) > self.depth_max_age:
                rospy.logwarn_throttle(5, "Depth is stale; publishing 2D detections only")
                depth_msg = None
            if depth_msg is not None and self.register_depth and self.lookup_extrinsics() is None:
                depth_msg = None

            t0 = time.time()
            self.process(color_msg, depth_msg)
            t_acc += time.time() - t0
            n += 1
            if n % self.log_every == 0:
                now = time.time()
                rospy.loginfo("%.1f FPS (camera %.1f Hz), %.1f ms/frame", self.log_every / (now - t_last),
                              self.n_rx / (now - t_last), 1000.0 * t_acc / self.log_every)
                self.n_rx = 0
                t_acc, t_last = 0.0, now

    def depth_to_meters(self, depth_msg):
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * self.depth_scale
        return depth.astype(np.float32)

    def estimate_depth(self, depth_m, x1, y1, x2, y2):
        """Median and spread of valid depth in the central part of the box, or None."""
        w, h = x2 - x1, y2 - y1
        m = (1.0 - self.roi_fraction) / 2.0
        rx1, rx2 = int(x1 + m * w), int(np.ceil(x2 - m * w))
        ry1, ry2 = int(y1 + m * h), int(np.ceil(y2 - m * h))
        roi = depth_m[max(ry1, 0):max(ry2, ry1 + 1), max(rx1, 0):max(rx2, rx1 + 1)]
        # Large boxes: sample on a regular grid (~4000 pixels) instead of sorting every pixel
        step = max(1, int(np.sqrt(roi.size / 4000.0)))
        roi = roi[::step, ::step]
        valid = roi[(roi > self.min_depth) & (roi < self.max_depth)]
        if valid.size < self.min_valid_pixels:
            return None
        p10, z, p90 = np.percentile(valid, [10, 50, 90])
        return float(z), float(max(p90 - p10, 0.05))

    def process(self, color_msg, depth_msg):
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_m = self.depth_to_meters(depth_msg) if depth_msg is not None else None
        if depth_m is not None and self.register_depth:
            depth_m = self.registrar.register(depth_m, self.Kd, self.K, self.T_c_d, color.shape[:2])
            self.publish_aligned_depth(depth_m, color_msg.header)
            self.publish_cloud(depth_m, color_msg.header)
        elif depth_m is not None and depth_m.shape[:2] != color.shape[:2]:
            rospy.logwarn_throttle(5, "Depth %s and color %s sizes differ; is align_depth:=true?",
                                   depth_m.shape[:2], color.shape[:2])
            depth_m = None

        dets, xyxy, confs, clss, ids = self.infer(color)
        if ids is not None and self.reid is not None:
            keys, decided = self.reid.assign(color_msg.header.stamp.to_sec(), color, list(ids), xyxy, clss, confs)
            ids = np.array(keys)
            for pending_key, old in decided.items():
                tr = self.tracks.pop(pending_key, None)
                if old in self.tracks:
                    self.tracks[old].reset_motion()  # positions across the gap are not one motion
                    if tr is not None:
                        for c, v in tr.votes.items():
                            self.tracks[old].votes[c] += v
                elif tr is not None:
                    tr.reset_motion()
                    self.tracks[old] = tr

        header = color_msg.header
        stamp = header.stamp.to_sec()
        arr3d = Detection3DArray(header=header)
        arr2d = Detection2DArray(header=header)
        markers = MarkerArray()
        markers.markers.append(Marker(header=header, action=Marker.DELETEALL))
        # Draw only if someone is watching the raw or JPEG image
        watched = self.pub_img.get_num_connections() > 0 or self.pub_jpg.get_num_connections() > 0
        annotated = color.copy() if self.publish_annotated and watched else None

        # Detections: every confident box this frame, tracked or not (what a safety consumer should use)
        for i, (box, score, cls_id) in enumerate(dets):
            label = self.label(cls_id)
            det2d = self.make_det2d(header, box, cls_id, score)
            arr2d.detections.append(det2d)
            d3 = self.box_to_3d(depth_m, box)
            if d3 is not None:
                arr3d.detections.append(self.make_det3d(header, d3, cls_id, score))
            if not self.tracker:  # with tracking, markers and drawing follow the tracks below
                text = "%s %.2f" % (label, score) + (" %.2fm" % d3[0][2] if d3 is not None else "")
                if d3 is not None:
                    markers.markers.extend(self.make_markers(header, i, label, d3[0][2],
                                                             arr3d.detections[-1].bbox, cls_id))
                if annotated is not None:
                    self.draw(annotated, box, text, class_color(cls_id))

        if self.tracker:
            tracks_msg = self.make_tracks(header, stamp, depth_m, xyxy, confs, clss, ids, markers, annotated)
            self.pub_tracks.publish(tracks_msg)
            for tid in [t for t, tr in self.tracks.items() if stamp - tr.last_seen > self.track_timeout]:
                del self.tracks[tid]

        self.pub_det2d.publish(arr2d)
        self.pub_det3d.publish(arr3d)
        self.heartbeat.beat()
        self.pub_markers.publish(markers)
        if annotated is not None:
            self.draw_safety(annotated)
        if annotated is not None and self.pub_img.get_num_connections() > 0:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = header
            self.pub_img.publish(out)
        if annotated is not None and self.pub_jpg.get_num_connections() > 0:
            ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                self.pub_jpg.publish(CompressedImage(header=header, format="jpeg", data=buf.tobytes()))

    def publish_cloud(self, depth_m, header):
        now = time.time()
        if self.pub_cloud.get_num_connections() == 0 or now - self.cloud_last < self.cloud_period:
            return
        self.cloud_last = now
        s = self.cloud_step
        z = depth_m[::s, ::s]
        v, u = np.mgrid[0:depth_m.shape[0]:s, 0:depth_m.shape[1]:s]
        ok = (z > self.cloud_range[0]) & (z < self.cloud_range[1])
        z, u, v = z[ok], u[ok], v[ok]
        xyz = np.stack([(u - self.K[0, 2]) * z / self.K[0, 0], (v - self.K[1, 2]) * z / self.K[1, 1], z], 1)
        msg = PointCloud2(header=header, height=1, width=len(xyz), is_bigendian=False, point_step=12,
                          row_step=12 * len(xyz), is_dense=True,
                          fields=[PointField(n, 4 * i, PointField.FLOAT32, 1) for i, n in enumerate("xyz")])
        msg.data = xyz.astype(np.float32).tobytes()
        self.pub_cloud.publish(msg)

    def publish_aligned_depth(self, depth_m, header):
        # Each only for its own subscribers (viewers often subscribe to camera_info alone)
        if self.pub_adepth is None:
            return
        if self.pub_adepth.get_num_connections() > 0:
            mm = np.clip(np.round(depth_m / self.depth_scale), 0, 65535).astype(np.uint16)
            msg = self.bridge.cv2_to_imgmsg(mm, encoding="16UC1")
            msg.header = header  # color stamp and frame: registered depth lives in the color image
            self.pub_adepth.publish(msg)
        if self.pub_adepth_info.get_num_connections() > 0:
            info = copy.copy(self.color_info)
            info.header = header
            self.pub_adepth_info.publish(info)

    def infer(self, color):
        """Detections and tracks for one BGR frame, from either backend.

        Returns dets [(box, score, class)] with score >= ~conf (every confident box, tracked or not), and the
        tracks as arrays xyxy, confs, clss, ids (ids is None without tracking or when nothing is tracked).
        """
        if self.backend == "deepstream":
            return self.infer_deepstream(color)
        kwargs = dict(conf=self.predict_conf, iou=self.iou, imgsz=self.imgsz, quantize=self.quantize,
                      device=self.device, classes=self.classes, verbose=False)
        if self.tracker:
            result = self.model.track(color, persist=True, tracker=self.tracker, **kwargs)[0]
            xyxy, confs, clss = self.model.predictor.trackers[0].raw
            keep = confs >= self.conf  # the tracker ran at its low threshold; report only confident boxes
            dets = list(zip(xyxy[keep], confs[keep], clss[keep].astype(int)))
        else:
            result = self.model.predict(color, **kwargs)[0]
        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy() if len(boxes) else np.empty((0, 4))
        confs = boxes.conf.cpu().numpy() if len(boxes) else np.empty(0)
        clss = boxes.cls.cpu().numpy().astype(int) if len(boxes) else np.empty(0, dtype=int)
        if not self.tracker:
            dets = list(zip(xyxy, confs, clss))
        # Only confirmed tracks are returned in track mode; ids is None when there are none
        ids = boxes.id.cpu().numpy().astype(int) if self.tracker and boxes.id is not None else None
        if self.tracker and ids is None:
            xyxy, confs, clss = xyxy[:0], confs[:0], clss[:0]
        return dets, xyxy, confs, clss, ids

    def infer_deepstream(self, color):
        h, w = color.shape[:2]
        if self.ds is None or self.ds_size != (w, h):
            sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
            from ds_pipeline import DSPipeline
            if self.ds is not None:
                self.ds.finish()
            rospy.loginfo("Starting DeepStream pipeline for %dx%d (first start can take a few seconds)", w, h)
            self.ds = DSPipeline(self.ds_infer_config, self.tracker, w, h)
            self.ds.start()
            self.ds_size = (w, h)
        out = self.ds.process(cv2.cvtColor(color, cv2.COLOR_BGR2RGBA), timeout=5.0)
        if out is None:
            rospy.logwarn_throttle(5, "DeepStream returned no result for a frame")
            out = ([], [])
        raw, tracked = out
        keep = (lambda c: True) if self.classes is None else (lambda c: c in self.classes)
        dets = [(np.array(d[:4]), d[5], d[4]) for d in raw if d[5] >= self.conf and keep(d[4])]
        if not self.tracker:
            return dets, np.array([d[0] for d in dets]).reshape(-1, 4), np.array([d[1] for d in dets]), \
                np.array([d[2] for d in dets], dtype=int), None
        tracked = [t for t in tracked if keep(t[4])]
        if not tracked:
            return dets, np.empty((0, 4)), np.empty(0), np.empty(0, dtype=int), None
        xyxy = np.array([t[:4] for t in tracked])
        # a track carried through a frame without a detection has confidence -0.1: report the tracker's
        confs = np.array([t[5] if t[5] >= 0 else t[7] for t in tracked])
        clss = np.array([t[4] for t in tracked], dtype=int)
        ids = np.array([t[6] for t in tracked], dtype=int)
        return dets, xyxy, confs, clss, ids

    @staticmethod
    def read_labels(infer_config):
        """Class names {id: name} from the labelfile-path of an nvinfer config."""
        base = os.path.dirname(infer_config)
        with open(infer_config) as f:
            path = next(l.split("=", 1)[1].strip() for l in f if l.startswith("labelfile-path"))
        with open(os.path.join(base, path)) as f:
            return {i: n.strip() for i, n in enumerate(f) if n.strip()}

    def make_tracks(self, header, stamp, depth_m, xyxy, confs, clss, ids, markers, annotated):
        """TrackedObjectArray for this frame's tracks (in output_frame), plus their markers and drawing."""
        cam = header.frame_id
        T_out = self.lookup(self.output_frame or cam, cam, header.stamp, static=True)
        out_frame = self.output_frame if T_out is not None else cam
        if T_out is None:
            T_out = np.eye(4)
        T_vel = self.lookup(self.velocity_frame or cam, cam, header.stamp)
        if T_vel is not None:
            R_out_vel = T_out[:3, :3] @ T_vel[:3, :3].T  # rotates a velocity-frame vector into output_frame
        msg = TrackedObjectArray()
        msg.header.stamp, msg.header.frame_id = header.stamp, out_frame

        for i, (box, score, cls_id) in enumerate(zip(xyxy, confs, clss)):
            tid = int(ids[i])
            tr = self.tracks.get(tid)
            if tr is None:
                tr = self.tracks[tid] = TrackState(stamp, self.track_history)
            cls_id = tr.vote(int(cls_id), float(score))
            label = self.label(cls_id)
            # Pending re-identification: no id yet (published as id 0, drawn as "#?")
            pending = self.reid is not None and tid >= Reidentifier.PENDING
            tag = "#? " if pending else "#%d " % tid
            d3 = self.box_to_3d(depth_m, box)
            pos = d3[0] if d3 is not None else None
            # Velocity is fitted on positions in velocity_frame; without that TF this frame, skip the sample
            pos_v = T_vel[:3, :3] @ pos + T_vel[:3, 3] if pos is not None and T_vel is not None else None
            u, v = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
            tr.update(stamp, (u, v), pos_v, self.velocity_window, self.velocity_min_samples, self.velocity_outlier)

            obj = TrackedObject(id=0 if pending else tid, class_id=int(cls_id), label=label, score=float(score),
                                bbox=self.make_det2d(header, box, cls_id, score).bbox,
                                has_position=pos is not None, age=stamp - tr.first_seen)
            text = tag + "%s %.2f" % (label, score)
            if pos is not None:
                obj.position.x, obj.position.y, obj.position.z = T_out[:3, :3] @ pos + T_out[:3, 3]
                if tr.has_vel and T_vel is not None:
                    obj.velocity.x, obj.velocity.y, obj.velocity.z = R_out_vel @ tr.vel
                    # the arrow is drawn in the camera frame, like the boxes
                    markers.markers.append(self.make_velocity_marker(header, tid, pos, T_vel[:3, :3].T @ tr.vel))
                obj.size.x, obj.size.y, obj.size.z = d3[1]
                text += " %.2fm" % pos[2]
                bbox3 = self.make_det3d(header, d3, cls_id, score).bbox
                markers.markers.extend(self.make_markers(header, tid, tag + label, pos[2], bbox3, tid))
            msg.objects.append(obj)

            if annotated is not None:
                col = (200, 200, 200) if pending else class_color(tid)
                if len(tr.trail) > 1:
                    cv2.polylines(annotated, [np.array(tr.trail, np.int32)], False, col, 2)
                self.draw(annotated, box, text, col)
        return msg

    def lookup(self, target, source, stamp, static=False):
        """4x4 transform taking points from `source` to `target`, or None if TF doesn't have it.

        Uses the transform at the image stamp, or the latest one if TF has not caught up to it yet.
        """
        if target == source:
            return np.eye(4)
        if static and (target, source) in self.static_tf:
            return self.static_tf[(target, source)]
        for t in (stamp, rospy.Time(0)):
            try:
                tr = self.tf_buffer.lookup_transform(target, source, t).transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            T = quaternion_matrix([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
            T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            if static:
                self.static_tf[(target, source)] = T
            return T
        rospy.logwarn_throttle(5, "No TF %s -> %s; tracks fall back to the camera frame / no velocity", source, target)
        return None

    def label(self, cls_id):
        return self.names.get(cls_id, str(cls_id)) if isinstance(self.names, dict) else self.names[cls_id]

    def box_to_3d(self, depth_m, box):
        """(position, size) in the color optical frame for a pixel box, or None without valid depth."""
        if depth_m is None:
            return None
        x1, y1, x2, y2 = box
        est = self.estimate_depth(depth_m, x1, y1, x2, y2)
        if est is None:
            return None
        z, extent = est
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        pos = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        return pos, np.array([(x2 - x1) * z / fx, (y2 - y1) * z / fy, extent])

    @staticmethod
    def make_det2d(header, box, cls_id, score):
        x1, y1, x2, y2 = box
        det = Detection2D(header=header)
        det.bbox = BoundingBox2D()
        det.bbox.center.x, det.bbox.center.y = float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)
        det.bbox.size_x, det.bbox.size_y = float(x2 - x1), float(y2 - y1)
        det.results.append(ObjectHypothesisWithPose(id=int(cls_id), score=float(score)))
        return det

    @staticmethod
    def make_det3d(header, d3, cls_id, score):
        pos, size = d3
        hyp = ObjectHypothesisWithPose(id=int(cls_id), score=float(score))
        hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = pos
        hyp.pose.pose.orientation.w = 1.0
        det = Detection3D(header=header)
        det.results.append(hyp)
        det.bbox = BoundingBox3D()
        det.bbox.center = hyp.pose.pose
        det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = [float(x) for x in size]
        return det

    @staticmethod
    def draw(img, box, text, col):
        x1, y1, x2, y2 = box
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, col, 2)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(p1[1], th + 4)
        cv2.rectangle(img, (p1[0], ty - th - 4), (p1[0] + tw + 2, ty), col, -1)
        cv2.putText(img, text, (p1[0] + 1, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.circle(img, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 3, col, -1)

    def make_markers(self, header, idx, label, z, bbox, color_id):
        b, g, r = [c / 255.0 for c in class_color(color_id)]
        life = rospy.Duration(self.marker_lifetime)

        box = Marker(header=header, ns="boxes", id=idx, type=Marker.CUBE, action=Marker.ADD)
        box.pose = bbox.center
        box.scale = bbox.size
        box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.35
        box.lifetime = life

        txt = Marker(header=header, ns="labels", id=idx, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        txt.pose.position = Point(bbox.center.position.x,
                                  bbox.center.position.y - bbox.size.y / 2.0 - 0.08,
                                  bbox.center.position.z)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.12
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = "%s %.2fm" % (label, z)
        txt.lifetime = life
        return [box, txt]


    def make_velocity_marker(self, header, tid, pos, vel):
        """Arrow showing where the object will be in 1 s at its current velocity."""
        arrow = Marker(header=header, ns="velocity", id=tid, type=Marker.ARROW, action=Marker.ADD)
        arrow.points = [Point(*pos), Point(*(pos + vel))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.02, 0.04, 0.06  # shaft, head diameter, head length
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 0.0, 0.9
        arrow.pose.orientation.w = 1.0
        arrow.lifetime = rospy.Duration(self.marker_lifetime)
        return arrow


def main():
    rospy.init_node("rs_detector")
    node = RealSenseDetector()
    node.spin()


if __name__ == "__main__":
    main()
