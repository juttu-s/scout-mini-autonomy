"""DeepStream pipeline fed from numpy frames: appsrc -> nvvideoconvert -> nvstreammux -> nvinfer -> nvtracker.

Two pad probes collect per-frame results:
  dets[idx]   raw detector boxes after nvinfer   (x1, y1, x2, y2, class, confidence)
  tracks[idx] tracked objects after nvtracker    (x1, y1, x2, y2, class, confidence, track id, tracker conf)
              confidence is -0.1 when the tracker carried the object through a frame without a detection
  done[idx]   time the frame left the tracker, pushed[idx] time it was pushed  -> latency

paced=True mimics a live camera: a leaky queue keeps only the newest frame (the ROS node does the same).
process() is the synchronous form used by detector_node.py: push one frame, wait for its results.
"""
import os
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import pyds  # noqa: E402

FRAME_NS = 33333333  # 30 Hz timestamps
TRACKER_LIB = "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so"


class DSPipeline:
    def __init__(self, infer_cfg, tracker_cfg, width, height, paced=False, tracker_size=(960, 544)):
        Gst.init(None)
        self.dets, self.tracks, self.pushed, self.done = {}, {}, {}, {}
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.next_idx = 0
        self.pipeline = Gst.Pipeline()

        def make(kind, name, **props):
            e = Gst.ElementFactory.make(kind, name)
            if e is None:
                raise RuntimeError("cannot create " + kind)
            for k, v in props.items():
                e.set_property(k.replace("_", "-"), v)
            self.pipeline.add(e)
            return e

        self.src = make("appsrc", "src", format=Gst.Format.TIME, is_live=paced, block=not paced,
                        caps=Gst.Caps.from_string("video/x-raw,format=RGBA,width=%d,height=%d,framerate=30/1"
                                                  % (width, height)))
        queue = make("queue", "q", max_size_buffers=1 if paced else 4, max_size_bytes=0, max_size_time=0,
                     leaky=2 if paced else 0)
        conv = make("nvvideoconvert", "conv")
        # GPU buffer format fed to nvinfer. NV12 halves chroma resolution; RGBA keeps the frame as the ROS node sees it
        caps = make("capsfilter", "caps", caps=Gst.Caps.from_string(
            "video/x-raw(memory:NVMM),format=" + os.environ.get("DS_FORMAT", "RGBA")))
        mux = make("nvstreammux", "mux", batch_size=1, width=width, height=height, live_source=paced,
                   batched_push_timeout=40000)
        infer = make("nvinfer", "infer", config_file_path=infer_cfg)
        self.tracker = None
        if tracker_cfg:
            self.tracker = make("nvtracker", "tracker", ll_lib_file=TRACKER_LIB, ll_config_file=tracker_cfg,
                                tracker_width=tracker_size[0], tracker_height=tracker_size[1])
        sink = make("fakesink", "sink", sync=False)

        self.src.link(queue)
        queue.link(conv)
        conv.link(caps)
        caps.get_static_pad("src").link(mux.get_request_pad("sink_0"))
        mux.link(infer)
        if self.tracker:
            infer.link(self.tracker)
            self.tracker.link(sink)
        else:
            infer.link(sink)
        infer.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, self._probe, False)
        (self.tracker or infer).get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, self._probe, True)
        self.bus = self.pipeline.get_bus()

    def _probe(self, pad, info, tracked):
        batch = pyds.gst_buffer_get_nvds_batch_meta(hash(info.get_buffer()))
        now = time.monotonic()
        l_frame = batch.frame_meta_list
        while l_frame is not None:
            fm = pyds.NvDsFrameMeta.cast(l_frame.data)
            idx = int(round(fm.buf_pts / FRAME_NS))
            objs = []
            l_obj = fm.obj_meta_list
            while l_obj is not None:
                o = pyds.NvDsObjectMeta.cast(l_obj.data)
                r = o.rect_params
                box = (r.left, r.top, r.left + r.width, r.top + r.height, o.class_id, o.confidence)
                objs.append(box + (int(o.object_id), o.tracker_confidence) if tracked else box)
                l_obj = l_obj.next
            with self.lock:
                if tracked:
                    self.tracks[idx] = objs
                    self.done[idx] = now
                    self.cond.notify_all()
                else:
                    self.dets[idx] = objs
            l_frame = l_frame.next
        return Gst.PadProbeReturn.OK

    def start(self):
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("pipeline failed to start")

    def push(self, rgba, idx):
        buf = Gst.Buffer.new_wrapped(rgba.tobytes())
        buf.pts, buf.duration = idx * FRAME_NS, FRAME_NS
        with self.lock:
            self.pushed[idx] = time.monotonic()
        self.src.emit("push-buffer", buf)

    def process(self, rgba, timeout=1.0):
        """Run one RGBA frame through the pipeline: (raw detections, tracks), or None on timeout."""
        idx = self.next_idx
        self.next_idx += 1
        self.push(rgba, idx)
        with self.cond:
            if not self.cond.wait_for(lambda: idx in self.tracks, timeout):
                return None
            self.pushed.pop(idx, None)
            self.done.pop(idx, None)
            return self.dets.pop(idx, []), self.tracks.pop(idx)

    def finish(self, timeout=60):
        """Send EOS and wait until every pushed frame has left the pipeline."""
        self.src.emit("end-of-stream")
        msg = self.bus.timed_pop_filtered(timeout * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if msg is not None and msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            raise RuntimeError("%s: %s" % (err, dbg))
        self.pipeline.set_state(Gst.State.NULL)
