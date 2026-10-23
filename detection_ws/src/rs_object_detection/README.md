# rs_object_detection

Real-time YOLO (COCO-80) detection on a RealSense D455 with 3D object positions from depth.
Runs on the Jetson AGX Orin with a TensorRT FP16 engine at 1280x720 / 30 Hz: ~27 FPS with `yolo11s` (default; ~24 before the per-box depth speedup) or ~28 FPS with `yolo11n` (measured before it), ~100 ms camera-to-detection latency.
ByteTrack tracking (on by default) adds stable ids, trails and 3D velocity, at ~20 FPS with `yolo11s`.

## Environment
- Python venv: `~/projects/rs_detection/.venv` (system site packages + NVIDIA torch 2.0, torchvision 0.15.1 built
  with `TORCH_CUDA_ARCH_LIST=8.6+PTX`, ultralytics 8.4.174)
- Workspace: `~/ros_utils/detection_ws` (also contains `vision_msgs` built from source), sourced by `~/ros_utils/setup.bash`

## Run
```bash
# Terminal 1 - camera. Note align_depth:=false: the detector aligns depth itself on the GPU.
source ~/ros_utils/setup.bash
roslaunch realsense2_camera rs_camera.launch camera:=camera/camera align_depth:=false tf_prefix:=camera

# Terminal 2 - detector (+ RViz)
source ~/ros_utils/setup.bash
roslaunch rs_object_detection detection.launch rviz:=true
```
Or start both at once with `launch_camera:=true`.

### View from a laptop (Foxglove)
Add `foxglove:=true` to start the Foxglove websocket bridge with the detector (port 8765, change with `foxglove_port:=`):
```bash
roslaunch rs_object_detection detection.launch foxglove:=true
```
On the laptop, open https://app.foxglove.dev or the Foxglove desktop app, choose **Open connection → Foxglove WebSocket**, and enter `ws://<orin-ip>:8765` (`hostname -I` on the Orin).
Add an **Image** panel for `/rs_detector/image_annotated`, and a **3D** panel for `/rs_detector/markers` + TF with the fixed frame set to `camera_link`.
No ROS install is needed on the laptop.

Launch args: `model` (path to `.pt` or `.engine`, default `models/yolo11s.engine`; `models/yolo11n.engine` is the faster, less accurate option), `conf`, `camera_ns` (default `/camera/camera`),
`register_depth`, `tracking` (default true), `tracker_cfg` (default `config/tracker.yaml`), `launch_camera`, `rviz`. Other parameters are in `config/detector.yaml` (e.g. `classes: [0]` to detect only people).

### Why not `align_depth:=true`?
`realsense-ros` here links the apt `librealsense2` 2.50 from `/opt/ros/noetic` (CPU only), not the CUDA build in `/usr/local`.
Its CPU alignment drops aligned depth to ~5 Hz and delivers color in bursts with ~200 ms latency.
With `align_depth:=false`, color and depth arrive at a steady 30 Hz, and the node reprojects raw depth into the color image on the GPU.
That matches the driver's aligned image to ~5 mm (median).
To use the driver's aligned topic instead: `align_depth:=true` on the camera and `register_depth:=false` on the detector.

## Topics (`/rs_detector/...`)
| Topic | Type | Content |
|---|---|---|
| `detections` | `vision_msgs/Detection3DArray` | class id, score, 3D position (m) and approx. size, in `camera_color_optical_frame` |
| `detections_2d` | `vision_msgs/Detection2DArray` | pixel boxes |
| `image_annotated` | `sensor_msgs/Image` | boxes, labels, distance |
| `markers` | `visualization_msgs/MarkerArray` | 3D boxes + labels for RViz, velocity arrows when tracking |
| `tracks` | `rs_object_detection/TrackedObjectArray` | tracking only: id, class, score, box, 3D position + velocity (m/s), size, age |

Class ids are COCO indices (0 = person, 39 = bottle, 56 = chair, 63 = laptop, ...).

3D position = median depth over the central `roi_fraction` of each box, deprojected through the color intrinsics.

## Tracking
`model.track()` with ByteTrack. YOLO runs at conf 0.1 and `config/tracker.yaml` decides what is shown: tracks start at ≥ 0.4, then weak detections keep existing tracks alive.
The node adds per-track state: a trail, a majority-vote class, and a velocity fitted robustly (Theil–Sen + least squares) to the last 0.5 s of 3D positions, ignoring depth flips.
Tuning: `velocity_*`, `track_history` and `track_timeout` in `config/detector.yaml`. Disable with `tracking:=false`.

## Re-identification (people)
ByteTrack only matches by predicted position, so turning the camera away and back used to give a person a new id.
New person tracks are now shown as `#?` while their appearance (a `yolo26n-reid` TensorRT embedding) is compared with people who are not currently visible. A match (cosine ≥ 0.6) gets the old id back; people are remembered for 300 s.
The tracker is also class-gated (`gate_classes: [0]` in `config/tracker.yaml`), so a person's track can't jump onto another object.
Build the engine once: see `PROJECT_LOG.md` section 9. Settings: `reid_*` in `config/detector.yaml`. Disable with `reid_model:=""`.
Detections without enough valid depth appear only in `detections_2d`.

## Re-export the TensorRT engine
Engines are tied to the TensorRT version and GPU, so rebuild after a JetPack upgrade, or to switch models (e.g. `yolo11m.pt`):
```bash
cd ~/ros_utils/detection_ws/src/rs_object_detection/models
~/projects/rs_detection/.venv/bin/yolo export model=yolo11s.pt format=engine half=True imgsz=640 device=0
```

## On the Scout Mini
`scout_safety/launch/robot.launch` runs this detector with `output_frame:=base_link velocity_frame:=odom`:
- `~tracks` positions are then in `base_link`;
- velocity is measured in `odom`, so the robot's own motion is removed.

With tracking on, `~detections`/`~detections_2d` carry every confident box of the frame, including tracks not yet confirmed, so a safety consumer sees a new person on the first frame. Markers and drawing follow the tracks.
See `scout_safety/README.md`.

## DeepStream backend (`backend:=deepstream`)
Detection and tracking run in an NVIDIA DeepStream 6.3 pipeline (`scripts/ds_pipeline.py`):
`appsrc → nvvideoconvert (RGBA) → nvstreammux → nvinfer (YOLO11s, FP16) → nvtracker (NvSORT)`.
The node pushes each color frame and waits ~20 ms for its results. Depth registration, 3D positions, re-ID, robot frames and all topics are the same code as the ultralytics backend.
`scout_safety/robot.launch` uses it by default.

| Live, 848×480 @ 30 Hz | ultralytics | deepstream |
|---|---|---|
| FPS | ~21 | **~29 (every frame)** |
| Image → detections latency p50 / p95 | 104 / 118 ms | **69 / 94 ms** |
| CPU (avg of 12 cores) | 70 % | **15 %** |

Files:
- `config/deepstream/config_infer_yolo11s.txt` (paths are relative to it);
- `config/deepstream/tracker_NvSORT.yml`;
- in `models/` (gitignored): `yolo11s_ds.onnx` (DeepStream-Yolo export, *not* the ultralytics ONNX), its cached engine `yolo11s_ds_b1_gpu0_fp16.engine`, `coco_labels.txt`, `libnvdsinfer_custom_impl_Yolo.so`.

The launch file sets `LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1` for this backend (the tracker library won't load after torch without it).
Recreating the assets is covered in `~/projects/rs_detection/deepstream/README.md`; the evaluation is in `PROJECT_LOG.md` section 11.
