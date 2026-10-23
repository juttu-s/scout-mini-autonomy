# Design decisions

Why the system looks the way it does, what was rejected, and why. Covers everything up to 2026-10-19:
- detection, tracking and re-ID;
- the safety layer;
- the DeepStream evaluation and the hybrid node;
- 3D mapping and navigation on the Scout Mini;
- recording sessions.

Measured numbers are from the Jetson AGX Orin (JetPack 5.1.x, ROS Noetic) unless stated.

---

## 1. Software environment

| Decision | Rejected | Why |
|---|---|---|
| New venv `~/projects/rs_detection/.venv` with `--system-site-packages` | system Python; reusing `~/projects/search/.venv` | The system Python has no torch. Reusing the search venv would tie two projects' dependencies together. System site-packages keeps rospy, cv_bridge and TensorRT visible |
| NVIDIA's JetPack torch 2.0 wheel | `pip install torch` | PyPI wheels for aarch64 have no CUDA |
| torchvision 0.15.1 built from source with `TORCH_CUDA_ARCH_LIST="8.6+PTX"` | `8.7`; plain `8.6`; the pip wheel | `8.7` isn't recognised by this torch. `8.6` builds but has no kernel that runs on Orin (sm_87). PTX is compiled just-in-time. The pip wheel has no CUDA NMS |
| `vision_msgs` built from source in the workspace | `apt install ros-noetic-vision-msgs` | No sudo available to the agent |
| Node launched with `launch-prefix=<venv python>` | activating the venv in each shell | roslaunch starts it correctly from any terminal |

## 2. Detection model
- **YOLO11, pretrained on COCO-80, no training.** The scope was "detect common objects and people now". Training would need a dataset.
- **TensorRT FP16 engine** over PyTorch: about 2× faster at negligible accuracy cost. The engine only works on this device and this TensorRT version, so the `.engine` files are git-ignored and rebuilt per device.
- **`yolo11s` instead of `yolo11n`** after `n` mislabelled a chair and missed people. Cost: 28 → 24 FPS at 1280×720. `yolo11m` stays the next step if accuracy is still short.

## 3. Depth: GPU registration inside the detector
- **Chosen:** the camera runs with `align_depth:=false`. The node aligns raw depth to the colour image on the GPU with torch:
  - back-project each depth pixel to 3D, apply the depth→colour extrinsics, project into the colour camera;
  - `scatter_reduce("amin")` keeps the nearest surface where several depth pixels land on one colour pixel;
  - a min-pool fills the small holes.
- **Rejected:**
  - the driver's `align_depth:=true`: realsense-ros links ROS's librealsense 2.50, which aligns on the CPU. Aligned depth came at ~5 Hz and colour arrived in bursts with ~190 ms latency;
  - rebuilding realsense-ros against the CUDA librealsense 2.57 in `/usr/local`: real risk that the ROS1 wrapper doesn't build, and the GPU path was fast enough. It's still the "proper" fix for the D455 IMU (§14).
- **Validated** against the driver's own aligned image on the same frame: median difference 5 mm, p95 28 mm.
- **Distance per box** = median of valid depth in the **central 50 %** of the box. This keeps background at the edges out. Size comes from the 10th–90th percentile spread.
- **Large boxes are grid-sampled** (~4,000 pixels) for these statistics: 4.5 → 0.4 ms per box, and on 584 identical boxes the result differed by median 0.1 mm.

## 4. Frame handling
- **Latest frame only:** `queue_size=1` and process the newest frame; skip frames rather than build a queue. Latency stays at ~0.1 s instead of growing.
- **No strict sync:** depth is attached if it's newer than `depth_max_age`, instead of using `ApproximateTimeSynchronizer`, which capped FPS at the slower stream.

## 5. Remote viewing
- **Foxglove (websocket bridge)** over `ssh -X` RViz and over a ROS1↔ROS2 bridge. The laptop runs ROS 2 Humble and Foxglove needs no ROS there; X forwarding of OpenGL is slow and fragile.
- **JPEG copy of the annotated image** (`~image_annotated/compressed`: 49 KB instead of 1.2 MB per frame). Drawing and encoding happen only while subscribed. The raw image saturated the link.

## 6. Tracking
- **ByteTrack through ultralytics `model.track`.** Rejected:
  - BoT-SORT: its appearance matching only applies where the IoU is already high, so it can't recover a person who reappears elsewhere;
  - writing a tracker from scratch: ByteTrack's two-stage matching (weak detections keep existing tracks alive) is the right behaviour and comes for free.
- **Class-gated subclass** (`bytetrack_gated`): a person track never matches a non-person detection. Plain ByteTrack moved a person's id onto a bed box and back. Other classes may still match each other, so a remote ↔ laptop label flip doesn't break the track.
- **The track's label is a score-weighted majority vote**, not the current frame's class (no flickering labels).
- **Velocity estimator:** Theil–Sen line over 0.5 s, then drop outliers > 0.25 m, then a least-squares refit. Rejected:
  - exponential smoothing: up to 30 m/s spikes from depth flips;
  - outlier test against the window median: rejects a moving object's own oldest and newest samples, so no velocity above ~1 m/s.
- **Velocity is measured in `odom`** and published in `base_link`, so the robot's own motion is removed.

## 7. Re-identification
- **Model:** `yolo26n-reid` (trained on MSMT17 for person re-ID), as a TensorRT FP16 engine, dynamic batch 1–8: ~3 ms for 4 crops.
- **Layer above ByteTrack:**
  - a new track is **pending ("#?")** until its embedding is compared with identities *not currently visible*. Match → old id; no match after 5 good crops or 2 s → new id;
  - pending tracks have no number, so ids don't flash and stay sequential.
- **Threshold 0.6**, from measured cosine similarities: the same person 0.93–0.95; different people ≤ 0.38.
- **Good crop** = not cut off at the **left/right** image edge, confident, not occluded. The top and bottom edges are not checked, because a close person is always cut off at the top. Requiring a crop fully inside the image disabled re-ID completely.
- **Only class person**, because it's a person re-ID model.

## 8. Safety layer (`scout_safety`)
- **Separate process, system Python, no torch:** a detector crash can't take the safety logic down with it.
- **The only publisher of `/cmd_vel`,** at a fixed rate, merging teleop (priority) and navigation. `twist_mux` isn't installed, and having one gatekeeper is simpler to reason about. **50 Hz,** because ugv_sdk says motion commands "must be called at a frequency >= 50Hz" (it was 20).
- **States:**
  - **SLOW:** speed scaled with distance;
  - **STOP:** forward speed 0, turning and reversing still allowed;
  - **full STOP:** someone within the turning radius;
  - **FAULT:** stale or missing perception → all zero.
- **The stop distance grows with speed:** `d0 + v·t_react + v²/2a`.
- **TTC = when a person's straight-line path relative to the robot enters a safety circle.** Rejected: plain closing speed, which stops for people merely walking past.
- **Person without depth:** if the box is taller than half the image, treat it as very close, so STOP. Closer than the D455's ~0.4–0.5 m range there is no depth, and "unknown" must not mean "safe".
- **Hold window:** the most restrictive state of the last 0.5 s applies. The first live test flickered OK ↔ SLOW every 50–100 ms.
- **Bypass detection:** the node asks the ROS master every 2 s for other `/cmd_vel` publishers. It can't stop them, so it makes them loud.
- **Reverse capped at 0.2 m/s:** the camera sees forward only.
- **Not a certified safety function;** the hardware e-stop stays mandatory. The remote's RC mode overrides the computer.

## 9. DeepStream
- **Evaluated side by side on the same recorded clips before switching.** It wasn't obvious it would help: the camera caps at 30 Hz and most latency is camera, driver and ROS transport.
- **Findings:** 4× throughput and 6 % vs 70 % CPU. Detections are the same model. Of the trackers, NvDCF accuracy was unusable with its default config (6 % coverage on a close person).
- **Hybrid node, chosen:** DeepStream does inference and NvSORT tracking; the existing code does depth, 3D, re-ID and all topics. The `backend` arg switches between ultralytics and DeepStream. Live result: 29 FPS (= camera), 69 ms latency, 15 % CPU, against 21 FPS, 104 ms and 70 % before.
- **Rejected:** a full DeepStream rewrite, which would mean re-implementing depth registration and safety outputs in probes for no extra gain; feeding DeepStream straight from the camera, which would lose the RealSense driver, its sync and its calibration.
- **RGBA buffers** into nvinfer instead of NV12, slightly more accurate. Bilinear GPU scaling didn't help.

## 10. SLAM and mapping
- **2D slam_toolbox:** written, then deleted unused, because the user wanted 3D.
- **DLIO's own map (no installs)** chosen over FAST-LIO2/FAST-LIO-SAM or a loop-closure back end, to start: DLIO already runs and gives good odometry. The known cost is **no loop closure**, so drift accumulates. That is now visible as fattened obstacles (FAILURES F-46).
- **Dense map built offline from the bag** (`bag_to_dense_map.py`). Rejected:
  - changing DLIO's 25 cm input voxel filter: it would change DLIO's odometry;
  - DLIO's map-node leaf at 5 cm: it didn't help, because keyframes are already 25 cm, and it caused system stalls.
- **Map cleaning rules:**
  - ignore returns < 1.2 m from the lidar (people beside or behind the robot while driving);
  - clear the robot's swept volume (±0.35 m of its path, up to robot height): the robot was there, so nothing static can be;
  - voxel hit ≥ 3 times.
- **Recording sessions** (`record_session.sh`): one command writes the bag (camera, lidar + IMU, odometry, TF) and the map together, so a session can be rebuilt or re-processed later, e.g. with RTAB-Map.

## 11. Navigation on the 3D map
- **Localize in 3D, plan in 2D.** The floor is flat indoors, so full 3D planning would add nothing for a ground robot. 2.5D elevation maps were kept for uneven terrain.
- **Localization:** `hdl_localization` (NDT) against `dlio_map.pcd`, predicting motion with DLIO's `odom → base_link`; no IMU. Rejected AMCL: a 2D slice throws away the 3D structure that makes the match robust.
- **Grid from the 3D map:** a height band from floor + 0.08 m to robot height (0.85 m) + 0.1 m. Tables and shelves count; floor and ceiling don't. Free = band empty and the column observed (floor or ceiling seen within 0.3 m), because the MID360 rarely sees the floor.
- **Floor-level frames** `map_floor` / `odom_floor`, 0.76 m below `map` / `odom`, because VoxelLayer can't have a negative `origin_z`.
- **Costmap sources:** MID360 (CropBox filtered) + D455 depth cloud. The camera covers low obstacles near the front, where the MID360 (7° down from 0.76 m) is blind.
- **Global costmap = static + live 3D obstacles + inflation, replanned at 1 Hz.** Static-only was tried after ghost obstacles appeared. It was reverted because a chair not in the map then blocked the path forever.
- **DWA, not TEB:** part of the `navigation` metapackage, fewer dependencies, and enough for a skid-steer at 0.4 m/s.
- **Planner tuning:**
  - DWA: `sim_time` 2.0, 15 × 30 samples, path/goal bias 24/24, `acc_lim_x` 0.6;
  - goal tolerance 0.35 m / 0.5 rad: a skid-steer can't sidestep, so tight tolerances meant turning back and forth at the goal;
  - inflation: global 0.9 m / scaling 2.5 (paths down the middle of aisles), local 0.6 m / 3.5.
- **No rotate recovery:** spinning blind next to people and desks is wrong for a safety robot. The recoveries only clear costmaps.
- **Camera mount estimated from the floor plane** (RANSAC on aligned depth) instead of a tape measure for height and tilt: 0.535 m, −1.1°.

## 12. Patched third-party code
Each patch is marked "local patch" in the source.

**`hdl_localization`:**
- C++17 on aarch64;
- `tf_future_offset`: post-dates `map → odom`, like AMCL's `transform_tolerance`;
- `points_queue_size` (1): newest scan only.

**`ndt_omp`:** no SSE flags on aarch64.

**Why patch at all:** each change is small and the upstream behaviour was the bug. A forked stack was rejected as more to maintain.

## 13. Testing approach
- **Isolated ROS master (port 11312)** for scenario tests whenever the real Scout base was up, because the safety node publishes `/cmd_vel`.
- **Synthetic tests before live ones:**
  - velocity estimator: noise, speeds, depth flips;
  - re-ID: a simulated camera pan;
  - safety node: 20 scenario tests.
- **The robot was only driven by people:** goals came from the user in Foxglove, and the agent monitored and analysed.

## 14. Open decisions
- **Ray-traced (free-space carving) grid,** proposed to remove drift-fattened and moved-object obstacles.
- **D455 IMU:** unload the kernel HID sensor drivers, or rebuild realsense-ros against librealsense 2.57.
- **Loop closure** (pose graph on DLIO, or RTAB-Map) for large areas.
- **Pending measurements:** `cam_x`/`cam_y` and where the MID360 sits on the robot (footprint).
