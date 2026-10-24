# scout_nav

Navigation for the Scout Mini on flat indoor floors, using a **3D lidar map**:

| Part | How |
|---|---|
| Mapping | DLIO (already in `scout_safety/robot.launch`) builds a 3D point-cloud map while you drive with teleop |
| Saving | `save_map.sh` saves the map as `dlio_map.pcd`, then `pcd_to_grid.py` turns it into the 2D grid that `move_base` uses |
| Localization | `hdl_localization` (NDT) matches each MID360 scan to the 3D map, predicts motion with DLIO odometry, and publishes `map → odom` |
| Planning | `move_base`: static layer (the grid) + **3D voxel layer** (live MID360 cloud + D455 depth cloud) + inflation; GlobalPlanner + DWA |
| Output | `/nav/cmd_vel` → `scout_safety` (slows or stops for people; teleop overrides) → `/cmd_vel` → Scout |

## Evaluation
Repeatable goal circuit with one CSV row per attempt (appended across sessions): `scripts/eval_nav_goals.py` with
`config/nav_eval_goals_lab2.yaml`. Setup, goals and safety rules: [`docs/nav_eval_protocol.md`](docs/nav_eval_protocol.md).

## Frames
`map` (hdl_localization) → `odom` (DLIO) → `base_link` (**the Scout's turning centre**, at lidar height) →
`livox_frame` (the MID360, 0.178 m straight ahead: static in `robot.launch` / `base.launch`, and DLIO's
`baselink2lidar`) and `camera_link` (the mount in `robot.launch`, also 0.178 m ahead).
Until 2026-10-23 `base_link` was the lidar itself. Every turn in place then swung it round a 0.18 m circle, and the
footprint sat 0.18 m too far forward. Sessions recorded before then (lab2) have no `base_link → livox_frame` in their
`/tf_static`; the map tools treat that as identity.
The map frame is DLIO's odom frame from the mapping run: its origin is where the robot was when mapping started, with
z = 0 at lidar height.

`map_floor` and `odom_floor` sit `lidar_height` (0.76 m) below `map` and `odom`, so the floor is at z = 0. The costmaps
and `map_server` use them because costmap_2d's VoxelLayer can't have a negative `origin_z`.

## Fixes found on the stand test (2026-10-17)
| Symptom | Cause | Fix |
|---|---|---|
| "sensor origin out of map bounds" | voxel grid below the lidar; `origin_z` is clamped to >= 0 | floor-level frames, floor-relative heights, 16 × 7 cm voxels (reaches above the lidar) |
| lethal cells at the robot itself | the Livox driver outputs (0,0,0) for no-return beams, ~18 % of a scan | `pcl/CropBox` drops ±0.4 m around the lidar → `/livox/lidar_filtered`, used by the costmaps and localization |
| DWA: "Could not transform the global plan" | `map → odom` stamped with the scan time, 0.3–0.7 s old | local patch `tf_future_offset` in hdl_localization (post-dates `map → odom` by 0.5 s, like AMCL's `transform_tolerance`) |
| localization 0.56 s behind, 415 % CPU | NDT_OMP on 10 cm scans using every core | scans downsampled to 25 cm, `OMP_NUM_THREADS=4` → 0.17 s, 148 % CPU, still 98.8 % inliers |
| `/odom` clash | hdl_localization publishes on `/odom` | outputs remapped to `/localization/...` |

## Install (once, needs sudo)
```bash
sudo apt install ros-noetic-navigation
```
`hdl_localization`, `hdl_global_localization`, `ndt_omp` and `fast_gicp` are built from source in `~/ros_utils/nav_ws`.
Two local patches were needed for the Jetson (aarch64) build, both marked "local patch" in their CMakeLists: `ndt_omp` had x86-only SSE flags, and `hdl_localization` used C++11 where PCL 1.10 needs C++14+.

## 1. Map
```bash
roslaunch scout_safety robot.launch foxglove:=true
rosrun scout_safety teleop.sh          # drive slowly through the area, turn gently
rosrun scout_nav save_map.sh lab       # BEFORE stopping the launch: DLIO's map is lost at shutdown
```
This writes `~/maps/lab/`:
- `dlio_map.pcd`: the 3D map;
- `grid.pgm` + `grid.yaml`: the 2D grid;
- `costmap_heights.yaml`: the floor-relative heights for the voxel layers;
- `grid_info.yaml`: the detected floor height and point counts.

The MID360's centre is **0.76 m above the floor** (measured). `save_map.sh` passes that to `pcd_to_grid.py`, because auto-detection is unreliable: the MID360 looks at most 7° down, so from 0.76 m it only sees the floor ~6 m away. To rebuild a grid with other settings:
```bash
rosrun scout_nav pcd_to_grid.py ~/maps/lab --lidar-height 0.76 --robot-height 0.85
```
`robot.launch` sets DLIO's map node to 5 cm voxels; the DLIO default of 25 cm left walls as dotted lines. For maps saved before that change, add `--min-points 1`.
Grid rules:
- **occupied:** points between floor + 0.08 m and floor + robot height + 0.1 m. So a table top at robot height counts as an obstacle, while the floor and the ceiling don't.
- **free:** nothing in that band, and the floor or ceiling was observed within 0.3 m.
- **unknown:** everything else.

People walking around during mapping leave "ghost" obstacles. Map an empty area, or clean up `grid.pgm` in an image editor.

## 2. Navigate
```bash
roslaunch scout_nav navigation.launch map:=lab            # robot at the mapping start pose
roslaunch scout_nav navigation.launch map:=lab init_x:=2.0 init_y:=1.0 init_yaw:=1.57
```
- **Initial pose:** if the robot doesn't start at the map origin, pass `init_*`, or publish a pose estimate on `/initialpose` (Foxglove 3D panel → publish pose estimate, or RViz "2D Pose Estimate").
- **Goals:** send them to `/move_base_simple/goal` (Foxglove "publish pose", RViz "2D Nav Goal") or through the `move_base` action.
- **Check localization:** `rostopic echo /localization/status` should show `has_converged: true` and an inlier fraction near 1. `/localization/aligned_points` is the live scan placed in the map.

## Tested so far (2026-10-17, robot standing still)
- **Saving:** `save_map.sh` → `dlio_map.pcd` + grid. One stationary keyframe gave a sparse map: floor detected from only 18 points (hence `--lidar-height`), ceiling at 2.4–3.0 m.
- **Localization:** `hdl_localization` on that map converged (inlier fraction 0.985, matching error 0.02). `map → odom` came out ≈ identity, as expected, and its topics are remapped away from the Scout's `/odom`.
- **`lab` map on the stand:** localization converged (98.8 % inliers, 0.9 cm error). A 1 m goal produced a global plan and DWA local plans, and `move_base` sent 0.10 m/s / 0.20 rad/s through the safety layer to `/cmd_vel`. Cancelling returned it to 0. The wheels didn't turn because the base was in RC mode (remote off).
- **Not yet tested:** driving to a goal on the floor, and localization while moving.

## Known limitations
- **Low obstacles near the robot:** the MID360 can't see them, but the D455 depth cloud (`/rs_detector/depth_points`) is a second voxel-layer source. It covers the front only (~87°), 0.3–3 m, and marks from 0.10 m above the floor. The sides and the rear still rely on the lidar.
- *(why the camera is needed)* **The lidar alone misses low obstacles near the robot.** The MID360 sees at most 7° down: from its 0.76 m height, the obstacle band (up to 0.6 m above the floor) starts to appear only ~1.3 m away, and floor-level objects only ~6 m away. Close, low obstacles need the camera's depth in the local costmap; that's the next step. People are still covered by the safety layer.
- **No reversing:** DWA is set to forward-only (`min_vel_x: 0`), since nothing watches behind the robot.
- **Footprint placeholder:** it's centred on the lidar. Measure where the MID360 sits on the robot and adjust `footprint` in `config/costmap_common.yaml`.
- **No global relocalization:** if localization is lost, set the pose again (`hdl_global_localization` is built but not enabled).

## Camera in the costmap (2026-10-17)
- **Point cloud:** the detector publishes `~depth_points`, a cloud made from the registered depth: every 4th pixel, 0.3–3.5 m, ≤ 10 Hz (7.4 Hz measured with navigation running), only while subscribed.
- **Camera mount:** `rosrun scout_nav estimate_camera_mount.py` fits the floor plane in the aligned depth (RANSAC) and prints `cam_z/cam_pitch/cam_roll` for `robot.launch`.
  - Measured: camera **0.535 m** above the floor → `cam_z -0.225` (it was a +0.20 placeholder), pitch −1.1°, roll 0.1°.
  - Check: floor points in `odom_floor` come out at +0.038 m median (p5 0.026), under the camera's 0.10 m obstacle threshold.
  - `cam_x`/`cam_y` can't be seen from the floor and are still placeholders: measure them.

## Final-heading oscillation (2026-10-17)
The robot reached the goal and then turned back and forth until `move_base` aborted ("Robot is oscillating. Even after executing recovery behaviors"). With 0.3–0.8 rad/s turning, 2.0 rad/s² acceleration and a 0.2 rad heading tolerance, the skid-steer base with localization lag overshot the final heading every time.
- Now: `max_vel_theta` 0.5, `min_vel_theta` 0.15, `acc_lim_theta` 0.8, `yaw_goal_tolerance` 0.3 rad (~17°), `xy_goal_tolerance` 0.2 m.
- If it still hunts, the next step is raising `yaw_goal_tolerance`. If the base doesn't turn at all at 0.15 rad/s (skid-steer friction), raise `min_vel_theta` a little.

## Recording a session (map + rosbag for RTAB-Map etc.)
```bash
sudo modprobe gs_usb && rosrun scout_bringup bringup_can2usb.bash   # once per boot
rosrun scout_nav record_session.sh lab2            # starts the stack with the D455 IMU, records
rosrun scout_safety teleop.sh                      # other terminal: drive slowly
# Ctrl+C in the record terminal: bag closed -> DLIO map + grid saved -> stack stopped
```
- **Output:** `~/maps/<name>/`:
  - the navigation map (`dlio_map.pcd`, `grid.*`, `costmap_heights.yaml`), built with the 0.76 m lidar height and 5 cm map voxels;
  - `recording/<name>_N.bag` in 4 GB chunks;
  - `launch.log`;
  - `session.txt`: settings, camera mount, topic list, `rosbag info`.
- **Camera IMU:** `--camera-imu` also records the D455 IMU. It's **off by default**: enabling it stalls the whole camera (0 Hz, "hwmon ... HW not ready") with ROS's librealsense 2.50 (RSUSB backend) while the Jetson kernel's HID sensor drivers are loaded.
- **Data rate:** ~90 MB/s, ~5.5 GB per minute. `--no-raw-depth` drops the raw depth image (~3.5 GB/min).
- **Existing stack:** if one is already running, the script uses it. It warns if the camera IMU is missing, because that needs `camera_imu:=true` on `robot.launch`.
- **Refuses to start** if `can0` is down (`scout_base_node` is required and aborts without it) or if the name already exists.

**Recorded topics**

| Group | Topics |
|---|---|
| Camera (D455, 848×480 @ 30) | `/camera/camera/color/image_raw` + `camera_info`; `/camera/camera/aligned_depth_to_color/image_raw` + `camera_info` (depth registered to color by the detector, 16UC1 mm, color stamps); `/camera/camera/depth/image_rect_raw` + `camera_info`; `/camera/camera/extrinsics/depth_to_color` |
| IMU 1: D455 | `/camera/camera/imu`: gyro 200 Hz + accel 100 Hz, linear interpolation; m/s², **no orientation** |
| Lidar + IMU 2: MID360 | `/livox/lidar` (PointCloud2, `livox_frame`); `/livox/imu`: **acceleration in g**, not m/s² |
| Odometry | `/dlio/odom_node/odom` (lidar-inertial, 100 Hz); `/odom` (wheels); `/scout_status` |
| TF | `/tf`, `/tf_static` (camera frames, `base_link → camera_link`) |
| Context | `/cmd_vel`, `/teleop/cmd_vel`, `/nav/cmd_vel`, `/scout_safety/state`, `/rs_detector/{tracks,detections}` |

**Notes for RTAB-Map later**
- **Inputs:** rgb `/camera/camera/color/image_raw`, depth `/camera/camera/aligned_depth_to_color/image_raw`, camera_info `/camera/camera/color/camera_info`. Use `approx_sync` off: they share stamps.
- **Odometry:** DLIO's `/dlio/odom_node/odom` (`frame_id` `/odom`, `child_frame_id` `/base_link`), or let RTAB-Map compute visual odometry. `scan_cloud`: `/livox/lidar`.
- **Robot frame:** `base_link` is the MID360, 0.76 m above the floor.
- **IMUs:** RTAB-Map needs IMU orientation, so run `imu_filter_madgwick` on `/camera/camera/imu` first. If using `/livox/imu`, scale its acceleration by 9.81.
- **Missing aligned depth:** if the detector wasn't running, `/camera/camera/aligned_depth_to_color` will be missing. Re-register offline from the raw depth with `depth_image_proc/register` (that's why raw depth and the extrinsics are recorded).

## Dense map from a recording (recommended after `record_session.sh`)
```bash
rosrun scout_nav bag_to_dense_map.py ~/maps/lab2      # ~1.5 min for a 10 min session
```
- **Why it's needed:** DLIO's saved map is sparse even with the 5 cm map node setting, because its keyframes are already voxelized at 25 cm by DLIO's input preprocessing (needed for odometry, so left alone).
- **What it does:** the script places every full-resolution MID360 scan from the bag at DLIO's pose for the middle of the scan, merges them into 5 cm voxels, and keeps voxels hit ≥ 3 times (drops people and stray returns).
- **Output:** it replaces `dlio_map.pcd` (the original is kept as `dlio_map_sparse.pcd`) and rebuilds the grid, so `navigation.launch map:=lab2` uses it unchanged.
- **lab2 (2026-10-18):** 5,891 scans, 107 M points → **1.67 M voxels** (DLIO's map: 57 k). Grid: 44,438 occupied cells (was 1,541), with solid walls and desk blocks.

## lab2 floor test (2026-10-18)
| Attempt | Problem | Fix |
|---|---|---|
| map | your own trail (people following the robot) stamped as walls: 51 % of the driven path occupied | `bag_to_dense_map.py`: ignore returns < 1.2 m from the lidar, clear the swept volume (±0.35 m of the path, below robot height) → 0 % |
| start-up | localization from (0,0) locked onto a wrong pose (80 % inliers, pose jumping 0.8 m) | set the real pose: `/initialpose` (Foxglove pose estimate) or `init_x/init_y/init_yaw`; check inliers > 0.95 |
| goal 1 | "Failed to find a valid plan": the global costmap had ghost obstacles stamped while localization was wrong | global costmap = static + inflation only (live obstacles via local costmap / DWA / safety) |
| goal 1 | recovery spin at 1.0 rad/s | `rotate_recovery` reads `~/TrajectoryPlannerROS`: `max_rotational_vel` 0.5 |
| goal 2 | "Failed to find a valid control": `map_floor ← odom_floor` extrapolation, localization ~0.7 s behind while driving | hdl_localization local patch `points_queue_size` = 1 (newest scan only; was 5 → backlog), `tf_future_offset` 0.7 s; delay now ≤ 0.39 s |
| **goal 3** | **reached: 79.8 s, 8.4 m driven, 0.14 m final error, 0 transform errors**, inliers ≥ 0.997 | — |

Still to tune: driving was hesitant (average 0.12 m/s, 19 turn-direction reversals over the drive). Candidates: DWA `path_distance_bias` / `goal_distance_bias`, `sim_time`, `forward_point_distance`, and a less tight `min_vel_theta`.
