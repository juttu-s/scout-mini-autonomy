# scout_safety

Person-aware speed and separation monitoring for the AgileX Scout Mini, using the RealSense D455 person
detections, tracks and 3D positions from `rs_object_detection`.

> **This is not a certified safety function.** It is a perception-based monitoring layer written in Python.
> Keep the Scout's hardware e-stop within reach, especially during testing.

## What it does
`scripts/safety_node.py` sits between the command sources and the robot. It is the **only publisher of
`/cmd_vel`**, at a fixed 20 Hz, and outputs zeros whenever nothing is allowed to move.

1. **Command input:** it picks the highest-priority input that is fresh (newer than 0.5 s):
   - `/teleop/cmd_vel` (priority 10): a human always overrides autonomy;
   - `/nav/cmd_vel` (priority 5).
2. **People:** it reads `/rs_detector/detections` (3D) and `/rs_detector/detections_2d`, transforms people into
   `base_link`, and decides a state:

| State | When | Effect |
|---|---|---|
| OK | nobody limits the motion | command passed through (capped at `max_speed`) |
| SLOW | a person in the corridor ahead within `slow_distance`, or a person seen without depth | forward speed scaled from `max_speed` (at `slow_distance`) down to 0 (at the stop distance), never below `min_slow_speed` while > stop distance |
| STOP | a person in the corridor closer than the stop distance; a tracked person whose path enters the safety circle within `ttc_stop`; a person box taller than half the image without depth | forward motion 0. Reversing (≤ `max_reverse_speed`) and turning are allowed so the robot can back off or turn away |
| STOP (full) | anyone within `footprint_radius + turn_margin` of the robot, or very close without depth | all motion 0, including turning |
| FAULT | no detector output, detections older than 0.3 s (stalled detector or frozen camera), or no `camera → base_link` TF | all motion 0; needs 1 s of healthy input to recover |

- **Clearances** are measured from the front bumper: base_link x minus `front_offset`.
- **The corridor** is `|y| < half_width + lateral_margin`.
- **The stop distance grows with speed:** `d_stop = stop_distance + v·reaction_time + v²/(2·decel)`.
  With the defaults at 0.5 m/s: 0.8 + 0.15 + 0.125 = **1.08 m**.
- **Time to collision** uses `/rs_detector/tracks`, which are in `base_link` with the robot's own motion removed (velocity fitted in `odom`). The person's velocity relative to the robot is followed in a straight line, and the node computes when that line enters the circle `footprint_radius + turn_margin`. People walking past outside that circle don't trigger it.
- **Hysteresis:** the most restrictive result of the last `clear_time` (0.5 s) applies, for STOP *and* SLOW. A person flickering at the corridor edge, or a box dropped for one frame, doesn't toggle the speed. The first live test showed OK ↔ SLOW every 50–100 ms without this.

Monitoring outputs:
- `/scout_safety/state` (JSON string): state, reason, nearest clearance, stop distance, TTC, active input, command in and out, detection age.
- `/scout_safety/markers`: stop and slow zone outlines in `base_link`, coloured by state, for Foxglove/RViz.

## Run on the robot
```bash
# once per boot (needs sudo)
sudo modprobe gs_usb && rosrun scout_bringup bringup_can2usb.bash

source ~/ros_utils/setup.bash
roslaunch scout_safety robot.launch cam_x:=<m> cam_y:=<m> cam_z:=<m> cam_pitch:=<rad> [foxglove:=true]

# drive (teleop must NOT publish to /cmd_vel directly)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/teleop/cmd_vel
# autonomy: remap the planner's cmd_vel to /nav/cmd_vel
```
`robot.launch` starts:
- the Scout base, the Livox MID360 and DLIO (same as `dm3_pkg/dm3.launch`, which is unchanged);
- the D455 at 848×480 @ 30 with `align_depth:=false` (the detector aligns depth on the GPU);
- the static `base_link → camera_link` transform;
- the detector with `output_frame:=base_link velocity_frame:=odom` and `backend:=deepstream`: 29 FPS, ~70 ms image→detection latency, ~15 % CPU. Use `backend:=ultralytics` for the previous inference path;
- this node (`required`: if it dies, the launch shuts down).

Bench test without the robot: `robot:=false` (camera + detector + safety only; velocity is then not odom-compensated).

## What drives `/cmd_vel` (checked 2026-10-13)
| Source | Status |
|---|---|
| `scout_base_node` | the only subscriber of `/cmd_vel`; nothing else was publishing |
| ROS navigation packages (`move_base`, planners, `twist_mux`) | not installed |
| `teleop_twist_keyboard` | installed. By default it publishes **`/cmd_vel` directly**; use `rosrun scout_safety teleop.sh`, which remaps it to `/teleop/cmd_vel` |
| `~/projects/search/SemanticSearch` (GOAT) | publishes `/cmd_vel` directly (`home_robot_hw/remote/ros.py`); not used with this stack |

**Anything that publishes `/cmd_vel` itself bypasses the safety layer.** The node checks the ROS master every 2 s, and logs an ERROR naming any other `/cmd_vel` publisher (also in `~state` as `bypass`).

**Plugging in a navigation stack later:**
- Remap its `cmd_vel` to `/nav/cmd_vel`.
- Odometry is on `/odom` (wheels) and `/dlio/odom_node/odom` (DLIO).
- Aligned depth is on `/camera/camera/aligned_depth_to_color/image_raw` + `camera_info`. The detector republishes its GPU-registered depth there (`publish_aligned_depth:=true` in `robot.launch`), so don't also turn on the driver's `align_depth`.
- Expect it to be paused: in SLOW/STOP the commanded speed is reduced or zero, so a planner may need a time-out or replan instead of assuming its commands are followed.

## Bring-up checklist (robot on a stand, wheels off the ground)
1. **Stop any Scout bringup already running.** `robot.launch` starts its own `scout_base_node`.
2. **Firmware timeout test**, without the safety stack: start only `roslaunch scout_bringup scout_robot_base.launch`, then
   `rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'`, and Ctrl+C the publisher. Do the wheels stop by themselves, and how fast?
   If they keep turning, a crashed safety node would leave the last command active, so don't do floor tests until that is handled.
3. **Measure** the camera mount (`cam_x/y/z/pitch`) and `front_offset`.
4. **Launch:** `roslaunch scout_safety robot.launch cam_x:=… cam_z:=… cam_pitch:=… foxglove:=true`, then
   `rostopic info /cmd_vel`. The **only** publisher must be `/scout_safety`.
5. **Drive:** `rosrun scout_safety teleop.sh` (speed 0.2 m/s; `SPEED=0.3 rosrun …` to change). Hold `i` to drive forward and walk in front:
   - wheels slow, then stop; turning (`j`/`l`) still works in STOP unless you're right next to the robot;
   - `rosnode kill /rs_detector` → wheels stop (FAULT).
6. **Then the floor:** `max_speed: 0.3`, a second person at the e-stop.

## Before the first drive: measure and set
| What | Where | Placeholder |
|---|---|---|
| Camera mount relative to `base_link` (DLIO's body frame) | `robot.launch` args `cam_x/y/z/yaw/pitch/roll` | 0.30 / 0 / 0.20 / 0 / 0 / 0 |
| base_link → front bumper | `front_offset` | 0.31 m |
| Half body width, footprint radius | `half_width`, `footprint_radius` | 0.30, 0.42 m (Scout Mini ≈ 0.61 × 0.58 m) |
| Braking deceleration on your floor | `decel` | 1.0 m/s² |

Check the mount: a person standing at a tape-measured 2.0 m in front of the bumper should show `nearest ≈ 2.0` in `/scout_safety/state`.

## Test procedure
1. **Offline logic tests** (`test/scenario_test.py`; 15 scenarios, each against a freshly started node with `config/safety.yaml`; CI runs each as its own job and opens an issue on failure). Run them in the safety image, which has its own master, so it's safe while the robot is up:
   ```bash
   docker run --rm --entrypoint bash -v ~/ros_utils/detection_ws/src/scout_safety/test:/test -v /tmp/out:/out \
       scout-safety:latest /test/run_scenario.sh --all /out      # or <scenario id> instead of --all
   ```
   Each writes `result.json` (each assertion with measured value vs threshold), a bag and the node log. Synthetic detections, tracks and commands. Covered:
   - zones and speed scaling, turning while stopped, the reverse cap;
   - teleop overriding navigation, input timeout;
   - people without depth;
   - a TTC stop for someone running at the robot, but not for someone off to the side;
   - STOP hold, no flicker for a person jumping in and out of the corridor, rogue `/cmd_vel` publisher reported;
   - detector stops publishing mid-run: FAULT within `max_detection_age` + 2 cycles (0.39 s; 0.31–0.32 s measured), every `/cmd_vel` exactly zero while teleop keeps commanding, then OK only after `recover_time`.
2. **Robot on a stand, wheels off the ground:** drive with teleop, then:
   - walk toward the camera and check the OK → SLOW → STOP transitions;
   - kill the detector: the wheels must stop within ~0.3 s;
   - kill the safety node: check that the Scout stops by itself on CAN command timeout. `scout_base` has no timeout in its own code, so this depends on the firmware. If it doesn't stop, don't drive on the ground until a guard is added.
3. **Reaction time:** compare the image stamp of the first frame with a person inside the stop zone with the first zero `/cmd_vel`. On the bench with the ultralytics backend, image → detections was 0.10 s typical (0.12 s max), and the safety node saw detections 0.12 s old typical (0.17 s max). With the DeepStream backend, image → detections is 69 ms typical / 94 ms p95. The robot's own CAN/motor delay comes on top.
4. **Ground test:** `max_speed: 0.3` first, an open area, and a second person at the e-stop. Tape-measure the actual stopping distance against `d_stop`.

## Known limitations
- **Forward view only:** the D455 sees ~87° × 58°. Sides and rear are blind, hence `max_reverse_speed` (0.2 m/s). The MID360 lidar sees 360° but isn't used here yet.
- **Close range:** the D455's minimum depth range is ~0.4–0.5 m. A person closer than that has no depth, which is handled by the box-size rule (big box → STOP).
- **Missed detections:** YOLO can miss people in poor light, when heavily occluded, in unusual poses, or when only partly in view. The stop logic only knows what the detector reports.
- **Classes:** only COCO class 0 (person) is protected by default (`classes`). Other obstacles need a separate obstacle check, e.g. from the depth image or the lidar.
- **Python, not real-time:** a scheduling hiccup shows up as stale detections and is handled as FAULT (fail-safe), not as missed protection.
