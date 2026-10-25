# Scout stack in containers

Four containers that talk over ROS 1, split by **failure domain** rather than one container per process.

| container | image | runs | restart |
|---|---|---|---|
| `master` | scout-safety | `roscore` (rosmaster + rosout) | never: ROS 1 nodes don't re-register with a new master |
| `safety` | scout-safety (ros-core + messages, no GPU) | `safety_node.py`, the only `/cmd_vel` publisher | unless-stopped |
| `perception` | scout-perception (DeepStream 6.3 L4T + ROS + torch) | RealSense driver, camera mount TF, detector | unless-stopped |
| `base` | scout-base (ros-base) | Scout base (CAN), MID360, DLIO, Foxglove; with `--nav` localization + move_base | unless-stopped |

`robot.launch` and `navigation.launch` are unchanged, so the native stack still works as a fallback. The container
launch files in `launch/` include the same pieces with the same defaults.

## Use
```
~/docker_setup.sh                 # once (sudo): docker group; log out and in
~/ros_utils/docker/build.sh       # build + tag images (first time ~2 h, mostly librealsense/torchvision)
~/can_up.sh                       # as before
~/robot_up.sh                     # or: ~/robot_up.sh --nav lab2 [x y yaw]
~/robot_exec.sh base rosrun scout_nav record_nav.sh lab2     # bags -> ~/bags
~/robot_exec.sh safety rosrun scout_safety teleop.sh         # keyboard teleop
                                  # (robot_exec.sh loads the ROS environment; plain `docker compose exec` doesn't)
~/robot_down.sh                   # closes any recording first, then stops perception, base, safety, master
TAG=20261022-abc1234 ~/robot_up.sh    # roll back to an older build (docker images | grep scout-)
```

## Why these choices
- **Host networking for all containers.** The MID360 sends UDP on the host's LAN, `can0` exists only in the host's
  network namespace (SocketCAN raw sockets need no extra capability), and ROS 1 uses random TCP ports. Without NAT
  or port mapping, inter-container ROS traffic is plain localhost TCP, as it is between native processes.
- **The master has its own container and is never restarted.** It must outlive everything else: after a master
  restart, running ROS 1 nodes neither re-register nor reconnect.
- **`init: true` everywhere.** tini runs as PID 1, reaps zombies and forwards signals. `stop_signal: SIGINT` is what
  roslaunch shuts down cleanly on. `base` gets a 120 s stop grace because it may be closing a multi-GB bag.
  `robot_down.sh` also stops recordings itself before `docker compose down`.
- **Health checks read heartbeat files, never `rostopic` or `rosnode`.** Each of those spawns a Python ROS node,
  and this stack is CPU-sensitive. `safety_node.py` and `detector_node.py` touch `/run/scout/<name>.alive` (shared
  tmpfs, at most once a second) when given the `~heartbeat_file` param; it's off by default, so the native stack is
  unchanged. For `base`, `scout_nav/heartbeat_relay.py` is a single persistent node that touches the file while
  `/odom` and `/livox/lidar` are fresh.
- **perception mounts all of `/dev`.** The D455 re-enumerates its USB device on `initial_reset`, so a fixed device
  path breaks. Access is still limited: the device cgroup only admits USB (major 189) and V4L (major 81) character
  devices, the udev rules stay on the host, and it's narrower than `--privileged`. The D455 IMU (HID/IIO) is not
  admitted; it's off by default anyway (it stalls the camera).
- **Bags go to `~/bags`, a bind mount** (`BAG_ROOT`), never into a container layer. `record_session.sh` symlinks
  `~/maps/<name>/recording` to the bag folder, so the offline tools still find them.
- **Image sizes.** scout-safety is about 2.1 GB, almost all of it the official `ros:noetic-ros-core` layers: ROS 1's
  Debian packages pull in cmake, compilers and Boost headers through roscpp and catkin, and a hand-trimmed
  `ubuntu:focal` + rospy/tf2 runtime came out the same size (2.11 GB, tested 2026-10-22). What keeps safety small is
  what it doesn't contain: no CUDA, TensorRT, torch or GPU runtime. On disk, its base layers are shared with
  scout-base (`ros-base` is built on `ros-core`), so it adds about 29 MB.
- **A watchdog in the entrypoint, because Docker never restarts an unhealthy container.** Docker only *reports*
  health, and roslaunch keeps running when a non-required node dies: a killed detector left the camera running and
  the container unhealthy indefinitely (tested 2026-10-23). With `HEARTBEAT_FILE` set (safety, perception, base),
  the entrypoint stops roslaunch once the heartbeat is older than `HEARTBEAT_MAX_AGE` (10 s), after a start-up
  grace period (`HEARTBEAT_GRACE`: 30 / 180 / 90 s). The container then exits and is restarted. This covers hangs
  as well as crashes. The base heartbeat requires `/odom`, `/livox/lidar` and DLIO's odometry.
- **`restart: on-failure`, with the exit code set by the entrypoint, and never started by a reboot.**
  - roslaunch exits 0 when a required node dies, so the entrypoint turns any exit it didn't ask for into 1
    (restarted). A deliberate stop (SIGINT/SIGTERM: `robot_down.sh`, `docker stop`, shutdown) exits 0, so the
    container is not restarted.
  - `unless-stopped`, used before, also restarted everything when Docker started at boot. After a reboot on
    2026-10-23, safety, base and perception came up without a master (`restart: no`) and looped: 24, 11 and 6
    restarts.
  - A power cut leaves a non-zero last exit, which `on-failure` would also restart at boot. To cover that,
    `robot_up.sh` stamps the containers with the kernel boot id (`SCOUT_BOOT_ID`), and a container started in any
    other boot exits 0 at once.
  - Tested: a crash restarts the container; a stop exits 0; another boot id refuses to start, with no restart.
- **Reproducible images.** Third-party code is cloned at the commits pinned in `pins/<name>.repos`, with this
  machine's local changes applied from `pins/<name>.patch` (`third_party.repos` is the same information in one
  file, for reading). Each Dockerfile step copies only the pins it uses, so changing one package rebuilds only the
  layers after it, not the hour-long librealsense build. Wheels are checked by SHA-256 against the ones in the
  host venv, and the Python packages are pinned (`perception/requirements.txt`). Run `sync_third_party.sh` after
  changing a third-party package, then commit. Images are tagged `<date>-<git commit>`.

## Failure behaviour

| what dies | what happens | verified |
|---|---|---|
| perception | safety: detections stale > 0.3 s → FAULT → all motion zero. The entrypoint watchdog sees the heartbeat stale (10 s) and restarts the container; the detector re-registers with the surviving master and safety recovers | **yes**. Without wheels: FAULT 0.304 s, OK again 29.7 s. On the stand, teleop 0.1 m/s (2026-10-23): wheels stopped 0.55 s after the crash, turning again after 34 s |
| base | nothing sends CAN commands → the Scout firmware stops the wheels by itself (timeout 0.60 s, measured natively). Safety keeps running; the container restarts (`scout_base_node` is required) | **yes**, on the stand, teleop 0.1 m/s (2026-10-23): wheels stopped 0.81 s after the crash (CAN frame 0x221), container restarted, turning again after 8.1 s |
| safety | nothing publishes `/cmd_vel`. `scout_base` sends one CAN command per message and doesn't repeat it, so the firmware timeout (0.60 s) stops the wheels; the container restarts | **yes**, on the stand, teleop 0.1 m/s (2026-10-23): wheels stopped 0.84 s after the crash, container restarted, turning again after 7.5 s |
| master | existing connections keep working; nothing can (re)start → `robot_down.sh` + `robot_up.sh` | to test |

**Firmware timeout, measured** (`rosrun scout_safety firmware_timeout_test.py`, wheels off the ground,
2026-10-23): the base stops on its own **0.60 s** after commands stop, so the "base" and "safety" rows are fail-safe,
and the planned fallback watchdog in `scout_base` isn't needed. At the 0.5 m/s cap, that's up to ~0.3 m of motion
before the stop. Still to check: the timeout at higher speed, on the floor.

**Behaviour found in the crash tests (by design today, decide before autonomy):**
- After a safety or base restart, the robot starts moving again on its own if a command source is still
  publishing, because the commands are fresh. For teleop that's the human's command; for `move_base`, a base
  restart also restarts DLIO (its odom resets), and navigation carries on driving on the new odometry.
- While base is down, safety stays `OK`: it doesn't treat stale `/odom` as a fault. That's harmless, because no
  CAN writer exists then. It would matter only if odometry stopped while `scout_base` kept running.

## Performance against native (measured 2026-10-23, robot on the stand, 60 s per run)

Same robot, same view, full stack running, nothing driving. Native was measured twice to show the run-to-run
spread (`latency_probe.py`: detector latency = receive time − camera frame time; DLIO latency = its per-scan path
message − the scan's arrival; CPU from `/proc`, which sees the containers' processes too).

| | containers | native (2 runs) |
|---|---|---|
| camera | 30.0 Hz | 30.0 / 30.0 Hz |
| detector | 23.1 Hz | 24.5 / 24.3 Hz |
| detector latency p50 / p95 | 93.0 / 109.6 ms | 96.6 / 114.4, 96.9 / 114.7 ms |
| `/cmd_vel` rate, jitter (std), worst gap | 50.0 Hz, 0.70 ms, 23.4 ms | 50.0 Hz, 0.76 / 0.75 ms, 24.8 / 23.8 ms |
| DLIO latency p50 / p95 | 20.5 / 23.2 ms | 18.9 / 20.9, 18.5 / 20.7 ms |
| system CPU (12 cores) | 22.0 % | 22.8 / 23.7 % |
| detector / DLIO / safety | 1.01 / 0.51 / 0.17 cores | 1.00 / 0.50 / 0.16 cores |
| camera driver | **0.16 cores** | 0.38 / 0.38 cores |

- **Safety's 50 Hz output is unchanged.**
- **Detector:** latency is slightly lower in containers and the rate about 1.3 Hz lower (one container sample).
  Both are within what the scene and the load change between runs.
- **DLIO is about 1.6 ms slower in containers**, consistently. The build flags are the same (Release, `-O3`).
  Not confirmed: Docker's default seccomp filter adds a small cost to every system call. 1.6 ms is negligible
  against the 100 ms scan period.
- **The camera driver uses less than half the CPU in the container.** Natively, realsense-ros loads ROS's apt
  `librealsense2` 2.50 (libusb/RSUSB backend). The image builds 2.57 from source with CUDA and the kernel backend,
  which is what `~/ros_utils/librealsense` was built for. That's also the likely fix for the D455
  IMU stall, so `camera_imu:=true` is worth retrying in the container.
- **Health checks** cost 4.5 ms per check, every 5 s, per container: about 0.1 % of a core in total.

**Start-up: CUDA JIT cache.** torchvision is built as sm_86 + PTX (torch 2.0 can't target the Orin's sm_87), so a
fresh container spent about 2 minutes JIT-compiling PTX before its first detection, while the 70 MB cache grew in
`~/.nv/ComputeCache`. That folder is now the named volume `scout_cuda_cache`. A fresh perception container reaches
full rate in **21 s instead of 139 s**; only the first start after an image change pays the compile again.

**Recording in containers, tested 2026-10-23.** `~/robot_exec.sh base rosrun scout_nav record_nav.sh lab2` with
navigation running wrote to `~/bags/lab2/nav_runs/<date_time>/`, files owned by `agile`. `~/robot_down.sh` found
the recording, sent SIGINT and waited for it to close (about 12 s), then stopped the containers in reverse order:
20 s in total. Result: `nav_0.bag`, 79 s, 618 MB, 60,893 messages, no `.active` file, `rosbag info` reads it.

## Test checklist (wheels off the ground for all of it until every item has passed)
1. `build.sh` succeeds. In perception: `python3 -c "import torch, pyds, ultralytics; print(torch.cuda.is_available())"` prints `True`.
2. `robot_up.sh`, then check:
   - `/cmd_vel` at 50 Hz;
   - detector FPS the same as native (about 30);
   - Foxglove teleop turns the wheels;
   - safety states as before.
3. While teleop commands motion, crash each container's main node in turn and time it from the crash to the
   wheels stopping (`/scout_status`). Crash means `docker exec <container> pkill -9 -f <node>`: `docker kill`
   counts as a manual stop, so it doesn't exercise the restart path:
   - `docker exec scout-perception-1 pkill -9 -f '[d]etector_node.py'`: safety goes to FAULT within 0.3 s. After the restart, `rosnode info
     /rs_detector` shows the new pid, `rostopic info /rs_detector/tracks` lists `/scout_safety`, and FAULT clears.
   - `docker exec scout-base-1 pkill -9 -f '[s]cout_base_node'`: compare against the firmware test. After the restart, `/odom` is back.
   - `docker exec scout-safety-1 pkill -9 -f '[s]afety_node.py'`: compare against the firmware test. After the restart, motion resumes only on a
     fresh teleop command.
   - `docker compose stop base` during a 2-minute `record_nav.sh`: the bag is closed (no `.active` files) within
     120 s.
4. Performance against native: detector FPS and latency with health checks on and off, safety's 50 Hz jitter, and
   DLIO latency.
5. `robot_up.sh --nav lab2`: set the pose in Foxglove, send one goal (`goal_monitor.py` exec'd in base), and record
   it; the bag lands in `~/bags/lab2/`.
