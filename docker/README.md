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
docker compose -f ~/ros_utils/docker/compose.yaml exec base rosrun scout_nav record_nav.sh lab2   # bags -> ~/bags
docker compose -f ~/ros_utils/docker/compose.yaml exec safety rosrun scout_safety teleop.sh      # keyboard teleop
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
  admitted; it's off by default anyway (FAILURES.md F-50: it stalls the camera).
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
- **`restart: unless-stopped`, not `on-failure`.** When a `required` node dies, roslaunch shuts down and exits
  with code **0**: a killed `safety_node.py` left its container exited, never restarted (tested 2026-10-23). Docker
  backs off between restarts, and `docker compose stop` / `docker kill` count as manual stops and aren't restarted.
- **Reproducible images.** Third-party code is cloned at the commits pinned in `pins/<name>.repos`, with this
  machine's local changes applied from `pins/<name>.patch` (`third_party.repos` is the same information in one
  file, for reading). Each Dockerfile step copies only the pins it uses, so changing one package rebuilds only the
  layers after it, not the hour-long librealsense build. Wheels are checked by SHA-256 against the ones in the
  host venv, and the Python packages are pinned (`perception/requirements.txt`). Run `sync_third_party.sh` after
  changing a third-party package, then commit. Images are tagged `<date>-<git commit>`.

## Failure behaviour

| what dies | what happens | verified |
|---|---|---|
| perception | safety: detections stale > 0.3 s → FAULT → all motion zero. The entrypoint watchdog sees the heartbeat stale (10 s) and restarts the container; the detector re-registers with the surviving master and safety recovers | **yes, 2026-10-23, wheels not involved:** FAULT after 0.304 s, container restarted, safety re-subscribed, OK again after 29.7 s, 30 FPS |
| base | nothing sends CAN commands → wheels stop **only if the Scout firmware has a command timeout** | **blocked on the firmware test** |
| safety | nothing publishes `/cmd_vel`. `scout_base` sends one CAN command per message and doesn't repeat it, so the wheels stop **only via the firmware timeout** | **blocked on the firmware test** |
| master | existing connections keep working; nothing can (re)start → `robot_down.sh` + `robot_up.sh` | to test |

**Gate before trusting the "base" and "safety" rows:** run `rosrun scout_safety firmware_timeout_test.py` with the
wheels off the ground (PROJECT_LOG.md lists it as not done).
- **If the firmware does not stop on its own:** add a command watchdog to `scout_base` (a 50 Hz timer that sends
  zero when `/cmd_vel` is older than 0.2 s), and move `scout_base` into the safety container. The remaining case,
  the CAN process itself dying mid-motion, is then covered only by the RC handset or e-stop.

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
