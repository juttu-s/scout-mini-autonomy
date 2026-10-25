# Scout Mini: perception, safety and navigation

[![CI](https://github.com/OWNER/REPO/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/ci.yml)

ROS Noetic stack for an AgileX Scout Mini on a Jetson AGX Orin, with a RealSense D455 and a Livox MID360:

| package | what |
|---|---|
| [`rs_object_detection`](detection_ws/src/rs_object_detection/README.md) | YOLO (TensorRT / DeepStream) on the D455, 3D positions from depth, tracking, a depth cloud for the costmaps |
| [`scout_safety`](detection_ws/src/scout_safety/README.md) | person-aware speed and separation monitor: the only `/cmd_vel` publisher, FAULT on stale perception |
| [`scout_nav`](nav_ws/src/scout_nav/README.md) | lidar-map localization (hdl_localization), move_base, and a repeatable navigation evaluation |
| [`slam_eval`](slam_eval_ws/src/slam_eval) | harness comparing DLIO, FAST-LIO, LIO-SAM and RTAB-Map on one bag (evo) |
| [`docker/`](docker/README.md) | the stack as four containers (master + safety, perception, base/nav), third-party code pinned by commit + patch |

## CI
[`.github/workflows/ci.yml`](.github/workflows/ci.yml), on every push and pull request:
- **lint:** ruff (errors only), shellcheck, XML / YAML parse;
- **build:** `catkin_make` of the four packages in `ros:noetic`, with `vision_msgs` at the images' pin; unit tests
  for the navigation-evaluation CSV handling and the SLAM metrics;
- **safety scenarios:** the safety image is built for amd64, and each scenario in
  `scout_safety/test/scenario_test.py` runs as its own job against a fresh node with the shipped config, e.g. the
  detector stopping mid-run must give a zero-output FAULT within the staleness window. On master, a failing scenario
  opens (or comments on) an issue with each failed assertion's measured value against its threshold; the bag and
  logs are attached to the run;
- **images** (master, tags, weekly): `safety` and `base` built for arm64; `v*` tags push them to GHCR.
  Perception needs CUDA / TensorRT on JetPack and is built on the robot (`docker/build.sh perception`).

Not tested in CI: anything that needs the robot, the sensors or the GPU.
