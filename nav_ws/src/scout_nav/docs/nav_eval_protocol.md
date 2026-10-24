# Navigation evaluation protocol: lab2

A fixed goal circuit, driven the same way every session, so results can be compared over time and across software
changes. One CSV row per goal attempt, appended across sessions.

- Script: `scout_nav/scripts/eval_nav_goals.py`
- Goals: `scout_nav/config/nav_eval_goals_lab2.yaml`
- Results: `~/maps/lab2/nav_eval/nav_eval.csv`

> **This is not a certified safety function.** The robot drives itself between goals. Read section 7 before every
> session. An operator with the e-stop in reach is required for every goal.

## 1. What a row means

One goal attempt: the robot is standing still, the script sends one goal to move_base, and the row records what happened
until move_base reports a result, the timeout expires, or the operator cancels.

| column | meaning |
|---|---|
| `session_id`, `start_time`, `git_rev`, `operator`, `notes` | which session; `git_rev` is the `~/ros_utils` commit (`-dirty` = uncommitted changes in scout_nav) |
| `lap`, `goal_name`, `goal_x/y/yaw_deg` | the goal, in the `map` frame |
| `start_x/y/yaw_deg` | robot pose when the goal was sent |
| `result` | `SUCCEEDED`, `ABORTED` (move_base gave up), `PREEMPTED`, `REJECTED`, `TIMEOUT` (cancelled by the script), `CANCELLED` (Ctrl+C), `SKIPPED` |
| `status_text` | move_base's text, e.g. "Failed to find a valid plan. Even after executing recovery behaviors." |
| `time_s` | goal sent → result |
| `final_x/y/yaw_deg`, `err_xy_m`, `err_yaw_deg` | last `/localization/odom` pose and its error to the goal (success means within 0.35 m and 29°, the DWA tolerances) |
| `path_len_m`, `straight_m`, `path_ratio` | driven length (steps ≥ 2 cm of `/localization/odom`), start → goal straight line, and their ratio |
| `recoveries_n`, `recoveries` | recovery behaviours run (`/move_base/recovery_status`). Only `conservative_reset` / `aggressive_reset` (costmap clears) are configured; there is no rotate recovery |
| `tf_errors_n`, `tf_error_first` | WARN+ log lines from any node about transforms (extrapolation, lookups, `TF_`), plus the script's own failed `map → base_link` lookup at the end |
| `planner_errors_n`, `control_errors_n` | move_base "NO PATH" / "Failed to get a plan" and "valid control" / "DWA planner failed" log lines |
| `safety_ok/slow/stop/fault` | `/scout_safety/state` messages per state (the node publishes at 50 Hz, so 50 ≈ 1 s) |
| `min_inlier` | worst localization inlier fraction during the goal (a drop means localization struggled) |
| `costmaps_cleared` | 1 = costmaps cleared just before the goal (the default; see section 5) |

Expected values for a clean lap (2026-10-23): plan lengths 5.2, 6.6, 1.1, 9.0, 10.1, 1.2, 6.6 m (A … F, home2).
So `path_ratio` should be near 1.1–1.4, and well above that means detours or weaving.

## 2. Physical setup (every session)

1. **Area:**
   - the lab2 desk area, with the furniture as mapped: chairs pushed in under the desks;
   - nothing standing in the aisles. On 2026-10-23, two unmapped objects of about 0.3 m stood at map (2.93, −2.62) and
     (3.48, −3.97), on the D → E and E → F legs. Remove such things, or record them in `--notes`.
   - Note any furniture you couldn't put back in `--notes`.
2. **People:**
   - Close the aisles on the route. An observer at each entrance, or signs, keeps people out of the circuit while a
     goal runs.
   - Nobody walks next to or behind the robot except the operator (section 7, items 1–2).
3. **Start mark:**
   - Floor tape at `home2`, map (2.08, 0.33). Put it there once by driving the robot there with localization good and
     taping the robot's **centre**. `base_link` is the turning centre since 2026-10-23; the lidar is 0.178 m (7 in)
     ahead of it.
   - Place the robot on the tape facing 83° in the map, i.e. towards goal A.
   - The first home, (−0.22, 0.28), was dropped: it sits in a pocket that is 0.40 m wide on the way in, and twice the
     robot couldn't finish there because it had no room to turn round.
4. **Robot:**
   - battery charged;
   - remote switched to **CAN / command mode**;
   - e-stop button tested once: press, release, re-enable.
5. **Operator:**
   - holds the e-stop and the RC handset;
   - has the laptop with Foxglove (`ws://<orin-ip>:8765`), camera image with the safety banner, map, costmap;
   - walks behind the robot at a distance, never in its path.

## 3. Software setup

```bash
~/can_up.sh                                          # sudo; CAN at 500 kbit/s
~/robot_up.sh --nav lab2 2.08 0.33 1.449             # all four containers, navigation started at home2 (83°)
source ~/ros_utils/setup.bash
rostopic echo -n1 /localization/status | grep inlier # must be > 0.95; else 2D Pose Estimate in Foxglove
rosrun scout_nav eval_nav_goals.py $(rospack find scout_nav)/config/nav_eval_goals_lab2.yaml --dry-run
```

The dry run checks localization and safety, and asks the planner for every leg. It sends no goals and writes nothing.
- A leg that says `plan NONE` with `goal cell cost 100` is usually a stale costmap mark: the real run clears the
  costmaps first.
- `plan NONE` on a free goal needs a look before you start.

Optional recording (bags go to `~/bags/lab2/nav_runs/`), in a second terminal:
```bash
~/robot_exec.sh base rosrun scout_nav record_nav.sh lab2
```

## 4. The goals

Frame `map`; `home2` is the start mark. A lap is A → B → C → D → E → F → home2, about 40 m driven.

| # | name | x | y | yaw ° | clearance at the goal | leg to the next: straight / planned (2026-10-23) / narrowest clearance |
|---|---|---|---|---|---|---|
| 0 | home2 | 2.08 | 0.33 | 83 | 0.97 m (≥ 0.60 m within 0.3 m) | → A: 5.0 / 5.2 m / 0.47 m |
| 1 | A_aisle_mid | 2.65 | 5.32 | 68 | 0.75 m | → B: 5.4 / 6.6 m / 0.40 m |
| 2 | B_north | 4.70 | 10.31 | −179 | 1.05 m | → C: 1.1 / 1.1 m / 1.05 m |
| 3 | C_north_w | 3.59 | 10.30 | −87 | 1.10 m | → D: 7.5 / 9.0 m / **0.35 m** |
| 4 | D_east_mid | 3.93 | 2.83 | −79 | 0.54 m | → E: 7.1 / 10.1 m / 0.42 m |
| 5 | E_south | 5.31 | −4.13 | −173 | 0.60 m | → F: 1.0 / 1.2 m / 0.60 m |
| 6 | F_sw | 4.32 | −4.26 | 116 | 1.07 m | → home2: 5.1 / 6.6 m / 0.50 m |

- **Clearance** is the distance to the nearest obstacle on the static map. The robot needs 0.31 m: half its width,
  0.29 m, plus 0.02 m padding.
- **Narrowest** is the tightest point of the best route for that radius.
- **Each yaw** faces the next goal.
- `home2` also has room to turn in place: at least 0.60 m clearance anywhere within 0.3 m of it, so arriving a little
  off still leaves the 0.45 m the body's corners sweep.

Changing a goal changes what the CSV compares. Re-survey it, update this table and the YAML together, and give the goal
a new name rather than reusing the old one.

## 5. Running a session

```bash
rosrun scout_nav eval_nav_goals.py $(rospack find scout_nav)/config/nav_eval_goals_lab2.yaml \
    --operator <name> --notes "<anything unusual: furniture, people, battery>" [--laps 2] [--from D_east_mid]
```

**Before each goal:**
- The script waits until localization is ≥ 0.95 and safety isn't in FAULT.
- It prints the goal, the distance and the timeout, and **waits for Enter**. Look along the route first; the camera
  only sees forward.
- `s` + Enter skips the goal, and the row says SKIPPED. `q` + Enter ends the session.

**Costmap clearing:** right before sending, it clears the costmaps and waits 1 s, so the sensors re-mark what is really
there. Without that, stale marks from an earlier goal or from a mislocalized spell decide the result. On 2026-10-23, a
mark on goal A that no sensor could see blocked two legs. `--no-clear` turns it off, and the row records which.

**Timeout:** 30 s + 6 s per metre of straight distance. Then the goal is cancelled and logged as TIMEOUT.

**Stopping:**
1. **Ctrl+C** cancels the active goal and logs it as CANCELLED. move_base stops commanding, and the safety node drops
   `/nav/cmd_vel` after 0.5 s.
2. If the robot does anything unexpected, **press the e-stop first**, then Ctrl+C.
3. Afterwards: `~/robot_down.sh` (it closes a running recording first).

**Where the robot stops:** if a goal ends ABORTED or TIMEOUT, the robot stays where it stopped. Check the spot before
the next goal. A start inside the inflated band around a desk can't be planned from (see FAILURES F-55 and the
inscribed-start cases). Push or teleop it about 10 cm into the open, or skip to `home2`.

## 6. Reading the CSV

```python
import pandas as pd
d = pd.read_csv("~/maps/lab2/nav_eval/nav_eval.csv")
d = d[d.result != "SKIPPED"]
print(d.groupby(["git_rev", "goal_name"]).agg(n=("result", "size"), ok=("result", lambda r: (r == "SUCCEEDED").mean()),
      err_xy=("err_xy_m", "median"), t=("time_s", "median"), ratio=("path_ratio", "median"), rec=("recoveries_n", "sum")))
```

- **Compare sessions by `git_rev`** to see whether a software change helped.
- **`min_inlier` below about 0.9** in a row means localization trouble; that run's errors are suspect.
- **Non-zero `tf_errors_n`** usually means a lagging `map → odom` (hdl_localization) or a restarted container.

## 7. Safety: what is unsafe with people nearby

The items marked **UNSAFE WITH PEOPLE NEARBY** are why the aisles are closed off during a session.

1. **UNSAFE WITH PEOPLE NEARBY: the safety node only looks forward.** `scout_safety` slows and stops for people seen by
   the D455 camera, about 87° wide, in front.
   - Behind and beside the robot nothing slows it down for a person. The lidar costmap treats people as obstacles to
     avoid, but it doesn't stop the robot for someone stepping in.
   - The robot doesn't reverse (DWA `min_vel_x: 0`), but it **turns in place** at up to 0.5 rad/s.
   - Before 2026-10-23 the footprint was centred on the lidar, 0.18 m ahead of the turning centre, so the rear
     0.18 m of the body wasn't collision-checked during turns. It's centred correctly now; rows recorded before
     then (`git_rev` before that change) were driven with the old geometry.
   - Its 0.62 × 0.58 m body sweeps a circle of about 0.42 m radius, and the corners swing out to the sides and rear,
     where the camera is blind.
2. **UNSAFE WITH PEOPLE NEARBY: large turns in place on arrival.**
   - Each goal's heading faces the next goal, so the robot turns when it arrives.
   - Using the straight-line approach headings (the real route curves):

     | arriving at | turn |
     |---|---|
     | B | about 113° |
     | C | about 92° |
     | E | about 94° |
     | F | about 71° |
     | home2 | about 33° |
     | A | about 15° |
     | D | about 8° |
   - At B, C and E it turns close to desks. Stand clear of the robot's sides while it turns.
3. **UNSAFE WITH PEOPLE NEARBY: tight legs.**
   - On C → D the best route has 0.35 m clearance against the 0.31 m the robot needs, so the body passes within about
     4 cm of desk edges.
   - Keep everyone out of that aisle, including the operator: there's no room to step aside.
4. **A crashed node doesn't stop the robot instantly.**
   - If the safety node or the base container dies, the Scout firmware stops the wheels by itself 0.60 s after the
     last command. Measured from 0.1 m/s: 0.81–0.84 s to a standstill.
   - At the 0.4 m/s DWA maximum, that is **up to about 0.3 m of motion with nothing in control**.
   - Leave at least that much margin in front of anyone, beyond the safety node's own stop distance.
5. **UNSAFE WITH PEOPLE NEARBY: automatic resume after a container restart.**
   - If the safety container crashes and restarts during a goal, move_base keeps sending commands, and the robot
     **moves again on its own** once the safety node is back (about 8 s).
   - If the base container restarts, the goal is lost and the robot stays stopped.
   - Either way the session must be treated as interrupted: Ctrl+C and check before continuing.
6. **UNSAFE WITH PEOPLE NEARBY: mislocalization.**
   - With a wrong pose (for example a look-alike desk row) the robot drives "correctly" in the wrong place, into desks.
   - The script won't start a goal below 0.95 inliers, but localization can still slip during a goal. Watch the
     `min_inlier` column.
   - Watch Foxglove: if the scan no longer lines up with the map, e-stop.
7. **UNSAFE WITH PEOPLE NEARBY: something right next to the robot is invisible.**
   - The camera cloud starts at 0.5 m from the camera (0.68 m ahead of the turning centre). Closer than that, the D455
     reads garbage: on 2026-10-24 it reported a fixed patch at 0.33 m, 0.6-0.7 m above the floor, in most frames with
     nothing there (the person in view was ~6 m away). Those phantom points sat on the footprint and blocked every
     motion at home2, which is why the camera now drops everything under 0.5 m.
   - At that range the lidar is blind below about 0.7 m too, so neither sensor marks a person, leg or desk edge right
     at the front, and the detector won't see a person that close either: the safety node stays `OK`.
   - Step at least 1 m away before a goal starts. `run_circuit.py` waits 10 s before its first goal for this.
8. **Invisible obstacles.**
   - The costmap ignores anything lower than 8 cm (lidar) or 10 cm (camera), and glass. Cables, bags, a foot at floor
     level, or a pet won't be seen.
   - The camera's person detection covers people standing in front, not someone sitting or lying on the floor.
9. **The RC handset outranks the computer.** In RC mode the base ignores `/cmd_vel`, so the safety node can't stop the
   robot. Use the handset or the e-stop, not the laptop.
10. **Not unsafe, but it makes results noisy:**
   - stale or unmapped obstacles can block a route mid-goal (aborts, not collisions);
   - people moving through the camera view cause SLOW/STOP (see the `safety_*` columns).
