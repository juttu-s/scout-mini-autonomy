#!/usr/bin/env python3
"""Run SLAM methods one after another on the same recorded session and measure them.

    rosrun slam_eval run_slam_eval.py ~/maps/<name> [--methods dlio fast_lio ...] [--out DIR]
                                      [--repeats N] [--duration S] [--rate R]

Needs <session>/eval/derived.bag (prepare_bag.py). Each method gets, in isolation: a fresh roscore on its own port
(sim time), its launch from config/methods.yaml, eval_probe.py, and the bag played at --rate (default 1.0: real time,
as on the robot). While it runs, the processes of its ROS nodes are sampled every 0.5 s (/proc: RSS, CPU time).
Results go to <out>/<method>[_rN]/: <method>_online.tum, _final.tum (loop-closing methods), _latency.csv,
metrics.json, launch.log; <out>/meta.json has the setup. Compare with compare_slam_eval.py <out>.
Refuses to run next to a live robot stack (CPU contention would distort the timing).
"""
import argparse
import datetime
import glob
import json
import os
import signal
import subprocess
import time
import xmlrpc.client

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
PORT = 11411
MASTER = "http://localhost:%d" % PORT
LIVE_STACK = ["scout_base_node", "dlio_odom_node", "move_base", "hdl_localization"]
TICK = os.sysconf("SC_CLK_TCK")
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024


def session_bags(d):
    return sorted(glob.glob(os.path.join(d, "recording", "*.bag")), key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))


def ros_env():
    e = dict(os.environ, ROS_MASTER_URI=MASTER)
    e.pop("ROS_HOSTNAME", None)
    e["ROS_IP"] = "127.0.0.1"
    return e


def live_stack():
    found = []
    for name in LIVE_STACK:
        r = subprocess.run(["pgrep", "-x", name[:15]], stdout=subprocess.PIPE, text=True)
        if r.stdout.strip():
            found.append(name)
    return found


def git_rev(path):
    r = subprocess.run(["git", "-C", path, "rev-parse", "--short", "HEAD"], stdout=subprocess.PIPE,
                       stderr=subprocess.DEVNULL, text=True)
    dirty = subprocess.run(["git", "-C", path, "status", "--porcelain"], stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, text=True).stdout.strip()
    return (r.stdout.strip() + ("+patched" if dirty else "")) or "n/a"


def pkg_path(pkg, env):
    return subprocess.run(["rospack", "find", pkg], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                          env=env).stdout.strip()


def proc_sample(pid):
    """(rss_kb, hwm_kb, cpu_s) for a live pid, or None."""
    try:
        with open("/proc/%d/status" % pid) as f:
            st = dict(l.split(":", 1) for l in f if ":" in l)
        with open("/proc/%d/stat" % pid) as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return int(st["VmRSS"].split()[0]), int(st["VmHWM"].split()[0]), (int(fields[11]) + int(fields[12])) / TICK
    except (OSError, KeyError, IndexError, ValueError):
        return None


def node_pid(master, node):
    try:
        code, _, uri = master.lookupNode("/slam_eval", node)
        if code != 1:
            return None
        code, _, pid = xmlrpc.client.ServerProxy(uri).getPid("/slam_eval")
        return pid if code == 1 else None
    except (OSError, xmlrpc.client.Error):
        return None


def wait_master(proc, timeout=20):
    m = xmlrpc.client.ServerProxy(MASTER)
    end = time.time() + timeout
    while time.time() < end:
        if proc.poll() is not None:
            raise RuntimeError("roscore exited")
        try:
            m.getSystemState("/slam_eval")
            return m
        except OSError:
            time.sleep(0.3)
    raise RuntimeError("roscore did not come up")


def stop(proc, name, timeout=20):
    """SIGINT to the process group, escalating to SIGTERM/SIGKILL."""
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            print("    %s did not stop on %s" % (name, signal.Signals(sig).name))


def run_method(name, cfg, bags, out, a, env):
    os.makedirs(out, exist_ok=True)
    log = open(os.path.join(out, "launch.log"), "w")
    spawn = lambda cmd: subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    core = spawn(["roscore", "-p", str(PORT)])
    launch = probe = play = None
    status = "ok"
    samples, hwm, cpu, pids = [], {}, {}, {}
    try:
        master = wait_master(core)
        subprocess.run(["rosparam", "set", "/use_sim_time", "true"], env=env, check=True)
        launch = spawn(["roslaunch", "--wait", "slam_eval", cfg["launch"], "out:=" + out])
        pcmd = ["rosrun", "slam_eval", "eval_probe.py", "--input", cfg["input"], "--output", cfg["output"],
                "--odom", cfg["odom"], "--out", out, "--name", name, "--stamp-offset", str(cfg.get("stamp_offset", 0)),
                "--body-to-lidar"] + [str(v) for v in cfg.get("body_to_lidar", [0, 0, 0])]
        if "path" in cfg.get("final", {}):
            pcmd += ["--final-path", cfg["final"]["path"]]
        probe = spawn(pcmd)
        # all nodes up before the bag starts
        end = time.time() + 60
        while time.time() < end and not all(node_pid(master, n) for n in cfg["nodes"] + ["/eval_probe"]):
            if launch.poll() is not None:
                raise RuntimeError("launch exited during start-up")
            time.sleep(0.5)
        pids = {n: node_pid(master, n) for n in cfg["nodes"]}
        missing = [n for n, p in pids.items() if not p]
        if missing:
            raise RuntimeError("nodes not up after 60 s: %s" % missing)
        time.sleep(2.0)
        pcmd = ["rosbag", "play", "--clock", "-q", "-r", str(a.rate), "-d", "1", "--topics"] + cfg["play"] + ["--bags"] + bags
        if a.duration:
            pcmd += ["-u", str(a.duration)]
        print("    playing (%s)" % ("%.0f s" % a.duration if a.duration else "whole bag"), flush=True)
        t0 = time.time()
        play = spawn(pcmd)
        cpu0 = {n: (proc_sample(p) or (0, 0, 0))[2] for n, p in pids.items()}
        drain_until = None
        while True:
            time.sleep(0.5)
            now = [proc_sample(p) for p in pids.values()]
            alive = [s for s in now if s]
            samples.append((time.time() - t0, sum(s[0] for s in alive), len(alive)))
            for n, s in zip(pids, now):
                if s:
                    hwm[n], cpu[n] = s[1], s[2]
            if len(alive) < len(pids) and status == "ok":
                dead = [n for n, s in zip(pids, now) if not s]
                status = "crashed: %s at %.0f s" % (",".join(dead), time.time() - t0)
                print("    " + status, flush=True)
            if play.poll() is not None and drain_until is None:
                drain_until = time.time() + a.drain
            if drain_until and time.time() > drain_until:
                break
        t_play = time.time() - t0
        if "rtabmap" in cfg.get("final", {}):
            try:
                subprocess.run(["rosrun", "slam_eval", "rtabmap_export.py", cfg["final"]["rtabmap"],
                                os.path.join(out, name + "_final.tum")], stdout=log, stderr=subprocess.STDOUT, env=env,
                               timeout=120)
            except subprocess.TimeoutExpired:
                print("    rtabmap export timed out: no final trajectory", flush=True)
    except RuntimeError as e:
        status, t_play = "failed: %s" % e, 0.0
        print("    " + status, flush=True)
        cpu0 = {}
    finally:
        stop(play, "rosbag play")
        stop(probe, "probe")          # writes the TUM/latency files on shutdown
        stop(launch, "launch", timeout=30)
        stop(core, "roscore")
        log.close()

    m = {"method": name, "status": status, "rate": a.rate, "play_wall_s": round(t_play, 2)}
    pj = os.path.join(out, name + "_probe.json")
    if os.path.exists(pj):
        m.update(json.load(open(pj)))
    lat_f = os.path.join(out, name + "_latency.csv")
    if os.path.exists(lat_f):
        lat = np.loadtxt(lat_f, delimiter=",", skiprows=1, ndmin=2)
        if len(lat):
            l = lat[:, 4] * 1000
            m.update(latency_ms={k: round(float(v), 2) for k, v in zip(
                ["mean", "p50", "p95", "p99", "max"], [l.mean(), *np.percentile(l, [50, 95, 99]), l.max()])})
            m["processing_wall_s"] = round(float(lat[:, 3].max() - lat[:, 2].min()), 2)  # first input -> last output
    if m.get("inputs"):
        m["dropped_pct"] = round(100.0 * (1 - m.get("matched", 0) / m["inputs"]), 2)
        if m.get("inputs_after_init"):  # without the scans a method uses to initialise (DLIO: 3 s IMU calibration)
            m["dropped_after_init_pct"] = round(100.0 * (1 - m["matched"] / m["inputs_after_init"]), 2)
    if cpu:
        cpu_s = sum(cpu[n] - cpu0.get(n, 0) for n in cpu)
        m["cpu_s"] = round(cpu_s, 2)
        m["mean_cores"] = round(cpu_s / max(t_play, 1e-6), 3)
        m["cpu_s_per_node"] = {n: round(cpu[n] - cpu0.get(n, 0), 2) for n in cpu}
    if samples:
        m["peak_rss_mb"] = round(max(s[1] for s in samples) / 1024.0, 1)
        m["peak_rss_mb_per_node"] = {n: round(v / 1024.0, 1) for n, v in hwm.items()}
        np.savetxt(os.path.join(out, "rss.csv"), np.array(samples), delimiter=",", header="t_s,rss_kb,alive", comments="")
    with open(os.path.join(out, "metrics.json"), "w") as f:
        json.dump(m, f, indent=1)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("--methods", nargs="+", default=["dlio", "fast_lio", "lio_sam", "rtabmap_icp", "rtabmap_rgbd"])
    ap.add_argument("--out")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--duration", type=float, default=0.0, help="play only the first S s of the bag (smoke test)")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--drain", type=float, default=5.0, help="s to keep running after the bag ends")
    a = ap.parse_args()
    d = os.path.expanduser(a.session_dir).rstrip("/")
    reg = yaml.safe_load(open(os.path.join(PKG, "config", "methods.yaml")))
    unknown = [m for m in a.methods if m not in reg]
    if unknown:
        raise SystemExit("unknown methods %s (config/methods.yaml has %s)" % (unknown, list(reg)))
    derived = os.path.join(d, "eval", "derived.bag")
    if not os.path.exists(derived):
        raise SystemExit("%s missing: run  rosrun slam_eval prepare_bag.py %s  first" % (derived, d))
    busy = live_stack()
    if busy:
        raise SystemExit("robot stack running (%s): stop it first, it would distort the timing" % ", ".join(busy))
    bags = session_bags(d) + [derived]
    out = os.path.expanduser(a.out or os.path.join("~/slam_eval", os.path.basename(d),
                                                   datetime.datetime.now().strftime("%Y%m%d_%H%M%S")))
    os.makedirs(out, exist_ok=True)
    env = ros_env()
    nvp = subprocess.run(["nvpmodel", "-q"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True).stdout.strip() \
        if subprocess.run(["which", "nvpmodel"], stdout=subprocess.DEVNULL).returncode == 0 else "n/a"
    revs = {p: git_rev(pkg_path(p, env)) for p in ["direct_lidar_inertial_odometry", "fast_lio", "lio_sam"]}
    meta = {"session": d, "bags": bags, "methods": a.methods, "repeats": a.repeats, "rate": a.rate,
            "duration": a.duration or None, "started": datetime.datetime.now().isoformat(timespec="seconds"),
            "nvpmodel": nvp, "revisions": revs, "registry": {m: reg[m] for m in a.methods}}
    json.dump(meta, open(os.path.join(out, "meta.json"), "w"), indent=1)
    print("results -> %s" % out)
    for rep in range(a.repeats):
        for name in a.methods:
            sub = name if a.repeats == 1 else "%s_r%d" % (name, rep + 1)
            print("[%s] %s" % (time.strftime("%H:%M:%S"), sub), flush=True)
            m = run_method(name, reg[name], bags, os.path.join(out, sub), a, env)
            lat = m.get("latency_ms", {})
            print("    %s | %s outputs, dropped %s %% (after init %s %%) | latency p50 %s / p95 %s ms | CPU %s s (%.2f cores) | peak RSS %s MB"
                  % (m["status"], m.get("matched", 0), m.get("dropped_pct", "-"), m.get("dropped_after_init_pct", "-"), lat.get("p50", "-"), lat.get("p95", "-"),
                     m.get("cpu_s", "-"), m.get("mean_cores", 0), m.get("peak_rss_mb", "-")), flush=True)
    print("done: rosrun slam_eval compare_slam_eval.py %s" % out)


if __name__ == "__main__":
    main()
