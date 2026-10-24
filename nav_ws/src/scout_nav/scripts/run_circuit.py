#!/usr/bin/env python3
"""Drive eval_nav_goals.py without a person at the keyboard, stopping at the first goal that doesn't succeed.

    rosrun scout_nav run_circuit.py <goals.yaml> [--start-delay 10] [eval_nav_goals.py options ...]

Answers each "Route clear?" prompt with Enter while every goal so far has SUCCEEDED, and with q after the first one
that didn't (so the robot isn't sent on from a spot it may be stuck in). Waits --start-delay s (default 10) before
the first goal so whoever placed the robot can step back: within 0.5 m of the camera neither the camera cloud nor
the lidar sees anything, and the detector won't flag a person (nav_eval_protocol.md, item 7). Only for runs the operator has explicitly
started from the robot's side, e-stop in hand: it replaces the per-goal Enter, not the operator.
"""
import os
import subprocess
import sys
import time

PROMPT = b"q = quit: "


def main():
    args, delay = sys.argv[1:], 10.0
    if "--start-delay" in args:
        i = args.index("--start-delay")
        delay = float(args[i + 1])
        del args[i:i + 2]
    for left in range(int(delay), 0, -1):
        print("\rfirst goal in %2d s: step at least 1 m away from the robot " % left, end="", flush=True)
        time.sleep(1.0)
    print()
    script = os.path.join(os.path.dirname(os.path.realpath(__file__)), "eval_nav_goals.py")
    p = subprocess.Popen([sys.executable, "-u", script] + args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    buf, failed = b"", None
    while True:
        ch = p.stdout.read(1)
        if not ch:
            break
        sys.stdout.buffer.write(ch)
        sys.stdout.buffer.flush()
        buf += ch
        if ch == b"\n":
            line = buf.decode(errors="replace").strip()
            for r in ("ABORTED", "TIMEOUT", "CANCELLED", "REJECTED", "PREEMPTED"):
                if line.startswith(r + " after"):
                    failed = r
            buf = b""
        elif buf.endswith(PROMPT):
            answer = b"q\n" if failed else b"\n"
            sys.stdout.buffer.write(answer if failed else b"(go)\n")
            p.stdin.write(answer)
            p.stdin.flush()
            buf = b""
    p.wait()
    if failed:
        print("stopped after a %s goal" % failed)
    sys.exit(p.returncode)


if __name__ == "__main__":
    main()
