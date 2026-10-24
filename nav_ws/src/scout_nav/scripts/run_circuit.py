#!/usr/bin/env python3
"""Drive eval_nav_goals.py without a person at the keyboard, stopping at the first goal that doesn't succeed.

    rosrun scout_nav run_circuit.py <goals.yaml> [eval_nav_goals.py options ...]

Answers each "Route clear?" prompt with Enter while every goal so far has SUCCEEDED, and with q after the first one
that didn't (so the robot isn't sent on from a spot it may be stuck in). Only for runs the operator has explicitly
started from the robot's side, e-stop in hand: it replaces the per-goal Enter, not the operator.
"""
import os
import subprocess
import sys

PROMPT = b"q = quit: "


def main():
    script = os.path.join(os.path.dirname(os.path.realpath(__file__)), "eval_nav_goals.py")
    p = subprocess.Popen([sys.executable, "-u", script] + sys.argv[1:], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
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
