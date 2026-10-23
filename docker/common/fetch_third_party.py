#!/usr/bin/env python3
"""Clone third-party sources at their pinned commits and apply the local patches.

    fetch_third_party.py <pins_dir> <dest_dir> <name> [<name> ...]

Reads <pins_dir>/<name>.repos (URL + commit) and applies <pins_dir>/<name>.patch if the pin names one. Used inside
the Dockerfiles, so an image is built from upstream + patches only (never from files on the Orin). Fetches just
the pinned commit (shallow) and fails loudly if a commit or patch does not apply.
"""
import os
import subprocess
import sys

import yaml


def run(*cmd, cwd=None):
    subprocess.run(cmd, cwd=cwd, check=True)


def main():
    pins, dest, names = sys.argv[1], sys.argv[2], sys.argv[3:]
    for name in names:
        r = yaml.safe_load(open(os.path.join(pins, name + ".repos")))["repositories"][name]
        d = os.path.join(dest, name)
        os.makedirs(d, exist_ok=True)
        run("git", "init", "-q", cwd=d)
        run("git", "remote", "add", "origin", r["url"], cwd=d)
        run("git", "fetch", "-q", "--depth", "1", "origin", r["version"], cwd=d)
        run("git", "checkout", "-q", "FETCH_HEAD", cwd=d)
        if os.path.exists(os.path.join(d, ".gitmodules")):
            run("git", "submodule", "update", "-q", "--init", "--recursive", "--depth", "1", cwd=d)
        if r.get("patch"):
            run("git", "apply", "--whitespace=nowarn", os.path.join(os.path.abspath(pins), r["patch"]), cwd=d)
        print("%s @ %s%s" % (name, r["version"][:9], " + " + r["patch"] if r.get("patch") else ""), flush=True)


if __name__ == "__main__":
    main()
