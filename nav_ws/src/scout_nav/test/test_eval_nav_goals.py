"""Unit tests for eval_nav_goals.py helpers (CSV handling, angles). Run: pytest nav_ws/src/scout_nav/test

ROS modules that aren't installed (CI runs this without a ROS workspace) are stubbed; the tested helpers don't use
them.
"""
import csv
import importlib
import math
import os
import sys
import types
from unittest import mock

import pytest

ROS_MODULES = ["actionlib", "rospy", "tf2_geometry_msgs", "tf2_ros", "actionlib_msgs", "actionlib_msgs.msg",
               "geometry_msgs", "geometry_msgs.msg", "hdl_localization", "hdl_localization.msg", "move_base_msgs",
               "move_base_msgs.msg", "nav_msgs", "nav_msgs.msg", "nav_msgs.srv", "std_srvs", "std_srvs.srv",
               "rosgraph_msgs", "rosgraph_msgs.msg", "std_msgs", "std_msgs.msg"]


@pytest.fixture(scope="module")
def eng():
    for name in ROS_MODULES:
        try:
            importlib.import_module(name)
        except ImportError:
            sys.modules[name] = mock.MagicMock(name=name)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
    return importlib.import_module("eval_nav_goals")


def rows(path):
    with open(path, newline="") as f:
        return list(csv.reader(f))


def test_open_csv_new_file_gets_header(eng, tmp_path):
    path = str(tmp_path / "sub" / "nav_eval.csv")  # directory is created too
    f, w = eng.open_csv(path)
    w.writerow({"session_id": "s1", "goal_name": "A"})
    f.close()
    r = rows(path)
    assert r[0] == eng.COLUMNS
    assert len(r) == 2 and r[1][eng.COLUMNS.index("goal_name")] == "A"


def test_open_csv_appends_without_second_header(eng, tmp_path):
    path = str(tmp_path / "nav_eval.csv")
    for sid in ("s1", "s2"):
        f, w = eng.open_csv(path)
        w.writerow({"session_id": sid})
        f.close()
    r = rows(path)
    assert r[0] == eng.COLUMNS
    assert [x[0] for x in r[1:]] == ["s1", "s2"]


def test_open_csv_refuses_other_columns(eng, tmp_path):
    path = tmp_path / "nav_eval.csv"
    path.write_text("session_id,something_else\nx,y\n")
    with pytest.raises(SystemExit, match="different columns"):
        eng.open_csv(str(path))
    assert path.read_text() == "session_id,something_else\nx,y\n"  # untouched


def test_open_csv_empty_file_gets_header(eng, tmp_path):
    path = tmp_path / "nav_eval.csv"
    path.write_text("")
    f, _ = eng.open_csv(str(path))
    f.close()
    assert rows(str(path))[0] == eng.COLUMNS


@pytest.mark.parametrize("a, want", [(0.0, 0.0), (math.pi / 2, math.pi / 2), (3 * math.pi / 2, -math.pi / 2),
                                     (-3 * math.pi / 2, math.pi / 2), (4 * math.pi + 0.1, 0.1)])
def test_wrap(eng, a, want):
    assert eng.wrap(a) == pytest.approx(want)


@pytest.mark.parametrize("deg", [0, 45, 90, 135, 179, -179, -90, -30])
def test_yaw_of_planar_quaternion(eng, deg):
    h = math.radians(deg) / 2
    q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))
    assert eng.yaw_of(q) == pytest.approx(math.radians(deg))
