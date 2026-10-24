#!/usr/bin/env python3
"""Project a 3D DLIO map (.pcd) to a 2D occupancy grid for move_base, for flat indoor floors.

    pcd_to_grid.py <map_dir> [--robot-height 0.5] [--resolution 0.05]

Reads <map_dir>/dlio_map.pcd (map frame = DLIO's odom frame of the mapping run; z = 0 at lidar height) and writes
  <map_dir>/grid.pgm + grid.yaml       map_server map: occupied = black, free = white, unknown = grey
  <map_dir>/costmap_heights.yaml       floor-relative height limits for the move_base voxel layers
  <map_dir>/grid_info.yaml             detected floor height and point counts

Floor: z = -lidar_height (measured; recommended), else the densest point layer below the lidar.
Obstacles: points from floor + min_height up to floor + robot_height + margin (furniture tops and shelves at robot
height count, the floor and ceiling don't).
Free: cells with nothing in the obstacle band whose column was observed (floor, or ceiling/upper walls above it).
The MID360 looks at most 7 deg down, so from ~0.7 m height it sees the floor only ~6 m away; indoors the ceiling is
what shows that a cell was observed. Everything else stays unknown.
"""
import argparse
import os
import sys

import numpy as np

TYPES = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
         ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8"}


def read_pcd_xyz(path):
    """x, y, z (N x 3 float32) from a binary or ascii PCD file."""
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            key, *vals = line.split()
            header[key.upper()] = vals
            if key.upper() == "DATA":
                break
        fields, sizes, types = header["FIELDS"], [int(s) for s in header["SIZE"]], header["TYPE"]
        counts = [int(c) for c in header.get("COUNT", ["1"] * len(fields))]
        n = int(header["POINTS"][0])
        dtype = np.dtype([(fld if fld != "_" else "_pad%d" % i, TYPES[(t, s)], (c,) if c > 1 else ())
                          for i, (fld, s, t, c) in enumerate(zip(fields, sizes, types, counts))])
        if header["DATA"][0] == "binary":
            data = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        elif header["DATA"][0] == "ascii":
            data = np.loadtxt(f, dtype=dtype, max_rows=n)
        else:
            sys.exit("compressed PCD not supported; save from DLIO (binary)")
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def detect_floor(z, below=-0.15):
    """z of the densest 2 cm layer below the lidar (the floor is the largest horizontal surface down there)."""
    zl = z[z < below]
    if zl.size < 100:
        sys.exit("too few points below the lidar to find the floor")
    hist, edges = np.histogram(zl, bins=np.arange(zl.min(), below + 0.02, 0.02))
    i = int(np.argmax(hist))
    return float((edges[i] + edges[i + 1]) / 2.0), int(hist[i])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_dir")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--robot-height", type=float, default=0.5, help="m above the floor the robot (incl. mast) occupies")
    ap.add_argument("--margin", type=float, default=0.1, help="extra height above the robot counted as obstacle")
    ap.add_argument("--min-height", type=float, default=0.08, help="m above the floor where obstacles start")
    ap.add_argument("--floor-tol", type=float, default=0.05, help="+- m around the floor counted as floor")
    ap.add_argument("--seen-radius", type=float, default=0.3,
                    help="a column counts as observed if a floor/ceiling point was seen within this distance (m)")
    ap.add_argument("--min-points", type=int, default=2, help="points needed to mark a cell occupied (noise filter)")
    ap.add_argument("--lidar-height", type=float, default=None,
                    help="measured height of the MID360 above the floor (m); floor is then z = -lidar_height")
    a = ap.parse_args()

    xyz = read_pcd_xyz(os.path.join(a.map_dir, "dlio_map.pcd"))
    if a.lidar_height is not None:
        floor_z, floor_n = -a.lidar_height, -1
    else:
        floor_z, floor_n = detect_floor(xyz[:, 2])
        if floor_n < 500:
            print("WARNING: floor detected from only %d points (z = %.3f). Measure the lidar height and pass "
                  "--lidar-height for a reliable grid." % (floor_n, floor_z))
    lo, hi = floor_z + a.min_height, floor_z + a.robot_height + a.margin
    obst = xyz[(xyz[:, 2] > lo) & (xyz[:, 2] < hi)]
    floor = xyz[np.abs(xyz[:, 2] - floor_z) < a.floor_tol]
    seen = xyz[(np.abs(xyz[:, 2] - floor_z) < a.floor_tol) | (xyz[:, 2] >= hi)]  # floor, or anything above the band

    res = a.resolution
    used = np.vstack([obst[:, :2], seen[:, :2]]) if len(obst) else seen[:, :2]
    xy_min, xy_max = used.min(0) - 1.0, used.max(0) + 1.0
    w, h = (np.ceil((xy_max - xy_min) / res)).astype(int)

    def counts(pts):
        ij = np.floor((pts[:, :2] - xy_min) / res).astype(int)
        c = np.zeros((h, w), np.int32)
        np.add.at(c, (ij[:, 1], ij[:, 0]), 1)
        return c

    occ = counts(obst) >= a.min_points
    # Lidar coverage is sparse (non-repetitive scan pattern, 5 cm voxels): spread each observation over seen_radius
    import cv2
    r = max(1, int(round(a.seen_radius / res)))
    observed = cv2.dilate((counts(seen) > 0).astype(np.uint8),
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))) > 0
    free = observed & ~occ

    img = np.full((h, w), 205, np.uint8)  # unknown
    img[free] = 254
    img[occ] = 0
    img = np.flipud(img)  # image row 0 is the top (max y)
    with open(os.path.join(a.map_dir, "grid.pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())
    with open(os.path.join(a.map_dir, "grid.yaml"), "w") as f:
        f.write("image: grid.pgm\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n" % (res, xy_min[0], xy_min[1]))

    # Height limits for the move_base voxel obstacle layers (global frames map/odom share DLIO's z = lidar height)
    # move_base works in floor-level frames (map_floor / odom_floor, 0.76 m below map / odom; see navigation.launch)
    # because VoxelLayer's origin_z can't be negative. So heights here are relative to the floor. The voxel grid
    # must reach above the sensor (the lidar) to raytrace from it, with at most 16 voxels per column.
    lidar_rel = -floor_z
    top = max(hi - floor_z, lidar_rel) + 0.1
    z_res = float(np.ceil(top / 16.0 * 100.0) / 100.0)
    band = (a.min_height, a.robot_height + a.margin)
    layer = ("    origin_z: 0.0\n    z_resolution: %.2f\n    z_voxels: 16\n"
             "    min_obstacle_height: %.3f\n    max_obstacle_height: %.3f\n"
             "    lidar: {min_obstacle_height: %.3f, max_obstacle_height: %.3f}\n"
             "    camera: {min_obstacle_height: %.3f, max_obstacle_height: %.3f}\n"
             % ((z_res,) + band + band + (max(band[0], 0.15), band[1])))  # camera: margin for floor depth noise + rocking (0.10 marked floor, 2026-10-23)
    with open(os.path.join(a.map_dir, "costmap_heights.yaml"), "w") as f:
        f.write("# generated by pcd_to_grid.py: heights relative to the floor (costmaps use map_floor / odom_floor);\n"
                "# floor at z = %.3f in the map frame, lidar %.2f m above it\n" % (floor_z, lidar_rel))
        for cm in ("global_costmap", "local_costmap"):
            f.write("%s:\n  obstacle_layer:\n%s" % (cm, layer))
    with open(os.path.join(a.map_dir, "grid_info.yaml"), "w") as f:
        f.write("floor_z: %.3f   # map frame; lidar height above floor = %.3f m\nobstacle_band: [%.3f, %.3f]\n"
                "points: %d\nobstacle_points: %d\nfloor_points: %d\ngrid: [%d, %d]\n"
                % (floor_z, -floor_z, lo, hi, len(xyz), len(obst), len(floor), w, h))

    print("floor at z = %.3f (lidar %.2f m above the floor%s)" % (
        floor_z, -floor_z, ", given" if floor_n < 0 else ", detected from %d pts" % floor_n))
    print("grid %dx%d @ %.2f m: %d occupied, %d free, %d unknown cells -> %s"
          % (w, h, res, occ.sum(), free.sum(), w * h - occ.sum() - free.sum(), os.path.join(a.map_dir, "grid.yaml")))


if __name__ == "__main__":
    main()
