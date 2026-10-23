#!/usr/bin/env python3
"""Write RTAB-Map's optimised graph as TUM (run_slam_eval.py calls it when the bag has ended, before shutdown).

    rosrun slam_eval rtabmap_export.py <rtabmap namespace> <out.tum>

/rtabmap/mapPath gives every pose the path's stamp, so the poses come from get_map_data instead (graph only):
graph.posesId/poses are the optimised poses, nodes[].stamp the time of each node.
"""
import sys

import rospy
from rtabmap_msgs.srv import GetMap


def main():
    ns, out = sys.argv[1].rstrip("/"), sys.argv[2]
    rospy.init_node("rtabmap_export", anonymous=True)
    srv = rospy.ServiceProxy(ns + "/get_map_data", GetMap)
    srv.wait_for_service(10)
    d = srv(True, True, True).data  # global, optimized, graphOnly (positional: "global" is a Python keyword)
    stamp = {n.id: n.stamp for n in d.nodes}
    rows = sorted((stamp[i], p) for i, p in zip(d.graph.posesId, d.graph.poses) if stamp.get(i, 0) > 0)
    with open(out, "w") as f:
        for t, p in rows:
            f.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (t, p.position.x, p.position.y, p.position.z,
                                                                   p.orientation.x, p.orientation.y, p.orientation.z,
                                                                   p.orientation.w))
    print("%s: %d optimised poses (%d nodes)" % (out, len(rows), len(d.nodes)))


if __name__ == "__main__":
    main()
