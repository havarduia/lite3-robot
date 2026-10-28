#!/usr/bin/env python3
"""Drop broken lidar frames from a bag before SLAM sees them.

When the Jetson's clock jumps mid-recording - it boots with a stale clock and
NTP corrects it a few minutes later - the Livox driver's 100 ms frame
buckets break and it publishes thousands of 96-point fragments in about a
second. Measured on walk_20260924_135139: 2481 fragments between two normal
scans, and KISS-SLAM registered every one of them as a scan, diverged to a
1.5 km map and never recovered.

    bag-clean ~/bags/walk_x     prints the bag to use: the original if it is
                                clean, else ~/bags/.clean/walk_x (same name,
                                so map output paths do not change)
"""
import os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

TOPIC = "/livox/lidar"
MIN_SHARE = 0.1     # a frame with <10% of a full frame's points is a fragment
# "Full" is the 95th percentile, not the median: in the bag above fragments
# were 77% of all messages, so the median frame WAS a fragment.


def reader(bag):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
    return rd


def main():
    bag = os.path.realpath(os.path.expanduser(sys.argv[1]))
    rd = reader(bag)
    rd.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    sizes = []
    while rd.has_next():
        _, data, _ = rd.read_next()
        m = deserialize_message(data, PointCloud2)
        sizes.append(m.width * m.height)
    sizes = np.array(sizes)
    full = np.percentile(sizes, 95)
    bad = sizes < MIN_SHARE * full
    if not bad.any():
        print(bag)
        return
    out = os.path.join(os.path.dirname(bag), ".clean", os.path.basename(bag))
    print(f"  dropping {bad.sum()} of {len(sizes)} lidar frames under "
          f"{int(MIN_SHARE * full)} points -> {out}", file=sys.stderr)
    if os.path.isdir(out):              # cleaned on an earlier run
        print(out)
        return
    os.makedirs(os.path.dirname(out), exist_ok=True)
    rd = reader(bag)
    wr = rosbag2_py.SequentialWriter()
    wr.open(rosbag2_py.StorageOptions(uri=out, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
    for t in rd.get_all_topics_and_types():
        wr.create_topic(t)
    i = 0
    while rd.has_next():
        topic, data, t = rd.read_next()
        if topic == TOPIC:
            i += 1
            if bad[i - 1]:
                continue
        wr.write(topic, data, t)
    del wr                              # closes the bag
    print(out)


if __name__ == "__main__":
    main()
