#!/usr/bin/env python3
"""Extract point clouds, IMU and images from a dataset recording (ROS 1 .bag).

Works on Windows, macOS and Linux *without* a ROS installation, using the
pure-Python `rosbags` library.

    pip install rosbags numpy
    python extract_bag.py recording.bag                 # list topics
    python extract_bag.py recording.bag -o out/         # extract everything
    python extract_bag.py recording.bag -o out/ --every 5 --max 200

Output
    out/<topic>/<timestamp_ns>.npy   point clouds, N x 3 (x, y, z) float32,
                                     or N x 4 with an rgb column if present
    out/<topic>/<timestamp_ns>.pcd   same clouds as ASCII PCD (--pcd)
    out/<topic>.csv                  IMU: t, orientation, angular velocity,
                                     linear acceleration
    out/<topic>/<timestamp_ns>.png   images (--images, needs opencv-python)
"""
import argparse
import csv
import os
import sys

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

# sensor_msgs/PointField datatype -> numpy dtype
_PF = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}

PC2 = "sensor_msgs/msg/PointCloud2"
IMU = "sensor_msgs/msg/Imu"
IMG = "sensor_msgs/msg/Image"


def cloud_to_array(msg):
    """PointCloud2 -> (N, 3) or (N, 4) float32 array, NaN points removed."""
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype in _PF and f.count == 1:
            names.append(f.name)
            formats.append(endian + _PF[f.datatype])
            offsets.append(f.offset)
    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    cols = [raw["x"], raw["y"], raw["z"]]
    if "rgb" in names:
        cols.append(raw["rgb"].astype(np.float32))
    pts = np.stack([c.astype(np.float32) for c in cols], axis=1)
    return pts[np.isfinite(pts[:, :3]).all(axis=1)]


def write_pcd(path, pts):
    has_rgb = pts.shape[1] == 4
    with open(path, "w") as fh:
        fh.write("# .PCD v0.7\nVERSION 0.7\n")
        fh.write("FIELDS x y z rgb\n" if has_rgb else "FIELDS x y z\n")
        fh.write("SIZE " + " ".join(["4"] * pts.shape[1]) + "\n")
        fh.write("TYPE " + " ".join(["F"] * pts.shape[1]) + "\n")
        fh.write("COUNT " + " ".join(["1"] * pts.shape[1]) + "\n")
        fh.write(f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        fh.write(f"POINTS {len(pts)}\nDATA ascii\n")
        np.savetxt(fh, pts, fmt="%.6f")


def safe(topic):
    return topic.strip("/").replace("/", "__") or "root"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", help="output folder (omit to only list topics)")
    ap.add_argument("--topics", nargs="*", help="only these topics")
    ap.add_argument("--every", type=int, default=1, help="keep every n-th cloud/image")
    ap.add_argument("--max", type=int, default=0, help="max clouds/images per topic")
    ap.add_argument("--pcd", action="store_true", help="also write .pcd files")
    ap.add_argument("--images", action="store_true", help="also extract images")
    args = ap.parse_args()

    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(args.bag) as reader:
        print(f"{args.bag}: {reader.duration / 1e9:.1f} s, {reader.message_count} messages")
        for c in reader.connections:
            print(f"  {c.topic:45s} {c.msgtype:32s} {c.msgcount:7d}")
        if not args.out:
            return

        wanted = {PC2, IMU} | ({IMG} if args.images else set())
        conns = [c for c in reader.connections if c.msgtype in wanted
                 and (not args.topics or c.topic in args.topics)]
        os.makedirs(args.out, exist_ok=True)
        seen, saved, imu_writers, files = {}, {}, {}, []

        for conn, t, raw in reader.messages(connections=conns):
            msg = store.deserialize_ros1(raw, conn.msgtype)
            name = safe(conn.topic)
            if conn.msgtype == IMU:
                if name not in imu_writers:
                    fh = open(os.path.join(args.out, name + ".csv"), "w", newline="")
                    files.append(fh)
                    w = csv.writer(fh)
                    w.writerow(["t_ns", "qx", "qy", "qz", "qw", "wx", "wy", "wz",
                                "ax", "ay", "az"])
                    imu_writers[name] = w
                o, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
                imu_writers[name].writerow([t, o.x, o.y, o.z, o.w, g.x, g.y, g.z,
                                            a.x, a.y, a.z])
                continue

            seen[name] = seen.get(name, 0) + 1
            if (seen[name] - 1) % args.every or (args.max and saved.get(name, 0) >= args.max):
                continue
            folder = os.path.join(args.out, name)
            os.makedirs(folder, exist_ok=True)
            if conn.msgtype == PC2:
                pts = cloud_to_array(msg)
                np.save(os.path.join(folder, f"{t}.npy"), pts)
                if args.pcd:
                    write_pcd(os.path.join(folder, f"{t}.pcd"), pts)
            else:
                import cv2
                buf = np.frombuffer(bytes(msg.data), np.uint8)
                if msg.encoding in ("rgb8", "bgr8"):
                    img = buf.reshape(msg.height, msg.width, 3)
                    if msg.encoding == "rgb8":
                        img = img[:, :, ::-1]
                elif msg.encoding in ("16UC1", "mono16"):
                    img = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.width)
                else:
                    img = buf.reshape(msg.height, msg.width, -1)
                cv2.imwrite(os.path.join(folder, f"{t}.png"), img)
            saved[name] = saved.get(name, 0) + 1

        for fh in files:
            fh.close()
        for name, n in saved.items():
            print(f"saved {n} frames from {name}")
        for name in imu_writers:
            print(f"saved IMU to {name}.csv")


if __name__ == "__main__":
    sys.exit(main())
