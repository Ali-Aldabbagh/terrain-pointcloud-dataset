#!/usr/bin/env python3
"""Quick 3D view of one extracted point cloud (.npy from extract_bag.py).

    pip install numpy matplotlib
    python view_cloud.py out/camera__depth__color__points/1617896036665215969.npy
"""
import sys

import matplotlib.pyplot as plt
import numpy as np


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    pts = np.load(sys.argv[1])
    if len(pts) > 30000:  # subsample for a responsive plot
        pts = pts[np.random.default_rng(0).choice(len(pts), 30000, replace=False)]
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    ax = plt.figure(figsize=(8, 6)).add_subplot(projection="3d")
    # RealSense optical frame: x right, y down, z forward -> plot z forward, -y up
    ax.scatter(x, z, -y, c=-y, s=1, cmap="viridis")
    ax.set_xlabel("x (m, right)")
    ax.set_ylabel("z (m, forward)")
    ax.set_zlabel("-y (m, up)")
    ax.set_title(f"{len(pts)} points")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
