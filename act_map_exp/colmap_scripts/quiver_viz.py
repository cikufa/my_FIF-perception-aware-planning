#!/usr/bin/env python3
"""Simple quiver reviewer for trajectory CSV exports.

Each CSV row must look like: x,y,z,dx,dy,dz.
The script can compare two files side-by-side or plot just one.
Usage:
  python3 tools/quiver_viz.py --before init_quiver.csv --after opt_quiver.csv
  python3 tools/quiver_viz.py --before init_quiver.csv --save out.png
  python3 tools/quiver_viz.py --plot-before --no-show
"""

import argparse
import csv
from typing import List, Tuple

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_quiver(path: str) -> List[Tuple[float, float, float, float, float, float]]:
    rows = []
    with open(path, newline="") as fp:
        reader = csv.reader(fp)
        for line in reader:
            if not line:
                continue
            if len(line) < 6:
                continue
            data = [float(val) for val in line[:6]]
            rows.append(tuple(data))
    return rows


def plot_quivers(ax, quivers, label, color, length_factor):
    if not quivers:
        return
    xs, ys, zs, dxs, dys, dzs = zip(*quivers)
    ax.quiver(
        xs,
        ys,
        zs,
        dxs,
        dys,
        dzs,
        length=length_factor,
        normalize=True,
        color=color,
        label=label,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot trajectory quivers from txts.")
    parser.add_argument("--before", help="CSV before optimization", default=None)
    parser.add_argument("--after", help="CSV after optimization", default=None)
    parser.add_argument( 
        "--save", help="Save figure instead of showing it", default=None
    )
    parser.add_argument(
        "--length", help="Arrow length multiplier", type=float, default=1.0
    )
    args = parser.parse_args()

    if not args.before and not args.after:
        parser.error("At least one of --before or --after must be provided.")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect([1, 1, 1])

    if args.before:
        quivers = load_quiver(args.before)
        plot_quivers(ax, quivers, "before", "tab:blue", args.length)
    if args.after:
        quivers = load_quiver(args.after)
        plot_quivers(ax, quivers, "after", "tab:orange", args.length)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    ax.set_title("Camera quivers")

    if args.save:
        plt.tight_layout()
        plt.savefig(args.save)
        print(f"Saved quiver comparison to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
