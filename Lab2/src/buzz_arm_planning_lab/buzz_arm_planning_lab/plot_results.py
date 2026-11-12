#!/usr/bin/env python3
"""Chart the output of plan_benchmark.

    ros2 run buzz_arm_planning_lab plot_results results.csv
    ros2 run buzz_arm_planning_lab plot_results results.csv --out charts.png --show

One column of panels per scene (the `label` column), three rows:
success rate, planning time per trial (log scale), path length per trial.
Only successful trials have a time and a length, so failed trials appear in
the success-rate row and nowhere else.
"""

import argparse
import csv
import statistics
from collections import OrderedDict

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# One fixed colour per planner, assigned by name so a planner keeps its colour
# whichever scenes or planners are in the file. Unknown planners take the
# remaining slots in order.
PLANNER_COLOURS = OrderedDict([
    ("RRTConnect", "#2a78d6"),
    ("RRT", "#eb6834"),
    ("RRTstar", "#1baf7a"),
    ("PRM", "#eda100"),
])
SPARE_COLOURS = ["#e87ba4", "#008300", "#7d5fd3", "#8c6d1f"]

INK = "#333333"
INK_MUTED = "#8a8a8a"
GRID = "#e4e4e4"


def read_rows(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["success"] = r["success"] == "1"
        for key in ("planning_time", "path_length", "duration"):
            r[key] = float(r[key]) if r[key] not in ("", None) else None
    return rows


def ordered_unique(values):
    return list(OrderedDict.fromkeys(values))


def colour_for(planner, assigned):
    if planner in PLANNER_COLOURS:
        return PLANNER_COLOURS[planner]
    if planner not in assigned:
        assigned[planner] = SPARE_COLOURS[len(assigned) % len(SPARE_COLOURS)]
    return assigned[planner]


def style_axis(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_MUTED)
    ax.tick_params(colors=INK, labelsize=9)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def strip(ax, x, values, colour):
    """Dots for every trial, spread sideways so equal values stay visible, plus a mean bar."""
    n = len(values)
    offsets = np.linspace(-0.18, 0.18, n) if n > 1 else np.zeros(1)
    ax.scatter(x + offsets, values, s=28, color=colour, edgecolor="white", linewidth=1, zorder=3)
    ax.hlines(statistics.mean(values), x - 0.3, x + 0.3, color=INK, linewidth=2, zorder=4)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", help="results.csv written by plan_benchmark --out")
    parser.add_argument("--out", help="PNG to write (default: <csv name>_charts.png)")
    parser.add_argument("--show", action="store_true", help="Open an interactive window as well")
    args = parser.parse_args(argv)

    rows = read_rows(args.csv)
    scenes = ordered_unique(r["label"] or "(no label)" for r in rows)
    planners = ordered_unique(r["planner"] for r in rows)
    assigned = {}
    xs = np.arange(len(planners))

    if not args.show:
        matplotlib.use("Agg")

    fig, axes = plt.subplots(3, len(scenes), figsize=(3.2 * len(scenes) + 1.5, 8.5), squeeze=False, sharey="row")
    fig.patch.set_facecolor("white")

    print(f"{'scene':10s} {'planner':12s} {'success':>8s} {'time mean':>10s} {'time sd':>8s} {'length mean':>12s}")
    for col, scene in enumerate(scenes):
        ax_success, ax_time, ax_length = axes[:, col]
        for x, planner in zip(xs, planners):
            trials = [r for r in rows if (r["label"] or "(no label)") == scene and r["planner"] == planner]
            if not trials:
                continue
            good = [r for r in trials if r["success"]]
            colour = colour_for(planner, assigned)

            rate = 100 * len(good) / len(trials)
            ax_success.bar(x, rate, width=0.6, color=colour, zorder=3)
            ax_success.text(x, rate + 2, f"{len(good)}/{len(trials)}", ha="center", va="bottom", fontsize=8, color=INK)

            times = [r["planning_time"] for r in good if r["planning_time"]]
            lengths = [r["path_length"] for r in good if r["path_length"] is not None]
            if times:
                strip(ax_time, x, times, colour)
            if lengths:
                strip(ax_length, x, lengths, colour)

            sd = statistics.stdev(times) if len(times) > 1 else 0.0
            print(f"{scene:10s} {planner:12s} {len(good):>3d}/{len(trials):<4d} "
                  f"{statistics.mean(times) if times else float('nan'):>10.3f} {sd:>8.3f} "
                  f"{statistics.mean(lengths) if lengths else float('nan'):>12.3f}")

        ax_success.set_title(f"scene: {scene}", fontsize=11, color=INK, loc="left")
        ax_success.set_ylim(0, 115)
        ax_time.set_yscale("log")
        for ax in (ax_success, ax_time, ax_length):
            style_axis(ax)
            ax.set_xticks(xs)
            ax.set_xticklabels(planners, rotation=20, ha="right")
            ax.set_xlim(-0.6, len(planners) - 0.4)
        if col == 0:
            ax_success.set_ylabel("success rate (%)", color=INK)
            ax_time.set_ylabel("planning time (s), log scale\ndots = trials, bar = mean", color=INK)
            ax_length.set_ylabel("path length (rad)\ndots = trials, bar = mean", color=INK)

    fig.suptitle(f"plan_benchmark results: {args.csv}", fontsize=12, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    out = args.out or args.csv.rsplit(".", 1)[0] + "_charts.png"
    fig.savefig(out, dpi=150)
    print(f"\nWrote {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
