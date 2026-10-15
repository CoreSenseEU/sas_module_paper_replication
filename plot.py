import argparse
from pathlib import Path
import textwrap

from rosbags.highlevel import AnyReader
import matplotlib.pyplot as plt
from matplotlib.transforms import Bbox
import numpy as np
import pandas as pd
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typedb_tactics.fuzzy.fuzzy_expressions import Proposition, Expression, FuzzyAnd, FuzzyOr, FuzzyNot
from typedb_tactics.fuzzy.fuzzy_sets import FuzzyTriangular
from typedb_tactics.fuzzy.relaxed_requirements import AsCloseAsPossibleExpression, ShallExpression


def just_this_requirement(req_list, req_name):
    for req in req_list:
        if req.name == req_name:
            return req.fulfillment
    return np.nan  # nan instead of None: matplotlib leaves a gap

def parse_config(s):
    # print(eval(s))
    return set(eval(s))


def content_bbox(fig, pad=0.05):
    # Axes with axis("off") still span their whole grid cell in a tight bbox,
    # so only count what is drawn in them (legend, text boxes).
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    boxes = []
    for ax in fig.axes:
        if ax.axison:
            boxes.append(ax.get_tightbbox(renderer))
            continue
        if ax.get_legend() is not None:
            boxes.append(ax.get_legend().get_window_extent(renderer))
        for text in ax.texts:
            patch = text.get_bbox_patch()
            boxes.append((patch or text).get_window_extent(renderer))
    bbox = Bbox.union(boxes).transformed(fig.dpi_scale_trans.inverted())
    return bbox.padded(pad)


def plot_bag(bag_path, req_names, title, min_person_gap=1.0):
    req_times = []
    req_values = {name: [] for name in req_names}
    req_values["MoveQuickly"] = []
    room_times = []
    room_values = []
    cmd_times = []

    config_change_times = []
    config_change_labels = []
    prev_config = None

    fz = FuzzyTriangular(a=-0.52, b=0.0, c=0.52)

    # A person is a run of non-empty /people messages bookended by empty ones,
    # so count each empty -> non-empty transition. Empty gaps shorter than
    # min_person_gap seconds are detector dropouts of the same person.
    persons_detected = 0
    prev_people_empty = True
    last_person_time = None

    with AnyReader([bag_path]) as reader:
        t0 = reader.start_time  # one shared zero for every topic
        topics = ["/requirements", "/global_costmap/current_room", "/fm_configuration", "/people", "/cmd_vel"]
        conns = [c for c in reader.connections if c.topic in topics]
        for conn, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t_sec = (t - t0) * 1e-9

            if conn.topic == "/requirements":
                req_times.append(t_sec)
                for name in req_names:
                    req_values[name].append(just_this_requirement(msg.requirements, name))
                    
            elif conn.topic == "/cmd_vel":
                cmd_times.append(t_sec)
                
                lin_vel = math.hypot(abs(msg.linear.x),abs(msg.linear.y))
                # if(lin_vel > 0.39):
                #     input(lin_vel)
                if(lin_vel < 0.09):
                    lin_vel = 0.00

                p = Proposition(name="lin_vel", value=lin_vel)
                rq = fz.membership(lin_vel)
                rq = ShallExpression(AsCloseAsPossibleExpression(proposition=p, fuzzy_set=fz,
                                               required_value=float(0.52))).fulfillment
                req_values["MoveQuickly"].append(rq)

            elif conn.topic == "/global_costmap/current_room":
                room_times.append(t_sec)
                room_values.append(1 if msg.data == "workcell" else 0)

            elif conn.topic == "/fm_configuration":
                config = parse_config(msg.data)
                # print(config)
                if prev_config is not None and config != prev_config:
                    config_change_times.append(t_sec)
                    config_change_labels.append(str(config.difference(prev_config)))
                prev_config = config

            elif conn.topic == "/people":
                people_empty = len(msg.people) == 0
                if prev_people_empty and not people_empty and (
                    last_person_time is None
                    or t_sec - last_person_time >= min_person_gap
                ):
                    persons_detected += 1
                if not people_empty:
                    last_person_time = t_sec
                prev_people_empty = people_empty
    mapping_lines = [
        line
        for i, text in enumerate(config_change_labels, start=1)
        for line in textwrap.wrap(
            f"{i}: Select{text}", width=34, subsequent_indent="   "
        )
    ]
    fig_height = max(5, 2 + 0.22 * len(mapping_lines))
    fig = plt.figure(figsize=(11, fig_height), constrained_layout=True)
    grid = fig.add_gridspec(1, 2, width_ratios=[4, 2])
    ax = fig.add_subplot(grid[0, 0])
    info_grid = grid[0, 1].subgridspec(
        2, 1, height_ratios=[len(req_names) + 3, max(1, len(mapping_lines))]
    )
    legend_ax = fig.add_subplot(info_grid[0, 0])
    mapping_ax = fig.add_subplot(info_grid[1, 0])
    legend_ax.axis("off")
    mapping_ax.axis("off")

    # Background shading from the room topic, using its own timestamps.
    # step="post" holds each value until the next message arrives.
    room_times = np.array(room_times)
    room_values = np.array(room_values)
    trans = ax.get_xaxis_transform()  # x in data units, y from 0 to 1 = full axis height
    ax.fill_between(room_times, 0, 1, where=room_values == 1, step="post",
                    color="red", alpha=0.10, transform=trans, label="Workcell")
    ax.fill_between(room_times, 0, 1, where=room_values == 0, step="post",
                    color="blue", alpha=0.10, transform=trans, label="Hallway")

    # Requirement lines, all sharing the /requirements timestamps
    paper_name = {
        "MoveQuickly" : "MaxThroughput",
        "CameraMotionAlignment": "CameraMotionAlignment",
        "MoveSafely" : "SocialDistancing"
    }
    for name in req_names:
        ax.plot(req_times, req_values[name], label=paper_name[name])

    ax.plot(cmd_times,req_values["MoveQuickly"], label=paper_name["MoveQuickly"]) #this is because of the retroactive calc to fix a mistake

    for i, t_change in enumerate(config_change_times):
        ax.axvline(t_change, color="black", linestyle="--", linewidth=1,
                   label="Adaptation" if i == 0 else None)
        ax.text(t_change, 0.98, str(i + 1), transform=ax.get_xaxis_transform(),
                va="top", ha="center", fontsize=9,
                bbox={"facecolor": "white", "alpha": 0.90, "edgecolor": "none", "pad": 1})

    ax.set_xlabel("Time since bag start (s)")
    ax.set_ylabel("Satisfaction")
    ax.set_title(title)
    handles, labels = ax.get_legend_handles_labels()
    legend_ax.legend(handles, labels, loc="upper left", frameon=False)
    mapping_ax.text(
        0.02, 0.98, "\n".join(mapping_lines) or "No configuration changes",
        transform=mapping_ax.transAxes, va="top", ha="left", fontsize=8,
        family="monospace",
        bbox={"facecolor": "white", "edgecolor": "gray", "boxstyle": "round,pad=0.5"},
    )
    run_means = {
        name: float(np.nanmean(values)) if not np.isnan(values).all() else np.nan
        for name, values in req_values.items()
    }
    run_max = {
        name: float(np.nanmax(values)) if not np.isnan(values).all() else np.nan
        for name, values in req_values.items()
    }
    return fig, run_means, run_max, persons_detected


def main():
    parser = argparse.ArgumentParser(
        description="Plot the requirements topics from each bag directory in a folder."
    )
    parser.add_argument("folder", type=Path, help="Folder containing timestamp-named bag directories")
    parser.add_argument("--min-person-gap", type=float, default=1.0,
                        help="Seconds of empty /people messages needed before a new person is counted")
    args = parser.parse_args()

    if not args.folder.is_dir():
        parser.error(f"not a folder: {args.folder}")

    bag_paths = sorted(path for path in args.folder.iterdir() if path.is_dir())
    if not bag_paths:
        parser.error(f"no bag directories found in: {args.folder}")

    title = input("What title should the plots have? > ")
    req_names = ["MoveSafely", "CameraMotionAlignment"]
    run_means = {}
    run_maxes = {}
    persons_detected = {}
    for bag_path in bag_paths:
        fig, run_means[bag_path.name], run_maxes[bag_path.name], persons_detected[bag_path.name] = plot_bag(
            bag_path, req_names, title, args.min_person_gap
        )
        output_path = args.folder / f"{bag_path.name}.pdf"
        fig.savefig(output_path, format="pdf", bbox_inches=content_bbox(fig))
        plt.close(fig)
        print(f"Saved {output_path}")

    run_means_df = pd.DataFrame.from_dict(run_means, orient="index")
    run_maxes_df = pd.DataFrame.from_dict(run_maxes, orient="index")

    per_run_df = run_means_df.assign(**{"Persons detected": pd.Series(persons_detected)})
    print(per_run_df)
    summary_df = pd.DataFrame({
        "Mean": per_run_df.mean(axis=0),
        "Max": run_maxes_df.max(axis=0),
        "Runs averaged": per_run_df.count(axis=0),
    }).rename_axis("Metric")
    table_path = args.folder / "mean_satisfaction.txt"
    table_path.write_text(
        summary_df.to_latex(float_format="%.3f"),
        encoding="utf-8",
    )
    print(f"Saved {table_path}")

    per_run_table_path = args.folder / "per_run_satisfaction.txt"
    per_run_table_path.write_text(
        per_run_df.to_latex(
            float_format="%.3f",
            index=True,
            index_names=["Run"],
            na_rep="--",
        ),
        encoding="utf-8",
    )
    print(f"Saved {per_run_table_path}")


if __name__ == "__main__":
    main()