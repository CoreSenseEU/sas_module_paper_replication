import argparse
from pathlib import Path
import textwrap

from rosbags.highlevel import AnyReader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import mannwhitneyu


def just_this_requirement(req_list, req_name):
    for req in req_list:
        if req.name == req_name:
            return req.fulfillment
    return np.nan  # nan instead of None: matplotlib leaves a gap

def parse_config(s):
    # print(eval(s))
    return set(eval(s))

def cmd_vel_nonzero(twist_msg, tolerance=1e-6):
    if tolerance < 0:
        raise ValueError("tolerance must be non-negative")

    return any((
        abs(twist_msg.linear.x) > tolerance,
        abs(twist_msg.linear.y) > tolerance,
        abs(twist_msg.linear.z) > tolerance,
        abs(twist_msg.angular.x) > tolerance,
        abs(twist_msg.angular.y) > tolerance,
        abs(twist_msg.angular.z) > tolerance,
    ))

def plot_bag(bag_path, req_names, title, cmd_vel_tolerance=1e-6):
    req_times = []
    req_values = {name: [] for name in req_names}
    cmd_vel_times = []
    cmd_vel_values = []
    moving_times = []
    moving_values = []
    u_rear = []
    config_change_times = []
    config_change_labels = []
    prev_config = None

    with AnyReader([bag_path]) as reader:
        t0 = reader.start_time  # one shared zero for every topic
        topics = ["/cmd_vel", "/requirements", "/fm_configuration", "/parameter_events"]
        # print([c.topic for c in reader.connections])
        conns = [c for c in reader.connections if c.topic in topics]
        moving = False
        for conn, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t_sec = (t - t0) * 1e-9
            if conn.topic == "/cmd_vel":
                moving_times.append(t_sec)
                moving_values.append(1 if cmd_vel_nonzero(msg, tolerance=cmd_vel_tolerance) else 0)
                moving = cmd_vel_nonzero(msg, tolerance=cmd_vel_tolerance)
            elif conn.topic == "/requirements":
                req_times.append(t_sec)
                for name in req_names:
                    req_eval = just_this_requirement(msg.requirements, name)
                    req_values[name].append(req_eval)
                    if(name == "SafeRearMovement"):
                        if(moving):
                            u_rear.append(req_eval)
                        else:
                            u_rear.append(0.0)


            elif conn.topic == "/fm_configuration":
                config = parse_config(msg.data)
                # print(config)
                if prev_config is not None and config != prev_config:
                    config_change_times.append(t_sec)
                    config_change_labels.append(str(config.difference(prev_config)))
                prev_config = config
            elif conn.topic == "/parameter_events":
                print(t_sec)
                # # input(">")
                # config_change_times.append(t_sec)
                # config_change_labels.append(str("param_event"))

    translation_singular = {
        "{'HigherVelocity'}" : "Maximum velocity set to increased",
        "{'SocialLayer'}" : "Social costmap layer activated",
        "{'MPPI_DIFF'}" : "Robot motion model set to differential drive",
        "{'MPPI_OMNI'}" : "Robot motion model set to omnidirectional drive",
        "{'DetectHumans'}" : "People detection activated",
        "{'DefaultVelocity'}" : "Maximum velocity set to default",
        "{'RecoverLidar'}" : "Attempt recovery of broken LiDAR sensor"
    }

    translation_double = {
        "{'HigherVelocity', 'DetectHumans'}" : translation_singular["{'HigherVelocity'}"] + " and " + translation_singular["{'DetectHumans'}"],
        "{'DetectHumans', 'HigherVelocity'}" : translation_singular["{'HigherVelocity'}"] + " and " + translation_singular["{'DetectHumans'}"],
    }

    translation_double.update(translation_singular)
    mapping_lines = [
        line
        for i, (t_change, text) in enumerate(zip(config_change_times, config_change_labels), start=1)
        for line in textwrap.wrap(
            f"{i}: {translation_double[text]} ({t_change:.1f} s)", width=34, subsequent_indent="   "
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
    moving_times = np.array(moving_times)
    moving_values = np.array(moving_values)
    trans = ax.get_xaxis_transform()  # x in data units, y from 0 to 1 = full axis height
    ax.fill_between(moving_times, 0, 1, where=moving_values == 1, step="post",
                    color="red", alpha=0.10, transform=trans, label="Moving")
    ax.fill_between(moving_times, 0, 1, where=moving_values == 0, step="post",
                    color="blue", alpha=0.10, transform=trans, label="Stationary")

    # Requirement lines, all sharing the /requirements timestamps
    for name in req_names:
        ax.plot(req_times, req_values[name], label=name)


    for i, t_change in enumerate(config_change_times):
        ax.axvline(t_change, color="black", linestyle="--", linewidth=1,
                   label="Adaptation" if i == 0 else None)
        ax.text(t_change, 0.98, str(i + 1), transform=ax.get_xaxis_transform(),
                va="top", ha="center", fontsize=8,
                bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none", "pad": 1})

    ax.set_xlabel("Time since bag start (s)")
    ax.set_ylabel("Satisfaction degree")
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
    u_rear_means = {
        name: float(np.nanmean(u_rear)) if not np.isnan(u_rear).all() else np.nan
    }
    return fig, run_means, u_rear_means


def process_folder(folder, req_names, title, cmd_vel_tolerance):
    """Plot every bag in `folder`, write its tables, and return the per-run U values."""
    bag_paths = sorted(path for path in folder.iterdir() if path.is_dir())
    if not bag_paths:
        raise SystemExit(f"no bag directories found in: {folder}")

    run_means = {}
    u_rear_means = {}

    for bag_path in bag_paths:
        fig, run_means[bag_path.name], u_rear_means[bag_path.name] = plot_bag(
            bag_path, req_names, title, cmd_vel_tolerance=cmd_vel_tolerance
        )
        output_path = folder / f"{bag_path.name}.pdf"
        fig.savefig(output_path, format="pdf")
        plt.close(fig)
        print(f"Saved {output_path}")

    run_means_df = pd.DataFrame.from_dict(run_means, orient="index")
    u_rear_df = pd.DataFrame.from_dict(u_rear_means, orient="index")
    summary_df = pd.DataFrame({
        "Mean satisfaction": run_means_df.mean(axis=0),
        "Mean U Satisfaction": u_rear_df.mean(axis=0),
        "Runs averaged": run_means_df.count(axis=0),
    }).rename_axis("Requirement")
    table_path = folder / "mean_satisfaction.txt"
    table_path.write_text(
        summary_df.to_latex(float_format="%.3f"),
        encoding="utf-8",
    )
    print(f"Saved {table_path}")

    per_run_table_path = folder / "per_run_satisfaction.txt"
    per_run_table_path.write_text(
        run_means_df.to_latex(
            float_format="%.3f",
            index=True,
            index_names=["Run"],
            na_rep="--",
        ),
        encoding="utf-8",
    )
    print(f"Saved {per_run_table_path}")

    return u_rear_df


def holm(p_values):
    """Holm-Bonferroni adjusted p-values (controls family-wise error rate)."""
    p = np.asarray(p_values, dtype=float)
    m = len(p)
    order = np.argsort(p)
    adjusted = np.empty(m)
    running_max = 0.0
    for rank, idx in enumerate(order):
        running_max = max(running_max, (m - rank) * p[idx])
        adjusted[idx] = min(1.0, running_max)
    return adjusted


def mann_whitney_vs_target(u_per_folder, target, req_name):
    """One-sided Mann-Whitney U: is the target's per-run U stochastically greater than each other folder's?"""
    x = u_per_folder[target][req_name].dropna().to_numpy()
    rows = {}
    for name, df in u_per_folder.items():
        if name == target:
            continue
        y = df[req_name].dropna().to_numpy()
        if len(x) == 0 or len(y) == 0:
            stat, p, r = np.nan, np.nan, np.nan
        else:
            # method="auto" -> exact distribution for small samples without ties
            stat, p = mannwhitneyu(x, y, alternative="greater")
            # Rank-biserial correlation: +1 means every target run beats every other run
            r = 2 * stat / (len(x) * len(y)) - 1
        rows[name] = {
            "n target": len(x),
            "n other": len(y),
            "Mean target": np.mean(x) if len(x) else np.nan,
            "Mean other": np.mean(y) if len(y) else np.nan,
            "Median target": np.median(x) if len(x) else np.nan,
            "Median other": np.median(y) if len(y) else np.nan,
            "MWU statistic": stat,
            "p (one-sided)": p,
            "r (rank-biserial)": r,
        }
    results_df = pd.DataFrame.from_dict(rows, orient="index").rename_axis(f"{target} vs")
    valid = results_df["p (one-sided)"].notna()
    results_df["p (Holm)"] = np.nan
    if valid.any():
        results_df.loc[valid, "p (Holm)"] = holm(results_df.loc[valid, "p (one-sided)"])
    return results_df


def main():
    parser = argparse.ArgumentParser(
        description="Plot the requirements topics from each bag directory in one or more folders, "
                    "and test whether the target folder's per-run U is greater than the others'."
    )
    parser.add_argument("folders", type=Path, nargs="+",
                        help="Folder(s) containing timestamp-named bag directories")
    parser.add_argument("--target", type=Path, default=None,
                        help="Folder whose U is tested to be greater than the others "
                             "(default: first folder)")
    parser.add_argument("--title", default=None,
                        help="Plot title (default: prompt once per folder)")
    parser.add_argument("--output", type=Path, default=None,
                        help="Path for the LaTeX Mann-Whitney table "
                             "(default: <target>/mann_whitney_u.txt)")
    parser.add_argument(
        "--cmd-vel-tolerance",
        type=float,
        default=0.02,
        help="Absolute component threshold for treating /cmd_vel as nonzero (default: 0.02)",
    )
    args = parser.parse_args()

    for folder in args.folders:
        if not folder.is_dir():
            parser.error(f"not a folder: {folder}")
    if args.cmd_vel_tolerance < 0:
        parser.error("--cmd-vel-tolerance must be non-negative")

    target = args.target if args.target is not None else args.folders[0]
    if target.resolve() not in [f.resolve() for f in args.folders]:
        args.folders.insert(0, target)
    if not target.is_dir():
        parser.error(f"not a folder: {target}")

    req_names = ["SafeRearMovement"]

    # Keyed by folder name for readable table rows
    u_per_folder = {}
    for folder in args.folders:
        title = args.title if args.title is not None else input(
            f"What title should the plots in {folder.name} have? > "
        )
        u_per_folder[folder.name] = process_folder(
            folder, req_names, title, args.cmd_vel_tolerance
        )

    if len(u_per_folder) < 2:
        print("Only one folder given: skipping Mann-Whitney U test.")
        return

    results_df = mann_whitney_vs_target(u_per_folder, target.name, "SafeRearMovement")
    print()
    print(f"One-sided Mann-Whitney U on per-run U (alternative: {target.name} > other)")
    print(results_df.to_string(float_format=lambda v: f"{v:.4f}"))

    output = args.output if args.output is not None else target / "mann_whitney_u.txt"
    output.write_text(
        results_df.to_latex(float_format="%.3f", na_rep="--"),
        encoding="utf-8",
    )
    print(f"Saved {output}")


if __name__ == "__main__":
    main()