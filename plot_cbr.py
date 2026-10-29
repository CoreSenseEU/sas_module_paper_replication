import argparse
from pathlib import Path
import textwrap

from rosbags.highlevel import AnyReader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


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


def main():
    parser = argparse.ArgumentParser(
        description="Plot the requirements topics from each bag directory in a folder."
    )
    parser.add_argument("folder", type=Path, help="Folder containing timestamp-named bag directories")
    parser.add_argument(
        "--cmd-vel-tolerance",
        type=float,
        default=1e-6,
        help="Absolute component threshold for treating /cmd_vel as nonzero (default: 1e-6)",
    )
    args = parser.parse_args()

    if not args.folder.is_dir():
        parser.error(f"not a folder: {args.folder}")
    if args.cmd_vel_tolerance < 0:
        parser.error("--cmd-vel-tolerance must be non-negative")

    bag_paths = sorted(path for path in args.folder.iterdir() if path.is_dir())
    if not bag_paths:
        parser.error(f"no bag directories found in: {args.folder}")

    title = input("What title should the plots have? > ")

    req_names = ["SafeRearMovement"]
    run_means = {}
    u_rear_means = {}

    for bag_path in bag_paths:
        fig, run_means[bag_path.name], u_rear_means[bag_path.name] = plot_bag(
            bag_path, req_names, title, cmd_vel_tolerance=args.cmd_vel_tolerance
        )
        output_path = args.folder / f"{bag_path.name}.pdf"
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
    table_path = args.folder / "mean_satisfaction.txt"
    table_path.write_text(
        summary_df.to_latex(float_format="%.3f"),
        encoding="utf-8",
    )
    print(f"Saved {table_path}")

    per_run_table_path = args.folder / "per_run_satisfaction.txt"
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


if __name__ == "__main__":
    main()