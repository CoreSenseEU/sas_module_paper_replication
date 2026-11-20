# Tactical Retreat Replication Package
This is a replication package to refer to the data and plots for the tactical retreat paper.

## Getting the data (Git LFS)

`results/suave-tactical-submission.zip` is stored with [Git LFS](https://git-lfs.com).
Without Git LFS, cloning still works but that file is only a small text pointer, not the actual zip.
Install Git LFS before cloning:

```Bash
git lfs install
git clone git@github.com:CoreSenseEU/sas_module_paper_replication.git
```

If you already cloned without it, run `git lfs install && git lfs pull` inside the repository.
Alternatively, download the zip directly from the GitHub web interface.

## Setup

Use Python 3.10 or newer. From the repository directory:

```Bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements
```

Alternatively, rebuild/reopen the development container in VS Code to install
the dependencies for all plotting scripts. You can simply open VS Code in the root of the repository (where .devcontainer) lives, and use the dev container (see https://code.visualstudio.com/docs/devcontainers/containers).

To build and run the container directly, from the repository directory:

```Bash
docker build -t sas_replication:latest -f .devcontainer/Dockerfile .
docker run --rm -it --user "$(id -u):$(id -g)" \
  -v "$PWD:/workspace" -w /workspace sas_replication:latest bash
```

Run the plotting commands below inside the container.

## Quality Inspection Testbed - Real World Evaluation

There are two python scripts, plot.py and plot_cbr.py.
These are both for the quality inspection testbed portion of the evaluation.
plot.py is for the 'demo' scenario where the robot travels between workcells, referred to as eval1 in the results.
plot_cbr.py is for the specific rear LiDAR failure scenario, where the robot only drives backwards within the workcell shortly, referred to as eval2 in the results.

Both scripts work about the same, they expect you to provide a directory within whichs are other directories each containing a ros bag.
Plots and a couple of tables, one averaging and one per run are then produced in that root folder.

You would plot the results of the baseline for Evaluation 1 as follows:
```
python3 plot.py results/eval1-baseline/
```

## SUAVE - Simulated Evaluation

[SUAVE](https://github.com/kas-lab/suave) is a ROS 2 exemplar in which an
underwater vehicle searches for and inspects a pipeline while thrusters fail
and water visibility changes. The data come from one campaign that
compares `TacticalRetreat` with SUAVE's other managing systems.

### Data

`results/suave-tactical-submission.zip` holds batch `batch_20261012_192250`:

| Path in the batch | Contents |
|---|---|
| `campaigns/exp3/rosbags/` | One ROS bag per run |
| `campaigns/exp3/<manager>_suave.csv` | Per-run mission metrics for each manager |
| `campaigns/exp3/*_reaction_time.csv`, `*_recovery_time.csv` | Reaction and recovery times |
| `campaigns/exp3/mission_config_run<N>.yaml`, `runs.json` | Mission configuration of each run |
| `campaign_results/exp3/requirement_fulfillment/` | Fulfillment of each requirement over time, compared across managers |
| `campaign_results/exp3/representative_run/rebetmc_run_0_9/` | Representative `TacticalRetreat` run: PGFPlots tables, `representative_run.tex` and `.pdf` |

From this repository directory, extract the data with:

```Bash
unzip results/suave-tactical-submission.zip -d results/suave/
```

The plots and tables used in the paper are already in `campaign_results/`, so
the steps below are only needed to regenerate them.

### Regenerating the representative-run figure

`export_representative_run_tikz.py` runs without ROS or SUAVE packages:

```Bash
python3 export_representative_run_tikz.py \
  results/suave/batch_20261012_192250 \
  --output-dir results/suave/regenerated
```

`export_representative_run_tikz.py` picks the run closest to the median of
search time, distance inspected and mean reaction time, which is `run_0_9`
for this batch. `--run-id` chooses another run. `--xmax` shortens the
figure's x axis and `--no-water-visibility` leaves out the water-visibility
panel. Outputs go to
`results/suave/regenerated/exp3/rebetmc_run_0_9/` with the command above.
Without `--output-dir`, they go to the batch's
`campaign_results/exp3/representative_run/rebetmc_run_0_9/`, replacing files
with the same names. Pass `--help` instead of the batch path to see all of
a script's options.

### Compiling the figures

The previous command, only generates `.tex` files. To actually generate a `.pdf` with the figure, you need to install latex and some latex utils.

In ubuntu/debian:

```bash
sudo apt install texlive-latex-extra texlive-pictures poppler-utils
```

With these dependencies installed, you can compile the `.tex` files and generate a PDF by passing the `--compile` flag:

```Bash
python3 export_representative_run_tikz.py  --compile \
  results/suave/batch_20261012_192250 \
  --output-dir results/suave/regenerated
```

## XText Entries
We provide the XText entries for the evaluation in the models folder.