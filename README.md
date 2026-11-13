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

## Quality Inspection Testbed - Real World Evaluation

There are two python scripts, plot.py and plot_cbr.py.
These are both for the quality inspection testbed portion of the evaluation.
plot.py is for the 'demo' scenario where the robot travels between workcells, referred to as eval1 in the results.
plot_cbr.py is for the specific rear LiDAR failure scenario, where the robot only drives backwards within the workcell shortly, referred to as eval2 in the results.

Both scripts work about the same, they expect you to provide a directory within whichs are other directories each containing a ros bag.
Plots and a couple of tables, one averaging and one per run are then produced in that root folder.

You would plot the results of the baseline for eval1 as follows:
```
python3 plot.py 
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

Extract it with:

```Bash
unzip results/suave-tactical-submission.zip -d $HOME/suave_results/
```

The plots and tables used in the paper are already in `campaign_results/`, so
the steps below are only needed to regenerate them.

### Regenerating the plots

The scripts read ROS 2 bags and use SUAVE's packages, so run them inside the
SUAVE Docker image `ghcr.io/kas-lab/suave-headless:main`. The scripts can be
found in [`suave_requirements/scripts`](https://github.com/kas-lab/suave/tree/main/suave_requirements/scripts),
and their README explains the requirements and the method in detail.
`export_representative_run_tikz.py` in this repository is a copy kept for
reference.

The commands below mount the extracted results at
`/home/ubuntu-user/suave_results` in the container, which starts in the SUAVE
workspace with it already sourced.

```Bash
docker pull ghcr.io/kas-lab/suave-headless:main

# Fulfillment of each requirement, compared across managers
docker run --rm \
  --mount type=bind,src=$HOME/suave_results,dst=/home/ubuntu-user/suave_results \
  ghcr.io/kas-lab/suave-headless:main \
  python3 src/suave/suave_requirements/scripts/plot_requirement_fulfillment.py \
  /home/ubuntu-user/suave_results/batch_20261012_192250

# Representative TacticalRetreat run (the paper figure)
docker run --rm \
  --mount type=bind,src=$HOME/suave_results,dst=/home/ubuntu-user/suave_results \
  ghcr.io/kas-lab/suave-headless:main \
  python3 src/suave/suave_requirements/scripts/export_representative_run_tikz.py \
  /home/ubuntu-user/suave_results/batch_20261012_192250
```

The container runs as a user with UID 1000. If your user has a different
UID, make `$HOME/suave_results` writable for it, or the scripts cannot save
their outputs. The outputs replace the ones shipped in the zip. Add
`--output-dir /home/ubuntu-user/suave_results/<dir>` after the batch path to
keep both.

`export_representative_run_tikz.py` picks the run closest to the median of
search time, distance inspected and mean reaction time, which is `run_0_9`
for this batch. `--run-id` chooses another run. `--xmax` shortens the
figure's x axis and `--no-water-visibility` leaves out the water-visibility
panel. Outputs go to
`$HOME/suave_results/batch_20261012_192250/campaign_results/exp3/`, or to
`--output-dir`. Pass `--help` instead of the batch path to see all of a
script's options.

### Compiling the figures

The SUAVE containers do not include LaTeX, so `--compile` is skipped there.
To compile a figure, run `pdflatex` on its `.tex` file on a machine with
LaTeX installed, including `pgfplots` and the `standalone` class (on Ubuntu,
`texlive-latex-extra`).
