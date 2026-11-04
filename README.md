# Tactical Retreat Replication Package
This is a replication package to refer to the data and plots for the tactical retreat paper.

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

## SUAVE - Plot Representative run

`export_representative_run_tikz.py` exports one run of one manager
(default `rebetmc`) as PGFPlots tables: the requirement fulfillment
(`--requirements`, default thruster availability and search footprint; the
combined reaction time is also available), the water visibility, the
adaptations, and the mission-phase and thruster-failure intervals. It also
writes a standalone `representative_run.tex` figure, a `caption.tex` sentence
listing the numbered adaptations, and a `manifest.json`.

```Bash
B=~/suave/results/batches/<batch>
python3 src/suave/suave_requirements/scripts/export_representative_run_tikz.py $B
python3 src/suave/suave_requirements/scripts/export_representative_run_tikz.py \
  $B --run-id run_0_24 --xmax 150 --compile
```

Output goes to
`<batch>/campaign_results/<campaign>/representative_run/<manager>_<run_id>/`
(or `--output-dir DIR/<campaign>/<manager>_<run_id>/`). `--xmax` ends the
figure's x axis at that mission time (the tables keep the whole run).
`--no-water-visibility` leaves out the water-visibility panel and writes
`representative_run_no_water_visibility.tex` instead, so both versions can
sit side by side.
`--compile` runs `pdflatex` on the figure when it is installed.

- **Run selection:** among accepted runs that found the pipeline, the run
  minimizing the summed `|value - median| / SD` of search time, distance
  inspected and mean reaction time (from `<manager>_suave.csv`, joined by
  `attempt_id`). The ranking is printed and kept in `manifest.json`;
  `--run-id` overrides it.
- **Time:** mission time, in seconds since the first activation of
  `f_generate_search_path`, used by every table. Values recorded before it
  are held at zero.
- **Search-only series:** search footprint and water visibility end at the
  first `/pipeline/detected` true, holding their last value up to it.
- **Adaptations** are actuation times: `spiral_altitude` changes of
  `/f_generate_search_path_node` on `/parameter_events` (the startup
  declaration is ignored) and manager activations/deactivations on
  `/f_maintain_motion_node/transition_event`.
- **Intervals:** searching runs from the first activation of
  `f_generate_search_path` to the first `/pipeline/detected` true,
  inspecting from there to `/mission_metrics/done` (or bag end), and thruster
  failure while `operational_thrusters` is below `--required-thrusters`
  (default `6`).
- **Water-visibility thresholds** come from the run's mission config and are
  labelled with their `expected_altitude`: visibility at or above a threshold
  expects that search altitude.
