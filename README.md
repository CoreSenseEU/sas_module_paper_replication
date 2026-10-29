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