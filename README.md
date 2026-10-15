# sas_module_paper_replication
This is a replication package to refer to the data and plots for the paper about the SAS Module

There are two python scripts, plot.py and plot_cbr.py.
These are both for the NDT Testbed portion of the evaluation.
plot.py is for the 'demo' scenario where the robot travels between workcells.
plot_cbr.py is for the specific rear LiDAR failure scenario, where the robot only drives backwards within the workcell shortly.

Both scripts work about the same, they expect you to provide a directory within whichs are other directories each containing a ros bag.
Plots and a couple of tables, one averaging and one per run are then produced in that root folder.