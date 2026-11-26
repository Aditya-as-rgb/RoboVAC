# RoboVAC re-clean environment

`robovac-rl` provides a Gymnasium environment for deciding which dirt detections to revisit after a grid coverage pass. The environment imports the same coverage planner, meter, seeded dirt generator, and brush model used by RoboVAC. Its wheel bundles those Python sources and the living-room map for use without ROS or a simulator.

Install the environment with `pip install .` from this directory. Add the `train` extra to run `python -m vacuum_rl.experiment` for masked PPO training and evaluation. See the [RoboVAC README](../README.md#vacuum_rl) for commands, assumptions, and results.
