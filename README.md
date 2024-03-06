# pybullet robot base

## installation

Using [Anaconda](https://docs.anaconda.com/free/anaconda/install/) or [Miniconda](https://docs.anaconda.com/free/miniconda/index.html):

```
conda create -n pybullet_robot_base python=3.10
conda activate pybullet_robot_base
conda install -c conda-forge pybullet numpy
```

## usage

There are two classes, a `Robot` and a `Goal` class.
The `Robot` class handles the simulation and offers a few functions:
- reset joint positions
- get current joint positions
- compute end effector position using forward kinematics
- check whether the robot is in collision with the environment or itself
- compute the Jacobian matrix
- visualise a goal position

The `Goal` class provides the goal position and a metric to check the distance and whether the goal has been reached or not.
There are six different goals, which can all be used in development and testing.

Finally, the `main.py` file shows some exemplary use of the classes.

## task

Your task is to implement a **simple** motion planning method that can find a collision-free trajectory from the robot's starting configuration to a configuration in which the robot's end effector reaches the goal.
There are a few things you should consider:
- implement your method using only **pure Python/numpy**, no external libraries are allowed; specifically, you are not allowed to use the inverse kinematics function of pybullet
- the **output** of your method should be **a list of collision-free waypoints** (arm configurations) with the last waypoint being in reach of the goal
- the starting configuration is always the same
- there are six goals available, but the method should only consider one goal at a time
- each goal is defined by its position, i.e. the orientation of the end effector does not matter
- you do not need to optimize for runtime or memory usage, however, the method should be able to find a valid path to at least some of the goals