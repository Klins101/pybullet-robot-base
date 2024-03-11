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
- get joint limits
- compute end effector position using forward kinematics
- check whether the robot is in collision with the environment or itself
- compute the Jacobian matrix
- visualise a goal position

The `Goal` class provides the goal position and a metric to check the distance and whether the goal has been reached or not.
There are six different goals, which can all be used in development and testing.

Finally, the `main.py` file shows some exemplary use of the classes.
It should run without any modifications, show the robot in the starting configuration and visualise all the goals.
Once you hit Enter on the command line, it shows a different (colliding) joint configuration.

## task

## Overview

This repository contains Python code implementing the $ \( J^+ \)$  -RTT (Jacobian Pseudoinverse-Based RRT) motion planning algorithm. The $ J^+ $-RTT algorithm is a variant of the Rapidly exploring Random Trees (RRT) algorithm, designed for finding collision-free paths in a 3D environment. This implementation utilizes Python with libraries like `numpy`, `matplotlib`.

## Requirements

- Python 3.11
- numpy
- matplotlib

## Usage

1. Clone the repository:

    ```bash
    git clone https://github.com/Klins101/pybullet-robot-base
    cd pybullet-robot-base
    ```

2. Run the main script:

    ```bash
    python main.py
    ```

## How it Works

The \( J^+ \) -RTT algorithm works by iteratively building a tree structure in the configuration space of the robot arm. Here's a brief overview of the algorithm:

1. **Initialization**: Start with a tree containing only the initial configuration.
2. **Expansion**: Iteratively expand the tree by generating random configurations and extending the tree towards these configurations while ensuring collision-free paths.
3. **Goal Biasing**: Occasionally bias the tree expansion towards the goal configuration to speed up convergence.
4. **Termination**: Terminate the algorithm when the goal configuration is reached or a maximum number of iterations is reached.

## Configuration

You can adjust various parameters in the algorithm to customize its behavior:

- `max_iter`: Maximum number of iterations.
- `step_size`: Step size for extending the tree towards random configurations.
- `threshold`: Threshold distance to consider a configuration close to the goal.


## Acknowledgments
This implementation was inspired by the $J^+$-RTT algorithm as described in the research literature on [Humanoid motion planning for dual-arm manipulation and re-grasping tasks by Vahrenkamp, Nikolaus, et al. ](https://h2t.iar.kit.edu/pdf/Vahrenkamp2009b.pdf). The 