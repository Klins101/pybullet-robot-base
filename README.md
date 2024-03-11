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
This repository presents Python code implementing the JPI RRT (Jacobian Pseudoinverse-Based RRT) motion planning algorithm for a 7 DOF Panda robot using JPI RRT as a requirement for a PhD in Computer Science at Aston University. moving towards the goal configuration while avoiding collisions with obstacles in the workspace.
This implementation utilizes Python with libraries like `numpy` and `matplotlib`.
The JPI-RRT algorithm combines random sampling with the Jacobian pseudoinverse to efficiently explore the configuration space of a robot arm and find collision-free paths from an initial configuration to a goal configuration. The algorithm iteratively builds a tree structure, gradually 

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

1. **Initialization**:
   - The JPI RRT algorithm is initialized with the start configuration of the robot arm and an empty tree containing only the start configuration.

2. **Expansion**:
   - Iteratively expand the tree by generating random configurations in the workspace.
   - Use the Jacobian pseudoinverse to compute joint velocities that move the end effector towards the randomly generated configurations.
   - Update the current configuration of the robot arm based on the computed joint velocities.
   - Ensure that the generated configurations are collision-free by performing collision checking.
   - If a collision-free configuration is found, add it to the tree.

3. **Goal Biasing**:
   - To speed up convergence, we occasionally bias the tree expansion towards the goal configuration to speed up convergence by sampling more points around the goal configuration.

4. **Termination**:
   - The algorithm is terminated when the maximum number of iterations is reached.

5. **Output**:
   - If a path from the start to the goal configuration is found, return the sequence of configurations that form the path.
   - If no path is found within the maximum number of iterations, return a failure message.


## Configuration

You can adjust various parameters in the algorithm to customize its behavior:

- `max_iter`: Maximum number of iterations.
- `step_size`: Step size for extending the tree towards random configurations.
- `threshold`: Threshold distance to consider a configuration close to the goal.


## Acknowledgments
This implementation was inspired by the $J^+$-RTT algorithm as described in the research literature on [Humanoid motion planning for dual-arm manipulation and re-grasping tasks by Vahrenkamp, Nikolaus, et al. ](https://h2t.iar.kit.edu/pdf/Vahrenkamp2009b.pdf). The environment and was created by [Martin Rudorfer] (https://github.com/mrudorfer/pybullet-robot-base) 