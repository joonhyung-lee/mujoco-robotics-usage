# mujoco-robotics-usage
This repo provides minimal hands-on code for MuJoCo Robotics Algorithms.

(This repository only covers the MuJoCo simulation. Realworld is not included)

MuJoCo related code is employed from following DeepMind's repos: 

* [MuJoCo] https://github.com/deepmind/mujoco

## Prerequisites

This repo is tested on following environment:

* Ubuntu: 20.04
* Python: 3.8.10
* mujoco: 2.3.2

### Install dependencies

Mujoco Engine
```bash
pip install mujoco

pip install mujoco-python-viewer
```

### Descriptions
Below is a list of files and their descriptions:

* Kinematic Solver
    1. Solve inverse kinematics in various method with 
        * [General], 
        * [Augmented],
        * [Nullspace projection]
        * [Repelling]
        * [RRT*]
* Trajectory Planning method
    1. [Task space planning]
        * [Quintic]
        * [Minimum Jerk]
        * [Linear movement]
    2. [Velocity profile method]: 
        * [Trapezoidal]
        * [s-Spline method]
    
    
* Mobile Planning method
    1. [Global planner]
        * [A*]
        * [RRT]
        * [RRT*]
    2. [Local planner]
        * [Pure-pursuit]
