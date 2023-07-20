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
        * [General]
           ![ik_general](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/3e4deab9-e112-453d-bb16-e3477813e840)

        * [Augmented],
        * [Nullspace projection]
        * [Repelling]
        * [RRT*]
        
        
* Trajectory Planning method
    1. [Task space planning]
        * [Quintic]
          ![ik_traj](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/7e8306b3-c8d3-496d-91b7-29479496535d)

          ![minimum_jerk](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/b0cafa74-e703-4003-a7bb-d05e28fb1d2c)

        * [Minimum Jerk]
        * [Linear movement]
    2. [Velocity profile method]: 
        * [Trapezoidal]
        * [s-Spline method]
    
* Demos
  1. [Pick-n-Place]
     ![pick-n-place-multiple-objects](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/c851c6dc-34cb-4031-9bba-947336d3b81e)

    
* Mobile Planning method
    1. [Mobile Velocity Control]
        * ![husky_ur](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/94036fc6-2e77-4ea0-a970-7c49e4d45878)
    2. [Global planner]
        * [A*]
        * [RRT]
        * [RRT*]
    3. [Local planner]
        * [Pure-pursuit]
        
        
* Point-cloud
    1. [Point-cloud Projection]
       ![pcd_projection](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/9be34382-3e8c-460b-84ea-bf403d3277e2)
       ![Screenshot from 2023-06-29 10-48-22](https://github.com/joonhyung-lee/mujoco-robotics-usage/assets/68883036/a2577c7e-8d3c-4d6a-8edc-157c2d7107ec)

    2. [RANSAC]
    3. [Iterative Closet Point]
    4. [Extrinsic calibration]
    
    
* Segmentations
    * [Unseen Object Clustering (UCN)](https://github.com/NVlabs/UnseenObjectClustering)
    * [Segment-Anything (SAM)](https://github.com/facebookresearch/segment-anything)


