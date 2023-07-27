# mujoco-robotics-usage
This repo provides minimal hands-on code for MuJoCo Robotics Algorithms.

(This repository only covers the MuJoCo simulation. Realworld is not included)

MuJoCo related code is employed from following repos: 

* [MuJoCo] https://github.com/deepmind/mujoco
* [YAMT] https://github.com/sjchoi86/yet-another-mujoco-tutorial-v3

Some of the robot models referenced mujoco_menagerie
* [mujoco_menagerie] https://github.com/deepmind/mujoco_menagerie

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
            <p float="left">
            <img src="./readme/gifs/ik_general.gif" width="49%" />
            <img src="./readme/gifs/wholebody_ik.gif" width="49%" />>
            </p>
        * [Augmented],
        * [Nullspace projection]
        * [Repelling]
        * [RRT*]
        
        
* Trajectory Planning method
    1. [Task space planning]
        * [Quintic]
        <p float="left">
        <img src="./readme/gifs/ik_traj.gif" width="49%" />
        <img src="./readme/gifs/minimum_jerk.gif" width="49%" />
        </p>
        * [Minimum Jerk]
        * [Linear movement]
    2. [Velocity profile method]: 
        * [Trapezoidal]
        * [s-Spline method]
    
* Demos
  1. [Pick-n-Place]
        <p align="center">
            <img src="./readme/gifs/pick-n-place-multiple-objects.gif" width="100%" alt="Pick-n-place multiple objects in succession on table-top scene.">
        </p>
    
* Mobile Planning method
    1. [Mobile Velocity Control]
        <p align="center">
            <img src="./readme/gifs/husky_ur.gif" width="100%" alt="Husky with UR just moving forward.">
        </p>
    2. [Global planner]
        * [A*]
        * [RRT]
        * [RRT*]
    3. [Local planner]
        * [Pure-pursuit]
        
        
* Point-cloud
    1. [Point-cloud Projection]
        <p align="center">
            <img src="./readme/image/project_pcd.png" width="100%" alt="Project point-cloud onto table from ego-centric view.">
        </p>
        <p align="center">
            <img src="./readme/gifs/pcd_projection.gif" width="100%" alt="Project point-cloud onto table from ego-centric view.">
        </p>

    2. [RANSAC]
    3. [Iterative Closet Point]
    4. [Extrinsic calibration]
    
    
* Segmentations
    * [Unseen Object Clustering (UCN)](https://github.com/NVlabs/UnseenObjectClustering)
    * [Segment-Anything (SAM)](https://github.com/facebookresearch/segment-anything)

* Miscellaneous
    * [Multi-vivwer] Add Multi-Viewer toy examples: Get images from camera defined in an XML(MJCF) files.
        <p float="left">
        <img src="./readme/gifs/multi-viewer-1.gif" width="49%" />
        <img src="./readme/gifs/multi-viewer-2.gif" width="49%" />>
        </p>