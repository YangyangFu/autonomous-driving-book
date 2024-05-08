# Autonoumous Driving: Beginner to Master

This book aims to help beginners to learn autonomous driving from coding excercises. 
The content is organized in a bottom-up approach, by introducing local longitudinal and lateral PID controller first, and all the way up to perception.
All topics are demosntrated through closed-loop simulation using CARLA.
I found this approach is easier for students in mechanical/electrical engineering to start with.

The covered topics are:

- [1 Introduction](./book/1-introduction.md)
- [2 Vehicle Dynamics](./book/2-vehicle-dynamics/content.md)
    - [2.1 Kinematic Bicycle Model](./book/2-vehicle-dynamics/kinematic-bicycle.md)
    - [2.2 Dynamic Bicycle Model](./book/2-vehicle-dynamics/dynamic-bicycle.md)
- [3 Trajectory Tracking](./book/3-trajectory-tracking/content.md)
    - [3.1 Longitudinal Control](./book/3-trajectory-tracking/longitudinal-control/content.md)
        - [3.1.1 PID Controller](./book/3-trajectory-tracking/longitudinal-control/pid.md)
    - [3.2 Lateral Control](./book/3-trajectory-tracking/lateral-control/content.md)
        - [3.2.1 Basic Concepts](./book/3-trajectory-tracking/lateral-control/basics.md)
        - [3.2.2 Pure Pursuit Controller](./book/3-trajectory-tracking/lateral-control/pure-pursuit.md)
        - [3.2.3 Stanley Controller](./book/3-trajectory-tracking/lateral-control/stanley.md)
        - [3.2.4 LQR Controller](./book/3-trajectory-tracking/lateral-control/lqr.md)
        - [3.2.5 MPC Controller](./book/3-trajectory-tracking/lateral-control/mpc.md)
    - Combined Control
        - MPC Controller
- Motion Planning
    - Route Planning
    - Behavior Planning
    - Motion Planning
- Motion Prediction
    - Sensor Fusion
    - Multi-target Tracking
    - Motion Prediction
- Simultenanous Localization and Mapping (SLAM)
    - Localization
    - Mapping
    - SLAM
- Perception
    - Camera 3D Object Detection
    - Lidar 3D Object Detection
    - Multi-sensor Object Detection

## How to Start

1. Run Carla server in docker. Make sure you have docker installed. Direct to the root folder of this repo and run the following command in a terminal:

    For Ubuntu,

    ```bash
        bash ./run_server/run_carla_0.9.14.sh
    ```

    For Windows,

    ```shell
        ./run_server/run_carla_0.9.14.bat
    ```

2. Open a new terminal to run Carla client:

    For Ubuntu,

    ```bash
    source setup_pythonpath.sh 
    cd ./book/3-trajectory-following
    python test_lateral_control.py
    ```
