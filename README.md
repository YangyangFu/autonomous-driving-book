# Autonoumous Driving: Beginner to Master

This book aims to help beginners to learn autonomous driving from coding excercises. 
The content is organized in a bottom-up approach, by introducing local longitudinal and lateral PID controller first, and all the way up to perception.
All topics are demosntrated through closed-loop simulation using CARLA.
I found this approach is easier for students in mechanical/electrical engineering to start with.

The covered topics are:

```text
- Introduction
- Vehicle Dynamics and Control
    - Kinematic Bycle Model
    - Dynamics Model
- Path Tracking 
    - PID Controllers
        - Longitudinal Control 
        - Lateral Control
    - MPC Controller
    - DRL Controller
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
```


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


## Contribution

If you are interested in any of the above topics, please feel free to contact me at fuyy2008@gmail.com
