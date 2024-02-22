# 3. Trajectory Following

In this chapter, we will learn how to make a vehicle follow a predefined trajectory. We will start by creating a trajectory using waypoints and then implement a controller to make the vehicle follow the trajectory. We will also learn how to visualize the trajectory and the vehicle's path in the CARLA simulator.


## 3.1 Trajectory Generation

Trajectory vs Path:

- A trajectory is a sequence of states (position, velocity, and acceleration) of a vehicle over time. This typically is generated from a motion planner or a trajectory optimization algorithm.
- A path is a sequence of waypoints that define the position of the vehicle over time.

For simplicity in this chapter, we will use the terms trajectory and path interchangeably, which means we will use a sequence of waypoints to define the trajectory of the vehicle.

The map is assumed to be perfectly known, and the trajectory can then be generated from a minimum distance graph search algorithm, such as A* or Dijkstra's algorithm. 

## 3.2 Trajectory Following

Trajectory following is the process of making a vehicle follow a predefined trajectory. This is achieved by implementing a controller that computes the steering, throttle, and brake commands to make the vehicle follow the trajectory. The controller can be a simple PID controller or a more complex model predictive controller (MPC).

### 3.2.1 PID Controller

**Longitudinal Control**: this controller is used to control the speed of the vehicle. It computes the throttle and brake commands to make the vehicle follow the desired speed. The throttle is within [-1, 1], where -1 is full brake and 1 is full throttle.

```python
throttle = PID(speed, desired_speed)
```

**Lateral Control**: this controller is used to control the steering of the vehicle. It computes the steering command to make the vehicle follow the desired trajectory. The steering is within [-1, 1], where -1 is full left and 1 is full right.


The angle between vehicle forward vector and waypoint forward vector is used to measure the error.
Following right-hand coordinate system, the cross product of the vehicle forward vector and the waypoint forward vector is used to determine the sign of the error.


**TODO: need work on this**

If the angle is within `[0, pi]`, the cross vector points positive direction. The vehicle is on the right side of the waypoint, when the error gets larger, more steering torward left is needed. 
If the angle is within `[pi, 2*pi]`, which means the vehicle is on the left side of the waypoint, the error is negative.


#### 3.2.1.1 PID Controller Tuning


### 3.2.2 Model Predictive Controller (MPC)

