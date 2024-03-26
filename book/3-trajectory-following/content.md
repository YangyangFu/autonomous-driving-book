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

### 3.2.1 Basics

#### 3.2.1.1 Lateral Control Errors



### 3.2.2 Pure Pursuit Controller

Pure pursuit is a tracking algorithm that works by calculating the curvature that will move a vehicle from its current position to a goal position. The goal position is a point on the path that is a lookahead distance away from the vehicle. The curvature is then used to calculate the steering angle of the vehicle.

Since the vehicle has already deviated from the path, the controller has to guide the vehicle back to the path through a new `path` or arc. The controller calculates the steering angle based on the curvature of the arc and the lookahead distance.


**Inertia Frame**

Assuming the desired point is the center of rear wheels, Pure pursuit algorithm in the inertial frame is shown as below.

$$
R = \frac{L_d}{2sin(\alpha)}
$$

Thus, the curvature of the arc is:

$$
k = \frac{1}{R} = \frac{2 sin(\alpha)}{L_d}
$$

To follow the arc, using kinematic bicyle model, the steering angle $\delta$ is:

$$
\delta = arctan(kL)
$$

where $L$ is the distance between the front and rear axles of the car.
Combining the above two equations, we get:

$$
\delta = arctan(\frac{2Lsin(\alpha)}{L_d})
$$


The angle between the vehicle heading and the path $\alpha$ can be calculated as:

$$
\alpha + \theta = arctan(\frac{y_g - y_r}{x_g - x_r})
$$

where $(x_g, y_g)$ is the goal point coordinates, and $(x_r, y_r)$ is the current pose of the vehicle rear wheels, $\theta$ is the yaw angle or heading angle of the vehicle.

QUESTIONS:

- what is the default direction of steering? negative for left, positive for right?


**Procedure**

if using vehicle frame, 
- find a new pose ($x_r, y_r, \theta_r$) of the car
- find the path point cloest to the car: it is possible that there multiple points one lookahead distance away from the car, in this case, the closest one (the one with least distance along the path) is chosen.
- transform the goal point to the vehicle frame
- go towards that waypoint with calculated steering angle
- localize the car with the new pose

**Tuning**

the look ahead distance $L_d$ is a parameter in the algorithm.
- A smaller $L_d$ leads to more aggressive maneuvering to follow a closer arc, and closer arcs can work against the dynamic limits of the car
- A larger $L_d$ leads to smoother maneuvering, but the car may not follow the path as closely, thus, leading to higher tracking errors.


The lookahead distance is usually chosen to be a function of the speed of the vehicle, so that $\omega$ will not become more sensitive to $\alpha$ when $v_r$ is higher. The higher the speed, the higher the lookahead distance. This is because at higher speeds, the vehicle will cover more distance in the time it takes to react to the path. Thus, the lookahead distance should be higher to ensure that the vehicle has enough time to react to the path.

**Notes**

If crosstrack error (e) is defined here as lateral distance between the heading vector and the goal point, then

$$
sin \alpha = \frac{e}{L_d}
$$

Thus, the steering angle is:

$$
\delta = arctan(\frac{2Lsin(\alpha)}{L_d}) = arctan(\frac{2Le}{L_d^2})
$$

- Pure pursuit is a porportional controller.
- The proportional gain $\frac{2L}{L_d^2}$ can be tuned at different speeds by creating a relationship between the speed and the lookahead distance. 

$$
L_d = k_v v_r
$$

$$
\delta = arctan(\frac{2Lsin \alpha}{k_vv_r})
$$

#### 3.2.3 Stanley Controller

The Stanley controller is a tracking algorithm that works by calculating the cross-track error (CTE) and the heading error of the vehicle. The controller then computes the steering angle of the vehicle to minimize the CTE and heading error.



### 3.2.4 PID Controller

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

