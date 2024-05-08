## 2.1. Kinematics Bicycle Model

#### 2.1.1  Rear Wheel Reference

How to steer vehicle if we know the curvature of the road?

- if the road is straight, the steering angle is 0
- if the road is curved with a constant curvature `k`, we can use the Ackermann steering model to calculate the steering angle $\delta$ of the vehicle.

$$
\delta = arctan(kL)
$$

where `L` is the distance between the front and rear axles of the vehicle. This model assumes that the vehicle is a bicycle model, where the front wheels are steered and the rear wheels are fixed. The desired point is at the center of the rear wheels.
