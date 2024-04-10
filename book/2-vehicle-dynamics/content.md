# 2. Vehicle Dynamics

## 2.1. Kinematics Bicycle Model

#### 2.1.1  Rear Wheel Reference

How to steer vehicle if we know the curvature of the road?

- if the road is straight, the steering angle is 0
- if the road is curved with a constant curvature `k`, we can use the Ackermann steering model to calculate the steering angle $\delta$ of the vehicle.

$$
\delta = arctan(kL)
$$

where `L` is the distance between the front and rear axles of the vehicle. This model assumes that the vehicle is a bicycle model, where the front wheels are steered and the rear wheels are fixed. The desired point is at the center of the rear wheels.

## 2.2 Dynamic Bicycle Model

Let $\theta$ be the yaw angle of the vehicle, $r$ be the yaw rate. 

$$
r = \dot \theta
$$

$$
\dot v_y = \frac{-c_f [\tan^{-1}(\frac{v_f + l_fr}{v_x}) - \delta] \cos(\delta) - c_r \tan^{-1}(\frac{v_y - l_rr}{v_x})}{m} - v_x r \\
\dot r = \frac{-l_fc_f [\tan^{-1}(\frac{v_y + l_fr}{v_x}) - \delta] + l_rc_r \tan^{-1}(\frac{v_y - l_rr}{v_x})}{I_z}
$$


The above model is nonlinear, and to apply linear control methods, the model must be linearized by using small angle assumptions.

$$
\begin{align}
\dot v_y &= \frac{-c_f [\frac{v_f + l_fr}{v_x} - \delta] - c_r \frac{v_y - l_rr}{v_x}}{m} - v_x r \\

&= \frac{-c_fv_y-c_fl_fr}{mv_x} + \frac{c_f\delta}{m} + \frac{-c_rv_y+c_rl_rr}{mv_x} - v_xr \\

&= \frac{-(c_f+c_r)}{mv_x}v_y + [\frac{l_rc_r - l_fc_f}{mv_x} - v_x] r + \frac{c_f}{m}\delta

\end{align}
$$

$$
\begin{align}
\dot r &= \frac{-l_fc_fv_y-l_f^2c_fr}{I_zv_x} + \frac{l_fc_f\delta}{I_z} + \frac{l_rc_rv_y - l_r^2c_rr}{I_zv_x} \\

&= \frac{l_rc_r - l_fc_f}{I_zv_x}v_y + \frac{-(l_f^2c_f + l_r^2c_r)}{I_zv_x}r + \frac{l_fc_f}{m}\delta

\end{align}
$$


The state-space format of the linearized model is:

$$
\begin{bmatrix}
\dot v_y \\
\dot r
\end{bmatrix}
= 
\begin{bmatrix}
\frac{-(c_f+c_r)}{mv_x} & \frac{l_rc_r - l_fc_f}{mv_x} - v_x \\
\frac{l_rc_r - l_fc_f}{I_zv_x} & \frac{-(l_f^2c_f + l_r^2c_r)}{I_zv_x}
\end{bmatrix} 
\begin{bmatrix}
v_y \\
r
\end{bmatrix}
+
\begin{bmatrix}
\frac{c_f}{m} \\
\frac{l_fc_f}{m}
\end{bmatrix}
\delta
$$

For control design, it is usefull to express the model with respect to the path, or reference.
With constant longitudinal velocity assumption, the desired yaw rate from the path is defined as:

$$
r(s) = k(s)v_x
$$

The desired lateral acceleration follows as:

$$
\dot v_y(s) = v_x r(s) = k(s)v_x^2
$$

Denote the orthogonal distance of the CG to the path as `e`,and the heading error as $\theta_e = \theta - \theta(s)$, and we have

$$
\begin{align}
\ddot e &= \dot v_y + v_x r - \dot v_y(s) \\
&= \dot v_y + v_x(r - r(s)) \\
&= \dot v_y + v_x \dot \theta_e
\end{align}
$$


$$
\dot e = v_y + v_x \sin(\theta_e)
$$

Substituting the above error terms into the linearized model, we have the following state-space model for tracking error variables:

$$
\begin{bmatrix}
\dot e \\
\ddot e \\
\dot \theta_e \\
\ddot \theta_e
\end{bmatrix}
= 
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & \frac{-(c_f+c_r)}{mv_x} & \frac{c_f+c_r}{m} & \frac{l_rc_r-l_fc_f}{mv_x} \\
0 & 0 & 0 & 1 \\
0 & \frac{l_rc_r - l_fc_f}{I_zv_x} & \frac{l_fc_f-l_rc_r}{I_z} & \frac{-(l_f^2c_f + l_r^2c_r)}{I_zv_x} 
\end{bmatrix}
\begin{bmatrix}
e \\
\dot e \\
\theta_e \\
\dot \theta_e
\end{bmatrix}
+ 
\begin{bmatrix}
0 \\
\frac{c_f}{m} \\
0 \\
\frac{l_fc_f}{I_z}
\end{bmatrix}
\delta
+ 
\begin{bmatrix}
0 \\
\frac{l_rc_r-l_fc_f}{mv_x} -v_x \\
0 \\
\frac{-(l_f^2c_f+l_r^2c_r)}{I_zv_x}
\end{bmatrix}
r(s)
$$


## 2.3 Bilinear Discretization

The continuous-time model can be discretized using the bilinear transformation method. Denote the continuous-time model as:

$$
\dot x = Ax + Bu + w \\
y = Cx 
$$

Applying integration to the continuous-time model, we have:

$$
x(t+\Delta t) = x(t) + \int_t^{t+\Delta t} Ax(\tau) + Bu(\tau) d\tau + \int_t^{t+\Delta t} w(\tau) d\tau
$$

Assuming the input and disturbance are constant over the interval, we have:

$$
x(t+\Delta t) = x(t) + \int_t^{t+\Delta t}A x(\tau)d\tau + Bu\Delta t + w \Delta t
$$

The integral term can be approximated by the midpoint rule:

$$
\int_t^{t+\Delta t}A x(\tau)d\tau \approx A\frac{x(t)+x(t+\Delta t)}{2}\Delta t
$$

Substitute the above approximation into the discretized model, we have:

$$
x(t+\Delta t) = x(t) + \frac{A(x(t)+x(t+\Delta t))}{2}\Delta t + Bu\Delta t + w \Delta t
$$ 

Rearranging the terms, we have:

$$
x(t+\Delta t) = (I - \frac{A\Delta t}{2})^{-1}(I + \frac{A\Delta t}{2})x(t) + (I - \frac{A\Delta t}{2})^{-1}Bu\Delta t + (I - \frac{A\Delta t}{2})^{-1}w\Delta t \\

y(t) = Cx(t)
$$

The above equation is the discretized model of the continuous-time model. The discretized model can be used for control design and simulation.




