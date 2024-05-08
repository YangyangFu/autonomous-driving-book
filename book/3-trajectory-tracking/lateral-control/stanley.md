# 3.3 Stanley Controller

I found the explanation of the stanley controller is a mess in the internet, and hard to understand. 
Generally, the stanley control law has two general format in the literature:

$$
\delta = \theta_e + arctan(\frac{k d_e}{v_f})
$$

or 

$$
\delta = -\theta_e - arctan(\frac{k d_e}{v_f})
$$

This mess is due to the definition of some variables. 

**METHOD 1**

some definitions:
- $d_e$ is the cross-track error, which is positive if the vehicle is on the left side of the path, and negative if the vehicle is on the right side of the path. 
- $v_f$ is the forward velocity of the vehicle.
- $\theta_f$ is the yaw angle of the vehicle front wheels.
- $\theta$ is the yaw angle of the vehicle.
- $\delta$ is the **steer angle**, and $\delta = \theta_f - \theta$. 
    - if $\delta$ is positive, the vehicle is steered left
    - if $\delta$ is negative, the vehicle is steered right
    - **NOTE: typically we use negative for steering left, and positive for steering right. In this case, we do the opposite**
- $\theta_e$ is the yaw angle of the vehicle relative to the nearest path segment, $\theta_e = \theta - \theta_g$
- $k$ is stanley gain, positive 

When the vehicle is at the left side of the path as shown in the above figure, we have
- $\theta > 0$
- $\theta_g > 0$
- $\theta_e > 0$
- $\delta < 0$
- $d_e > 0$


The change rate of cross-track error can be shown as:

$$
\dot d_e = v_fsin(\delta+\theta_e)
$$

To design a expontial converging controller, we have:

$$
\dot d_e = -k d_e
$$

This gives:
- if the vehicle is on the left side of the path, the error is positive, and will expontially decay to 0
- if the vehicle is on the right side of the path, the error is negative, and will expontially converge to 0

Combining the above two equations, we have:

$$
    v_fsin(\delta+\theta_e) = -k d_e
$$

Therefore, 

$$
\delta = -arcsin(\frac{k d_e}{v_f}) - \theta_e
$$

Thus, when the vehicle is on the left side of a path, the steer command should be negative to turn the vehicle right.


When the vehicle is on the right side of a path as shown in above figure, we have:
- $\theta > 0$
- $\theta_g > 0$
- $\theta_e < 0$
- $\delta > 0$
- $d_e < 0$

We can reach the same steer control policy using the above equations.


**METHOD 2**

The Stanley paper uses the following definitions to derive the control policy:
- $\delta$ is the steer angle, and $\delta = \theta - \theta_f$. 
    - negative for steering left 
    - positive for steering right
    - **Note: this is a conventional definition**


When vehicle is on the left side of a path as shown in the above figure, we have:
- $\theta > 0$
- $\theta_g > 0$
- $\theta_e > 0$
- $\delta > 0$
- $d_e > 0$

The change rate of cross-track error can be shown as:

$$
\dot d_e = v_fsin(\theta_e - \delta)
$$

To design a expontial converging controller, we have:

$$
\dot d_e = -k d_e
$$

After rearranging, we have:

$$
\delta = arcsin(\frac{k d_e}{v_f}) + \theta_e
$$

Thus, when the vehicle is on the left side of a path, the steer command should be positive to turn the vehicle right.

We can have the same control policy when the vehicle is on the right side of the path.


**NOTES**

Therefore, the key here is the definition of steer angle.
- if the environment uses negative for steering left, and positive for steering right, then choose `METHOD 2`
- if the environment uses positive for steering left, and negative for steering right, then choose `METHOD 1`


Because $sin()$ is bound to $[-1, 1]$, to aovid easy saturation, the steering angle can be rewritten as:

$$
\delta = arctan(\frac{k d_e}{v_f}) + \theta_e
$$

For small steering, the above can still maintain a local exponential convergence. 

To avoid oversensitivity to the cross-track error, the steering angle can be further modified as:

$$
\delta = arctan(\frac{k d_e}{k_v + v_f}) + \theta_e
$$

where $k_v$ is a soft gain to tune the sensitivity of the controller to the cross-track error. The stanley paper suggests $k_v = 1 m/s$.

By adding the physical constraints of maximum steering angle, the steering angle can be further constrained to $[-\delta_{max}, \delta_{max}]$.


**Left-handed System**
Note the above derivation is based on the right-handed coordinate system. If the left-handed coordinate system is used, the transition between the two systems can be done by:

```text
RHS = LHS
```

For position,
```text
x_RHS = x_LHS
y_RHS = -y_LHS
z_RHS = z_LHS
```

For rotation,
```text
roll_RHS = -roll_LHS
pitch_RHS = pitch_LHS
yaw_RHS = -yaw_LHS
```