
## 3.1 Trajectory Generation

Trajectory vs Path:

- A trajectory is a sequence of states (position, velocity, and acceleration) of a vehicle over time. This typically is generated from a motion planner or a trajectory optimization algorithm.
- A path is a sequence of waypoints that define the position of the vehicle over time.


### 3.1.1 Cubic Spiral Trajectory
The problem to solve here is given a start waypoint $(x_s, y_s, \theta_s, \kappa_s)$ and a goal waypoint $(x_g, y_g, \theta_g, \kappa_g)$, generate a cubic spiral trajectory that connects the two waypoints, and the sample a set of waypoints along the trajectory as the reference trajectory.

$$
\kappa (s) = a_0 + a_1s + a_2s^2 + a_3s^3
$$

Based on simple geometric and kinematic constraints, we have the following equations using the start point as the origin::

$$
\begin{align}
x(s) &= \int_0^s \cos(\theta(s'))ds' \\
y(s) &= \int_0^s \sin(\theta(s'))ds' \\
\theta(s) &= \int_0^s \kappa(s')ds = a_0s + \frac{1}{2}a_1s^2 + \frac{1}{3}a_2s^3 + \frac{1}{4}a_3s^4\\
\kappa(s) &= a_0 + a_1s + a_2s^2 + a_3s^3 
\end{align}
$$

The parameter $\bold{a} = [a_0, a_1, a_2, a_3]$ are determined by the boundary conditions at the start and goal waypoints.

Directly solving the above root-finding problem is time consuming and not efficient. Instead, we can use the following steps to solve the problem:
instead of directing solving for $\bold{a}$, we introduce another set of parameter $\bold{p}=[p_0, p_1, p_2, p_3]$ so that 

$$
\begin{align}
    \kappa(0) = p_0 \\
    \kappa(\frac{1}{3}s_g) = p_1 \\
    \kappa(\frac{2}{3}s_g) = p_2 \\
    \kappa(s_g) = p_3
\end{align}
$$

In this way, if we solve for $\bold{p}$, we can get $a$ by solving a linear equation:

$$
\begin{align}
    a_0(\bold{p}) &= p_0 \\
    a_1(\bold{p}) &= -\frac{11p_0 - 18p_1+9p_2-2p_3}{2s_g} \\
    a_2(\bold{p}) &= \frac{9(2p_0-5p_1+4p_2-p_3)}{2s_g^2} \\
    a_3(\bold{p}) &= -\frac{9(p_0 - 3p_1 + 3p_2 - p_3)}{2s_g^3}
\end{align}
$$

Once we know $\bold{p} = [p_0, p_1, p_2, p_3]$ and $s_g$, we will know $\bold{a}$. 
Because $p_0$ and $p_3$ can be know from starting point and goal point, we need find $p_1, p_2, s_g$ to solve the root-finding problem.
The following derivative holds from kinematic constraints:

$$
\begin{align}
\frac{dx(\bold{p}, s)}{ds} &= \cos(\theta(\bold{p}, s)) \\
\frac{dy(\bold{p}, s)}{ds} &= \sin(\theta(\bold{p}, s)) \\
\frac{d\theta(\bold{p}, s)}{ds} &= \kappa(\bold{p}, s) 
\end{align}
$$


Let the state of the waypoint $\bold{x}(s) = [x(s), y(s),\theta(s)]$, the objective is to find the optimal $\bold{p}$ so that the end point of the cubic spiral can be as close as possible to the goal waypoint. The root-finding problem can be defined as:

$$
f(\bold{p}) = \bold{x}_g-\bold{x}(\bold{p}) = 0
$$

Using Newton-Raphson method, the optimal $\bold{p}$ can be found by iteratively solving the following equation:

$$
\begin{align}
\Delta \bold{x} &= \bold{x}_g - \bold{x}(\bold{p}_i, s_g) \\
\Delta \bold{p} &= \bold{J}(\bold{x}(\bold{p}_i))^{-1} \Delta \bold{x}\\
\bold{p}_{i+1} &= \bold{p}_i + \Delta \bold{p}
\end{align}
$$

The Jacobian matrix $\bold{J}$ is defined as:

$$
\bold{J}(\bold{x}(\bold{p}_i)) 
=
\begin{bmatrix}
\frac{\partial x(\bold{p}_i, s_g)}{\partial p_1} & \frac{\partial x(\bold{p}_i, s_g)}{\partial p_2} & \frac{\partial x(\bold{p}_i, s_g)}{\partial s_g} \\
\frac{\partial y(\bold{p}_i, s_g)}{\partial p_1} & \frac{\partial y(\bold{p}_i, s_g)}{\partial p_2} & \frac{\partial y(\bold{p}_i, s_g)}{\partial s_g} \\
\frac{\partial \theta(\bold{p}_i, s_g)}{\partial p_1} & \frac{\partial \theta(\bold{p}_i, s_g)}{\partial p_2} & \frac{\partial \theta(\bold{p}_i, s_g)}{\partial s_g} \\
\end{bmatrix}
$$

For notion completeness, we use the whole $\bold{p}$ vector in the Jacobian matrix:
$$
\bold{J}(\bold{x}(\bold{p}_i, s_g)) = [\frac{\partial \bold{x}(\bold{p}_i, s_g)}{\partial \bold{p}}, \frac{\partial \bold{x}(\bold{p}_i, s_g)}{\partial s_g}]
$$

This leads to a 3x5 matrix, but we need remove the fisrt column (related to $p_0$) and the forth column (related to $p_3$) to make the matrix 3x3 for the Newton-Raphson method.


Knowing the following method to calculate the derivatives for a definite integral:

$$
F(x) = \int_{v(x)}^{g(x)} f(t)dt
$$

$$
\frac{dF(x)}{dx} = f(g(x))g'(x) - f(v(x))v'(x)
$$

For 
$$
F(x, y) = \int_0^x f(t,y)dt
$$

The derivative can be calculated as:

$$
\begin{align}
\frac{dF(x,y)}{dx} &= \frac{\partial F(x,y)}{\partial x} + \frac{\partial F(x,y)}{\partial y}\frac{dy}{dx} \\
\frac{dF(x,y)}{dy} &= \frac{\partial F(x,y)}{\partial y} + \frac{\partial F(x,y)}{\partial x} \frac{dx}{dy}\\
\end{align}
$$

If $x$ and $y$ are independent, then $\frac{dx}{dy} = 0$, and $\frac{dy}{dx} = 0$.
Thus,

$$
\begin{align}
\frac{dF(x,y)}{dx} &= \frac{\partial F(x,y)}{\partial x} = f(x,y) \\
\frac{dF(x,y)}{dy} &= \frac{\partial F(x,y)}{\partial y} = \int_0^{x} \frac{\partial f(t,y)}{\partial y} dt\\
\end{align}
$$


If $x$ and $y$ are dependent, we can assume $y = y(x)$, then

$$
\begin{align}
\frac{dF(x,y)}{dx} &= \frac{\partial F(x,y)}{\partial x} + \frac{\partial F(x,y)}{\partial y}\frac{dy}{dx} = f(x,y) + \int_0^{x} \frac{\partial f(t,y)}{\partial y} dt \frac{dy}{dx} \\
\frac{dF(x,y)}{dy} &= \frac{\partial F(x,y)}{\partial y} + \frac{\partial F(x,y)}{\partial x} \frac{dx}{dy} = \int_0^{x} \frac{\partial f(t,y)}{\partial y} dt + f(x,y) \frac{dx}{dy}\\
\end{align}
$$


For the cubic spiral trajectory, the derivatives can be calculated as:


Based on the above folulation, the partial derivatives in the Jacobian matrix can be calculated as:
$$
\begin{align}
\frac{\partial \theta(\bold{p}_i, s)}{\partial \bold{p}} &= \frac{\partial \theta(\bold{p}_i, s)}{\partial \bold{a}} \frac{\partial \bold{a}}{\partial \bold{p}} \\
\frac{\partial \theta(\bold{p}_i, s)}{\partial s_g} &= \frac{\partial \theta(\bold{p}_i, s)}{\partial \bold{a}} \frac{\partial \bold{a}}{\partial s_g}\\
\frac{\partial y(\bold{p}_i, s)}{\partial \bold{p}} |_{s=s_g} &= \frac{\partial \int_0^{s_g} \sin(\theta(\bold p_i, s'))ds'}{\partial \bold{p}} = \int_0^{s_g} \frac{\partial \sin(\theta(\bold p_i, s'))}{\partial \bold{p}} ds'= \int_0^{s_g} \cos(\theta(\bold p_i, s'))\frac{\partial \theta(\bold p_i, s')}{\partial \bold{p}}ds' \\
\frac{\partial y(\bold{p}_i, s)}{\partial s_g} |_{s = s_g} &= \frac{\partial y(\bold p_i, s_g)}{\partial s_g} + \frac{\partial y(\bold p_i, s_g)}{\partial \bold p}\frac{\partial \bold p_i}{\partial s_g} 
= \sin(\theta(\bold p_i(s_g), s_g)) + \int_0^{s_g} \cos(\theta(\bold p_i(s_g), s'))\frac{\partial \theta(\bold p_i(s_g), s')}{\partial s_g}ds' \\
\frac{\partial x(\bold{p}_i, s)}{\partial \bold{p}} |_{s=s_g} &= \frac{\partial \int_0^{s_g} \cos(\theta(\bold p_i, s'))ds'}{\partial \bold{p}} = \int_0^{s_g} \frac{\partial \cos(\theta(\bold p_i, s'))}{\partial \bold{p}} ds'= -\int_0^{s_g} \sin(\theta(\bold p_i, s'))\frac{\partial \theta(\bold p_i, s')}{\partial \bold{p}}ds' \\
\frac{\partial x(\bold{p}_i, s)}{\partial s_g} |_{s = s_g} &= \frac{\partial x(\bold p_i, s_g)}{\partial s_g} + \frac{\partial x(\bold p_i, s_g)}{\partial \bold p}\frac{\partial \bold p_i}{\partial s_g} 
= \cos(\theta(\bold p_i(s_g), s_g)) - \int_0^{s_g} \sin(\theta(\bold p_i(s_g), s'))\frac{\partial \theta(\bold p_i(s_g), s')}{\partial s_g}ds' \\
\end{align}
$$

The integrals on the right-hand side can be solved using numerical integration methods, such as the trapezoidal rule or Simpson's rule.