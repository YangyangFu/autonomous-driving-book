from typing import Tuple, List
import numpy as np
from scipy.integrate import simpson 
import carla

from agents.tools.misc import normalize_rad_angle
from agents.navigation.trajectory import Trajectory, RoadPoint

class CubicSpiral:
    """
    Cubic spiral generator between given start point and end point

    Model description:
    x(s) = x_0 + \int_0^s cos(theta(s')) ds'
    y(s) = y_0 + \int_0^s sin(theta(s')) ds'

    theta(s) = theta_0 + a0*s + a1/2*s^2 + a2/3*s^3 + a3/4*s^4
    kappa(s) = a0 + a1*s + a2*s^2 + a3*s^3

    Solver boundary shooting problem with Newton-Raphson method

    """
    def __init__(self):
        self.simpson_size = 9
        self.p_params = np.zeros((4,1))
        self.a_params = np.zeros((4,1))
        self.s_g = -1.0
        
        # newton raphson parameters
        self.tolerance = 1e-6
        self.max_iter = 100

    def generate_spiral(self, start: carla.Transform, end: carla.Transform) -> Tuple[bool, np.array, float]:
        """
        Generate cubic spiral between start and end points
        """
        # starting point
        x_s = start.location.x 
        y_s = start.location.y
        th_s = normalize_rad_angle(np.radians(start.rotation.yaw)) 
        k_s = self._estimate_waypoint_curvature(start)

        self.start = RoadPoint(x_s, y_s, th_s, k_s)

        # ending point
        x_e = end.location.x
        y_e = end.location.y
        th_e = normalize_rad_angle(np.radians(end.rotation.yaw))
        k_e = self._estimate_waypoint_curvature(end)

        # transform to start point frame
        x_t = x_e - x_s
        y_t = y_e - y_s

        x_g = x_t * np.cos(th_s) + y_t * np.sin(th_s)
        y_g = -x_t * np.sin(th_s) + y_t * np.cos(th_s)
        th_g = normalize_rad_angle(th_e - th_s) 
        k_g = k_e
        
        # target state
        state_g = np.zeros((4,1))
        state_g[0,0] = x_g
        state_g[1,0] = y_g
        state_g[2,0] = th_g
        state_g[3,0] = k_g

        # initial guess of s_g, will be updated by Newton-Raphson method
        s_g = (th_g * th_g / 5.0 + 1.0) * np.sqrt(x_g * x_g + y_g * y_g)

        # parameters 
        p = np.zeros((4,1))
        p[0,0] = k_s
        p[3,0] = k_g

        # Newton-Raphson method
        error = 1e6 * np.ones((4,1))

        iter = 0
        while iter < self.max_iter and np.linalg.norm(error) > self.tolerance:

            # update error
            state_g_hat = self.update_goal_state(*p, s_g)
            error = state_g - state_g_hat

            # compute jacobian
            jacobian = self.update_jacobian(*p, s_g)

            # update delta_p
            delta_p = np.linalg.inv(jacobian) @ error

            # update p
            p[1,0] += delta_p[0,0]
            p[2,0] += delta_p[1,0]
            p[3,0] += delta_p[2,0]
            s_g += delta_p[3,0]
            
            iter += 1
        
        # log convergence
        success = np.linalg.norm(error) <= self.tolerance
        if success:
            print("Newton-Raphson method converged in {} iterations".format(iter))
        else:
            print("Newton-Raphson method did not converge")
        
        # save results
        self.p_params = p
        self.a_params = self.p_to_a(*p, s_g)
        self.s_g = s_g

        return success

    def p_to_a(self, p0, p1, p2, p3, sg):
        """
        Convert p to a
        """
        sg_inv = 1.0 / sg

        a = np.zeros((4,1))
        a[0,0] = p0
        a[1,0] = - (11*p0 - 18*p1 + 9*p2 - 2*p3) / 2.0 * sg_inv
        a[2,0] = 9*(2*p0 - 5*p1 + 4*p2 - p3) / 2.0 * sg_inv**2
        a[3,0] = -9*(p0 - 3*p1 + 3*p2 - p3) / 2.0 * sg_inv**3

        return a

    def partial_a_p(self, p0, p1, p2, p3, sg):
        """
        Partial derivative of a with respect to p

        [[0, 0, 0, 0],
        [9/sg, -9/(2*sg), 1/sg, -(11*p0 - 18*p1+9*p2-2*p3)/2*(-1/sg**2)],
        [-45/(2*sg**2), 36/(2*sg**2), -9/(2*s_g**2), 9*(2*p0-5*p1+4*p2-p3)/2*(-2/sg**3)],
        [27/(2*sg**3), -27/(2*sg**3), 9/(2*sg**3), -9*(p0-3*p1+3*p2-p3)/2*(-3/sg**4)]]
        
        """
        sg_inv = 1.0 / sg

        da_dp = np.zeros((4,4))
        da_dp[1,0] = 9.0 * sg_inv
        da_dp[1,1] = -9.0 / 2.0 * sg_inv
        da_dp[1,2] = sg_inv
        da_dp[1,3] = -(11*p0 - 18*p1 + 9*p2 - 2*p3) / 2.0 * (- sg_inv**2)

        da_dp[2,0] = -45.0 / 2.0 * sg_inv**2
        da_dp[2,1] = 36.0 / 2.0 * sg_inv**2
        da_dp[2,2] = -9.0 / 2.0 * sg_inv**2
        da_dp[2,3] = 9.0 * (2*p0 - 5*p1 + 4*p2 - p3) / 2.0 * (-2 * sg_inv**3)

        da_dp[3,0] = 27.0 / 2.0 * sg_inv**3
        da_dp[3,1] = -27.0 / 2.0 * sg_inv**3
        da_dp[3,2] = 9.0 / 2.0 * sg_inv**3
        da_dp[3,3] = -9.0 * (p0 - 3*p1 + 3*p2 - p3) / 2.0 * (-3 * sg_inv**4)

        return da_dp
    
    def partial_kappa_a(self, s):
        """
        Partial derivative of kappa with respect to a

        """
        dk_da = np.zeros((4,1))
        dk_da[0,0] = 1.0
        dk_da[1,0] = s 
        dk_da[2,0] = s**2
        dk_da[3,0] = s**3

        return dk_da

    def partial_kappa_p(self, p0, p1, p2, p3, sg, s):
        """
        Partial derivative of kappa with respect to p

        $$
        \frac{\partial \kappa}{\partial p} = \frac{\partial \kappa}{\partial a} \frac{\partial a}{\partial p} 
        $$

        """

        dk_da = self.partial_kappa_a(s)
        da_dp = self.partial_a_p(p0, p1, p2, p3, sg)

        dk_dp = dk_da.T @ da_dp
        
        return dk_dp

    def partial_theta_a(self, s):
        """
        Partial derivative of theta with respect to a

        $$
        \theta(s) = a0*s + a1/2*s^2 + a2/3*s^3 + a3/4*s^4
        $$

        """
        dth = np.zeros((4,1))
        dth[0,0] = s
        dth[1,0] = s**2 / 2.0
        dth[2,0] = s**3 / 3.0
        dth[3,0] = s**4 / 4.0

        return dth
    
    def partial_theta_p(self, p0, p1, p2, p3, sg, s):
        """
        Partial derivative of theta with respect to p

        $$
        \frac{\partial \theta}{\partial p} = \frac{\partial \theta}{\partial a} \frac{\partial a}{\partial p} 
        """

        dth_da = self.partial_theta_a(s)
        da_dp = self.partial_a_p(p0, p1, p2, p3, sg)

        dth_dp = dth_da.T @ da_dp

        return dth_dp
    
    def partial_xy_p(self, p0, p1, p2, p3, sg, s):

        # s to intervals
        ds = s / (self.simpson_size - 1) 

        s_points = [i * ds for i in range(self.simpson_size)]
        a = self.p_to_a(p0, p1, p2, p3, sg)

        th_points = [self.theta(a, s) for s in s_points]
        dth_dp_points = [self.partial_theta_p(p0, p1, p2, p3, sg, s) for s in s_points]

        # for x
        sin_th_points = [np.sin(th) for th in th_points]
        sin_th_dth_dp = [sin_th * dth_dp for sin_th, dth_dp in zip(sin_th_points, dth_dp_points)]

        # for y 
        cos_th_points = [np.cos(th) for th in th_points]
        cos_th_dth_dp = [cos_th * dth_dp for cos_th, dth_dp in zip(cos_th_points, dth_dp_points)]
        
        # simpson integration
        dx_dp = -simpson(sin_th_dth_dp, s_points)
        dy_dp = simpson(cos_th_dth_dp, s_points)

        return dx_dp, dy_dp

    def theta(self, a, s):
        """
        Theta function
        """
        return a[0] * s + a[1] / 2.0 * s**2 + a[2] / 3.0 * s**3 + a[3] / 4.0 * s**4

    def kappa(self, a, s):
        """
        Kappa function
        """
        return a[0] + a[1] * s + a[2] * s**2 + a[3] * s**3
    
    def xy(self, a, s):
        """
        get x and y coordinates using simpson integration
        """
        ds = s / (self.simpson_size - 1)
        s_points = [i * ds for i in range(self.simpson_size)]
        th_points = [self.theta(a, s) for s in s_points]

        cos_th_points = [np.cos(th) for th in th_points]
        sin_th_points = [np.sin(th) for th in th_points]

        x = simpson(cos_th_points, s_points)
        y = simpson(sin_th_points, s_points)

        return x, y

    def update_goal_state(self, p0, p1, p2, p3, sg):
        """
        Update goal state
        """
        # initialize states
        state_g = np.zeros((4,1))

        # p to a
        a = self.p_to_a(p0, p1, p2, p3, sg)

        # get goal states
        kappa_g = self.kappa(a, sg)
        theta_g = self.theta(a, sg)
        x_g, y_g = self.xy(a, sg)

        state_g[0,0] = x_g
        state_g[1,0] = y_g
        state_g[2,0] = theta_g
        state_g[3,0] = kappa_g

        return state_g 
    
    def update_jacobian(self, p0, p1, p2, p3, sg):
        """
        Update jacobian
        """
        # initialize jacobian
        jacobian = np.zeros((4,4))

        # partial derivative of x and y with respect to p
        dx_dp, dy_dp = self.partial_xy_p(p0, p1, p2, p3, sg, sg)

        # partial derivative of theta with respect to p
        dth_dp = self.partial_theta_p(p0, p1, p2, p3, sg, sg)

        # partial derivative of kappa with respect to p
        dk_dp = self.partial_kappa_p(p0, p1, p2, p3, sg, sg)

        jacobian[0,:] = dx_dp
        jacobian[1,:] = dy_dp
        jacobian[2,:] = dth_dp
        jacobian[3,:] = dk_dp

        return jacobian

    def _estimate_waypoint_curvature(self, waypoint, distance = 0.2):
        """
        Estimate waypoint curvature based on HD map
        """
        pre_waypoint = waypoint.next(distance)[-1] # this is a hard-coded value.

        yaw = np.radians(waypoint.transform.rotation.yaw)
        pre_yaw = np.radians(pre_waypoint.transform.rotation.yaw)

        dx = waypoint.transform.location.x - pre_waypoint.transform.location.x
        dy = waypoint.transform.location.y - pre_waypoint.transform.location.y

        curvature = 2 * np.sin((yaw - pre_yaw) / 2) / np.sqrt(dx**2 + dy**2)

        return curvature 
    
    def get_sampled_spiral(self, num_samples) -> Trajectory:
        """
        Get sampled spirals
        """
        if num_samples < 2:
            raise ValueError("Number of samples should be greater than 1")

        ds =  self.s_g / (num_samples - 1)

        s_points = [i * ds for i in range(num_samples)]
        kappa_points = [self.kappa(self.a_params, s) for s in s_points]
        theta_points = [self.theta(self.a_params, s) for s in s_points]

        # here we use trapezoidal integration instead of simpson integration to save computation time
        x_points = [0.0] * num_samples
        y_points = [0.0] * num_samples

        for i in range(1, num_samples):
            x_points[i] = x_points[i-1] + ds * (np.cos(theta_points[i]) + np.cos(theta_points[i-1])) / 2
            y_points[i] = y_points[i-1] + ds * (np.sin(theta_points[i]) + np.sin(theta_points[i-1])) / 2


        # to global frame
        x_points = [x + self.start.x for x in x_points]
        y_points = [y + self.start.y for y in y_points]
        theta_points = [normalize_rad_angle(theta + self.start.theta) for theta in theta_points]

        # output trajectory
        traj = Trajectory(x_points, y_points, theta_points, kappa_points)

        return traj

    
