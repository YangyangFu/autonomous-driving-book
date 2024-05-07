from typing import Tuple, List
import numpy as np
from scipy.integrate import simpson, trapezoid
from scipy import linalg 
import carla

from agents.tools.misc import normalize_rad_angle
from agents.navigation.trajectory import Path, PathPoint

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
    def __init__(self, simpson_size = 9, tolerance = 1e-2, max_iter = 100):
        self.simpson_size = simpson_size
        self.p_params = np.zeros(4)
        self.a_params = np.zeros(4)
        self.s_g = -1.0
        
        # newton raphson parameters
        self.tolerance = tolerance
        self.max_iter = max_iter

    def generate_spiral(self, start: carla.Waypoint, end: carla.Waypoint) -> Tuple[bool, np.array, float]:
        """
        Generate cubic spiral between start and end points
        """
        # starting point
        x_s = start.transform.location.x 
        y_s = start.transform.location.y
        th_s = normalize_rad_angle(np.radians(start.transform.rotation.yaw)) 
        k_s = self._estimate_waypoint_curvature(start)

        self.start = PathPoint(x_s, y_s, th_s, k_s)

        # ending point
        x_e = end.transform.location.x
        y_e = end.transform.location.y
        th_e = normalize_rad_angle(np.radians(end.transform.rotation.yaw))
        k_e = self._estimate_waypoint_curvature(end)

        # transform to start point frame
        x_t = x_e - x_s
        y_t = y_e - y_s

        x_g = x_t * np.cos(th_s) + y_t * np.sin(th_s)
        y_g = -x_t * np.sin(th_s) + y_t * np.cos(th_s)
        th_g = normalize_rad_angle(th_e - th_s) 
        k_g = k_e
        
        # target state
        state_g = np.zeros(3)
        state_g[0] = x_g
        state_g[1] = y_g
        state_g[2] = th_g
        #state_g[3,0] = k_g

        # initial guess of s_g, will be updated by Newton-Raphson method
        s_g = (th_g * th_g / 5.0 + 1.0) * np.sqrt(x_g * x_g + y_g * y_g)
        #s_g = np.sqrt(x_g * x_g + y_g * y_g)
        # parameters 
        p = np.zeros((4))
        p[0] = k_s
        p[3] = k_g

        

        # Newton-Raphson method
        error = 1e6 * np.ones((3,1))
        diff = np.abs(error).sum()

        iter = 0
        
        while iter < self.max_iter and diff > self.tolerance:

            # update error
            state_g_hat = self.update_goal_state(p, s_g)
            error = state_g - state_g_hat
            diff = np.abs(error).sum()

            # compute jacobian
            jacobian = self.update_jacobian(p, s_g)
            jacobian = jacobian[:, [1,2,4]]
            
            # update delta_p
            
            #delta_p = np.linalg.inv(jacobian) @ error
            delta_p = linalg.solve(jacobian, error.reshape(-1,1))

            # update p
            p[1] += delta_p[0,0]
            p[2] += delta_p[1,0]
            #p[3,0] += delta_p[2,0]
            s_g += delta_p[2,0]
            
            iter += 1

        # log convergence
        success = np.linalg.norm(error) <= self.tolerance
        if success:
            print("Newton-Raphson method converged in {} iterations: Sg: {:.2f}".format(iter, s_g))
        else:
            print("Newton-Raphson method did not converge: Sg: {:.2f}".format(s_g))
        
        # save results
        self.p_params = p
        self.a_params = self.p_to_a(p, s_g)
        self.s_g = s_g

        return success

    def p_to_a(self, p, sg):
        """
        Convert p to a
        """
        sg_inv = 1.0 / sg

        a = np.zeros(4)
        a[0] = p[0]
        a[1] = - (11*p[0] - 18*p[1] + 9*p[2] - 2*p[3]) / 2.0 * sg_inv
        a[2] = 9*(2*p[0] - 5*p[1] + 4*p[2] - p[3]) / 2.0 * sg_inv**2
        a[3] = -9*(p[0] - 3*p[1] + 3*p[2] - p[3]) / 2.0 * sg_inv**3

        return a

    def partial_a_p(self, sg):
        """
        Partial derivative of a with respect to p
        a = [a0, a1, a2, a3]
        p = [p0, p1, p2, p3]

        ::
        [[0, 0, 0, 0],
        [9/sg, -9/(2*sg), 1/sg, -(11*p0 - 18*p1+9*p2-2*p3)/2*(-1/sg**2)],
        [-45/(2*sg**2), 36/(2*sg**2), -9/(2*s_g**2), 9*(2*p0-5*p1+4*p2-p3)/2*(-2/sg**3)],
        [27/(2*sg**3), -27/(2*sg**3), 9/(2*sg**3), -9*(p0-3*p1+3*p2-p3)/2*(-3/sg**4)]]
        
        """
        sg_inv = 1.0 / sg

        da_dp = np.zeros((4,4))
        da_dp[0,0] = 1.0 # da0/dp0 

        da_dp[1,0] = -11.0 / 2.0 * sg_inv # da1/dp0
        da_dp[1,1] = 18.0 / 2.0 * sg_inv # da1/dp1
        da_dp[1,2] = -9.0 / 2.0 * sg_inv # da1/dp2
        da_dp[1,3] = 2.0 / 2.0 * sg_inv # da1/dp3

        da_dp[2,0] = 18.0 / 2.0 * sg_inv**2 # da2/dp0
        da_dp[2,1] = -45.0 / 2.0 * sg_inv**2 # da2/dp1
        da_dp[2,2] = 36.0 / 2.0 * sg_inv**2 # da2/dp2
        da_dp[2,3] = -9.0 / 2.0 * sg_inv**2 # da2/dp3

        da_dp[3,0] = -9.0 / 2.0 * sg_inv**3 # da3/dp0
        da_dp[3,1] = 27.0 / 2.0 * sg_inv**3
        da_dp[3,2] = -27.0 / 2.0 * sg_inv**3
        da_dp[3,3] = 9.0 / 2.0 * sg_inv**3

        return da_dp
    
    def partial_a_sg(self,p, sg):
        """
        Partial derivative of a with respect to sg

        ::
        [0,
        -(11*p0 - 18*p1+9*p2-2*p3)/2*(-1/sg**2),
        9*(2*p0-5*p1+4*p2-p3)/2*(-2/sg**3),
        -9*(p0-3*p1+3*p2-p3)/2*(-3/sg**4)]

        """
        p0, p1, p2, p3 = p
        sg_inv = 1.0 / sg
        da_ds = np.zeros(4)

        da_ds[0] = 0.0
        da_ds[1] = -(11*p0 - 18*p1 + 9*p2 - 2*p3) / 2.0 * (- sg_inv**2)
        da_ds[2] = 9.0 * (2*p0 - 5*p1 + 4*p2 - p3) / 2.0 * (-2 * sg_inv**3)
        da_ds[3] = -9.0 * (p0 - 3*p1 + 3*p2 - p3) / 2.0 * (-3 * sg_inv**4)

        return da_ds
    
    def partial_kappa_a(self, s):
        """
        Partial derivative of kappa with respect to a

        """
        dk_da = np.zeros((4))
        dk_da[0] = 1.0
        dk_da[1] = s 
        dk_da[2] = s**2
        dk_da[3] = s**3

        return dk_da

    def partial_theta_a(self, s):
        """
        Partial derivative of theta with respect to a

        $$
        \theta(s) = a0*s + a1/2*s^2 + a2/3*s^3 + a3/4*s^4
        $$

        """
        dth = np.zeros((4))
        dth[0] = s
        dth[1] = s**2 / 2.0
        dth[2] = s**3 / 3.0
        dth[3] = s**4 / 4.0

        return dth
    
    def partial_theta_p(self, sg, s) -> np.array:
        """
        Partial derivative of theta with respect to p

        $$
        \frac{\partial \theta}{\partial p} = \frac{\partial \theta}{\partial a} \frac{\partial a}{\partial p} 
        
        return: np.array of shape (1,4)

        """
        # (4,)
        dth_da = self.partial_theta_a(s)
        # (4,4)
        da_dp = self.partial_a_p(sg)

        # (4) = (1,4) @ (4,4)
        dth_dp = dth_da.reshape(1,-1) @ da_dp

        return dth_dp
    
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

        cos_th_points = np.cos(th_points).reshape(-1) #[np.cos(th) for th in th_points]
        sin_th_points = np.sin(th_points).reshape(-1) #[np.sin(th) for th in th_points]

        x = simpson(cos_th_points, x = s_points)
        y = simpson(sin_th_points, x = s_points)

        return x, y

    def update_goal_state(self, p, sg):
        """
        Update goal state
        """
        # initialize states
        state_g = np.zeros(3)

        # p to a
        a = self.p_to_a(p, sg)

        # get goal states
        #kappa_g = self.kappa(a, sg)
        theta_g = self.theta(a, sg)
        x_g, y_g = self.xy(a, sg)

        state_g[0] = x_g
        state_g[1] = y_g
        state_g[2] = theta_g
        #state_g[3,0] = kappa_g

        return state_g 
    
    def partial_dth_dsg(self, p, s, sg):
        """
        Partial derivative of theta with respect to s_g
        """
        # (4, )
        dth_da = self.partial_theta_a(s)
        # (4,)
        da_dsg = self.partial_a_sg(p, sg)

        # (1,) = (1,4) @ (4, 1)
        dth_dsg = dth_da.reshape(1,-1) @ da_dsg.reshape(-1, 1)
        
        return dth_dsg
    
    def update_jacobian(self, p, sg):
        """
        Update jacobian
        """
        # s to intervals
        ds = sg / (self.simpson_size - 1) 

        # (n,)
        s_points = [i * ds for i in range(self.simpson_size)]
        a = self.p_to_a(p, sg)

        # (n,)
        th_points = np.array([self.theta(a, s) for s in s_points])
        # (n, 4)
        dth_dp_points = np.vstack([self.partial_theta_p(sg, s) for s in s_points])
        # (n, )
        dth_dsg_points = np.array([self.partial_dth_dsg(p, s, sg) for s in s_points]).flatten()

        # for x, y
        sin_th_points = np.array([np.sin(th) for th in th_points]).reshape(-1)
        cos_th_points = np.array([np.cos(th) for th in th_points]).reshape(-1)

        # (n, 4) = (n, 1) * (n, 4)
        sin_th_dth_dp = sin_th_points.reshape(-1, 1) * dth_dp_points
        cos_th_dth_dp = cos_th_points.reshape(-1, 1) * dth_dp_points
        
        # (n, ) = (n, ) * (n, )
        sin_th_dth_dsg = sin_th_points * dth_dsg_points
        cos_th_dth_dsg = cos_th_points * dth_dsg_points

        # simpson integration
        # (4,)
        dx_dp = -simpson(sin_th_dth_dp.T, x = s_points)
        dy_dp = simpson(cos_th_dth_dp.T, x = s_points)
        # (1,)
        dx_dsg = cos_th_points[-1] - simpson(sin_th_dth_dsg, x = s_points)
        dy_dsg = sin_th_points[-1] + simpson(cos_th_dth_dsg, x = s_points)

        # jacobian
        jacobian = np.zeros((3,5))
        jacobian[0,:-1] = dx_dp
        jacobian[0,-1] = dx_dsg
        jacobian[1,:-1] = dy_dp
        jacobian[1,-1] = dy_dsg
        jacobian[2,:-1] = dth_dp_points[-1,:]
        jacobian[2,-1] = dth_dsg_points[-1]

        return jacobian

    def _estimate_waypoint_curvature(self, waypoint: carla.Waypoint, distance = 0.2):
        """
        Estimate waypoint curvature based on HD map
        """
        pre_waypoint = waypoint.previous(distance)[-1] # this is a hard-coded value.

        yaw = np.radians(waypoint.transform.rotation.yaw)
        pre_yaw = np.radians(pre_waypoint.transform.rotation.yaw)

        dx = waypoint.transform.location.x - pre_waypoint.transform.location.x
        dy = waypoint.transform.location.y - pre_waypoint.transform.location.y

        curvature = 2 * np.sin((yaw - pre_yaw) / 2) / np.sqrt(dx**2 + dy**2)

        return curvature 
    
    def get_sampled_trajectory(self, num_samples) -> Path:
        """
        Get sampled spirals
        """
        if num_samples < 2:
            raise ValueError("Number of samples should be greater than 1")

        ds =  self.s_g / (num_samples - 1)

        s_points = [i * ds for i in range(num_samples)]
        kappa_points = [self.kappa(self.a_params, s) for s in s_points]
        theta_points = [normalize_rad_angle(self.theta(self.a_params, s) + self.start.theta) for s in s_points]

        # iterative trapezoidal integration
        x_points = [0.0] * num_samples
        y_points = [0.0] * num_samples
        x_points[0] = self.start.x 
        y_points[0] = self.start.y


        dx = 0
        dy = 0

        for i in range(1, num_samples):
            dx = (dx / i) * (i-1) + (np.cos(theta_points[i]) + np.cos(theta_points[i-1])) / (2*i)
            dy = (dy / i) * (i-1) + (np.sin(theta_points[i]) + np.sin(theta_points[i-1])) / (2*i)
            x_points[i] = s_points[i] * dx + x_points[0]
            y_points[i] = s_points[i] * dy + y_points[0]
            
            # the following is not correct
            #x_points[i] = x_points[i-1] + ds * (np.cos(theta_points[i]) + np.cos(theta_points[i-1])) / 2
            #y_points[i] = y_points[i-1] + ds * (np.sin(theta_points[i]) + np.sin(theta_points[i-1])) / 2

        # output trajectory
        path = Path(x_points, y_points, theta_points, kappa_points, s_points)

        return path

    # TODO: this is not right
    def get_sampled_trajectory1(self, num_samples):

        ds = self.s_g / (num_samples - 1)
        s_points = [i * ds for i in range(num_samples)]
        kappa_points = [self.kappa(self.a_params, s) for s in s_points]
        theta_points = [normalize_rad_angle(self.theta(self.a_params, s)) for s in s_points]

        x_points = [0] * num_samples
        y_points = [0] * num_samples
        for i in range(1, num_samples):
            x_points[i] = trapezoid(np.cos(theta_points[:i+1]), x = s_points[:i+1])
            y_points[i] = trapezoid(np.sin(theta_points[:i+1]), x = s_points[:i+1]) 


        x_points = [x + self.start.x for x in x_points]
        y_points = [y + self.start.y for y in y_points]
        theta_points = [normalize_rad_angle(th + self.start.theta) for th in theta_points]

        path = Path(x_points, y_points, theta_points, kappa_points, s_points)

        return path

    
