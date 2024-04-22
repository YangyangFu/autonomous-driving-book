import numpy as np
from scipy.integrate import simpson 

from agents.tools.misc import normalize_rad_angle

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

        pass 

    def generate_spiral(self, start, end):
        """
        Generate cubic spiral between start and end points
        """
        # starting point
        x_s = start.location.x 
        y_s = start.location.y
        th_s = normalize_rad_angle(np.radians(start.rotation.yaw)) 
        k_s = self._estimate_waypoint_curvature(start)

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

        # initial guess of s_g, will be updated by Newton-Raphson method
        s_g = (th_g * th_g / 5.0 + 1.0) * np.sqrt(x_g * x_g + y_g * y_g)

        # parameters 
        p = np.zeros((4,1))
        p[0,0] = k_s
        p[3,0] = k_g

        # Newton-Raphson method

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
    
    def partial_kappa_a(self, a0, a1, a2, a3, s):
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
        a = self.p_to_a(p0, p1, p2, p3, s)
        dk_da = self.partial_kappa_a(*a, s)
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
    
    def get_sampled_spiral(self, num_samples):
        """
        Get sampled spirals
        """
        pass 
    
