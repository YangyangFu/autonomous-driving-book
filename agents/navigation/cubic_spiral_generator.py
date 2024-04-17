import numpy as np
from agents.tools.misc import normalize_rad_angle

class CubicSpiral:
    """
    Cubic spiral generator between given start point and end point

    Model description:
    x_p(s) = \int_0^s cos(theta_p(s)) ds
    y_p(s) = \int_0^s sin(theta_p(s)) ds

    theta_p(s) = a0*s + a1/2*s^2 + a2/3*s^3 + a3/4*s^4
    kappa_p(s) = a0 + a1*s + a2*s^2 + a3*s^3

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

        # ending point
        x_e = end.location.x
        y_e = end.location.y
        th_e = normalize_rad_angle(np.radians(end.rotation.yaw))

        # transform to start point frame
        x_t = x_e - x_s
        y_t = y_e - y_s

        x_g = x_t * np.cos(th_s) + y_t * np.sin(th_s)
        y_g = -x_t * np.sin(th_s) + y_t * np.cos(th_s)
        th_g = normalize_rad_angle(th_e - th_s) 

        

    def get_sampled_spiral(self, num_samples):
        """
        Get sampled spirals
        """
        pass 
    
