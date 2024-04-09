import numpy as np
from collections import deque 

class StanleyLateralController():
    """
    StanleyLateralController implements lateral control using Stanley method.
    
    """
    def __init__(self, vehicle, wheel_base, k=2.0, soft_gain=1.0, max_steer=1.0):
        """ Constructor method 
        
        :param vehicle: actor to apply to local planner logic onto
        :param wheel_base: distance between front and rear wheels
        :param k: gain for the control policy
        :param soft_gain: gain for the softening of the control policy

        :return: None
        """
        self._vehicle = vehicle
        self._wheel_base = wheel_base
        self.max_steer = max_steer
        self.past_steering = self._vehicle.get_control().steer

        # control policy parameters
        self.k = k
        self.ks = soft_gain

        # for debugging purpose
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoints):

        # find goal point 
        goal_point = self.find_goal_point(waypoints)
        
        # calculate heading error
        heading_error = self.heading_error(goal_point)

        # calculate cross track error
        cte = self.cross_track_error(goal_point)
        #cte = self.cross_error(goal_point, self._vehicle.get_transform())
        
        # calculate steering angle 
        steering = self.control_policy(heading_error, cte)

        # regulate steering angle
        if steering > self.past_steering + 0.1:
            steering = self.past_steering + 0.1
        elif steering < self.past_steering - 0.1:
            steering = self.past_steering - 0.1

        self.past_steering = steering

        # save error for debugging
        self._e_buffer.append(cte)

        return steering  
    
    def normalize_angle(self, angle):
        """ Normalize the angle between -pi and pi 
        
        :param angle: angle to normalize
        :return: normalized angle
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def find_goal_point(self, waypoints):
        # TODO: it is better to interpolate the waypoints to get a smooth path
        # find the closest waypoint to the vehicle
        next_wp = waypoints[0]
        
        return next_wp


    def heading_error(self, goal_point) -> float:
        
        theta_e = np.radians(self._vehicle.get_transform().rotation.yaw - goal_point[0].transform.rotation.yaw)

        return self.normalize_angle(theta_e)

    def cross_track_error(self, goal_point) -> float:
         
        # vehicle transform and location
        ego_loc = self._vehicle.get_location()
        ego_yaw = self._vehicle.get_transform().rotation.yaw

        # front axle location
        xf = ego_loc.x + self._wheel_base / 2.0 * np.cos(np.radians(ego_yaw))
        yf = ego_loc.y + self._wheel_base / 2.0 * np.sin(np.radians(ego_yaw))

        # goal point location
        xg = goal_point[0].transform.location.x
        yg = goal_point[0].transform.location.y
        g_vec = goal_point[0].transform.get_forward_vector()
        g_vec = np.array([g_vec.x, g_vec.y, 0.0])

        # cross-track error vector
        e = np.array([xg - xf, yg - yf, 0.0])

        # cross-track error 
        cross = np.cross(e, g_vec)

        # if on the left side of a path, de is negative
        de = cross[2]
        
        return de
    
            
    def control_policy(self, heading_error, cross_track_error) -> float:
        """ Stanley control policy 
        
        :return: steering angle
        """ 
        vel = self._vehicle.get_velocity()
        speed = np.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) # m/s

        # NOTE: CARLA uses left-handed coordinate system. The error needs to be negated acoording to method 1.
        steering = -heading_error - np.arctan2(self.k * cross_track_error, self.ks + speed)

        steering = self.normalize_angle(steering)

        # normalize to [-1, 1] based on max steering angle
        steering = steering / self.max_steer
        
        return np.clip(steering, -1.0, 1.0)