# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math
import numpy as np
import carla
from agents.tools.misc import get_speed
from agents.navigation.lqr_lateral_control import LQRLateralControl

class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """


    def __init__(self, vehicle, config, offset=0, max_throttle=0.75, max_brake=0.3,
                 max_steering=0.4):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param offset: If different than zero, the vehicle will drive displaced from the center line.
        Positive values imply a right offset while negative ones mean a left one. Numbers high enough
        to cause the vehicle to drive through other lanes might break the controller.
        """

        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self.past_steering = self._vehicle.get_control().steer

        # longitudinal controller
        if config['longitudinal_controller']['name'] == 'PID':
            self._lon_controller = PIDLongitudinalController(self._vehicle, **config['longitudinal_controller']['args'])
        else:
            raise ValueError("Longitudinal controller not recognized: {}".format(config['longitudinal_controller']['name']))
        
        # lateral controller
        if config['lateral_controller']['name'] == 'PID':
            self._lat_controller = PIDLateralController(self._vehicle, offset, **config['lateral_controller']['args'])
        elif config['lateral_controller']['name'] == 'PurePursuit':
            self._lat_controller = PurePursuitLateralController(self._vehicle, **config['lateral_controller']['args'])
        elif config['lateral_controller']['name'] == 'Stanley':
            self._lat_controller = StanleyLateralController(self._vehicle, **config['lateral_controller']['args'])
        elif config['lateral_controller']['name'] == 'LQR':
            self._lat_controller = LQRLateralControl(self._vehicle, **config['lateral_controller']['args'])
        else:
            raise ValueError("Lateral controller not recognized: {}".format(config['lateral_controller']['name']))

    def run_step(self, target_speed, waypoints):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: distance (in meters) to the waypoint
        """

        acceleration = self._lon_controller.run_step(target_speed)
        current_steering = self._lat_controller.run_step(waypoints)
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.
        """
        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1
        """
        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)
        
        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering

        return control


    def change_longitudinal_PID(self, args_longitudinal):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller.change_parameters(**args_longitudinal)

    def change_lateral_PID(self, args_lateral):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller.change_parameters(**args_lateral)


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        current_speed = get_speed(self._vehicle)

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line. If might cause issues if the value
                is large enough to make the vehicle invade other lanes.
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._offset = offset
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoints):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

            :param waypoints: a list of waypoints
            :return: steering control in the range [-1, 1] where:
            -1 maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoints[0][0], self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

            :param waypoint: target waypoint
            :param vehicle_transform: current transform of the vehicle
            :return: steering control in the range [-1, 1]
        """
        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Get the vector vehicle-target_wp
        if self._offset != 0:
            # Displace the wp to the side
            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
                                                         y=self._offset*r_vec.y)
        else:
            w_loc = waypoint.transform.location

        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        # check the direction of angle between v_vec and w_vec
        # if positive, then steering is to the left of the waypoint
        # based on the right hand rule, if the v_vec is pointing to the right of the w_vec, 
        # then the angle from v_vec to w_vec is (0, pi), else (pi, 2*pi)
        _cross = np.cross(v_vec, w_vec)
        # if more than pi, then reverse action is needed.
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class PurePursuitLateralController():
    """
    PurePursuitLateralController implements lateral control using pure pursuit method.
    """

    def __init__(self, vehicle, wheel_base, lookahead_gain=0.1):
        self._vehicle = vehicle
        self._wheel_base = wheel_base
        self._lookahead_gain = lookahead_gain

        # for debugging purpose
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoints):
        """Execute one step of lateral control to steer the vehicle towards a certain waypoint."""

        # Calculate the look ahead distance
        lookahead_distance = self.get_lookahead_distance()
        lookahead_point, _ = self.find_lookahead_point(self._vehicle.get_location(), waypoints, lookahead_distance)

        # Calculate the steering angle
        steering = self.pure_pursuit_control(lookahead_point, lookahead_distance)


        return steering 
    
    def get_lookahead_distance(self):
        velocity = self._vehicle.get_velocity()
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        lookahead_distance = max(2, self._lookahead_gain * speed)

        return lookahead_distance

    def find_lookahead_point(self, vehicle_location, waypoints, lookahead_distance):
        """
        Find the point on the path that is closest to the vehicle and further than the current position
        """
        closest_len = 10000
        lookahead_point = waypoints[0]
        for i in range(len(waypoints)):
            waypoint, _ = waypoints[i]
            distance = np.linalg.norm([waypoint.transform.location.x - vehicle_location.x, 
                                       waypoint.transform.location.y - vehicle_location.y])
            if distance < closest_len:
                closest_len = distance
                if distance > lookahead_distance:
                    lookahead_point = waypoint
                    break
        return lookahead_point, i
        

    def pure_pursuit_control(self, lookahead_point, lookahead_distance):
        alpha = math.atan2(lookahead_point.transform.location.y - self._vehicle.get_location().y, 
                           lookahead_point.transform.location.x - self._vehicle.get_location().x) \
                            - np.radians(self._vehicle.get_transform().rotation.yaw)
        
        delta = math.atan2(2.0 * self._wheel_base * math.sin(alpha) / lookahead_distance, 1.0)

        # save steering error for debug
        error = self.calculate_steering_error(lookahead_point, self._vehicle.get_transform())
        self._e_buffer.append(error)

        return np.clip(delta, -1.0, 1.0)
    

    def calculate_steering_error(self, waypoint, vehicle_transform):
        """
        TODO: this looks sketchy, need to be checked
        Calculate the steering error of the vehicle based on the current waypoint
        """
        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Get the vector vehicle-target_wp
        w_loc = waypoint.transform.location
        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        # check the direction of angle between v_vec and w_vec
        # if positive, then steering is to the left of the waypoint
        # based on the right hand rule, if the v_vec is pointing to the right of the w_vec, 
        # then the angle from v_vec to w_vec is (0, pi), else (pi, 2*pi)
        _cross = np.cross(v_vec, w_vec)
        # if more than pi, then reverse action is needed.
        if _cross[2] < 0:
            _dot *= -1.0

        return _dot

    def change_parameters(self):
        pass


class StanleyLateralController():
    """
    StanleyLateralController implements lateral control using Stanley method.
    
    """
    def __init__(self, vehicle, wheel_base, k=2.0, soft_gain=1.0):
        """ Constructor method 
        
        :param vehicle: actor to apply to local planner logic onto
        :param wheel_base: distance between front and rear wheels
        :param k: gain for the control policy
        :param soft_gain: gain for the softening of the control policy

        :return: None
        """
        self._vehicle = vehicle
        self._wheel_base = wheel_base

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
        cross = np.cross(g_vec, e)
        print("goal point: ", xg, yg, "vehicle: ", xf, yf)
        print("g, e, cross: ", g_vec, e)
        print("cross: ", cross[2])
        print("====================================")
        # if on the left side of a path, de is negative
        de = cross[2]
        
        return de
    
            
    def control_policy(self, heading_error, cross_track_error) -> float:
        """ Stanley control policy 
        
        :return: steering angle
        """ 
        speed = get_speed(self._vehicle)
        # NOTE: CARLA uses left-handed coordinate system. The error needs to be negated acoording to method 1.
        steering = -heading_error + np.arctan2(self.k * cross_track_error, self.ks + speed)

        steering = self.normalize_angle(steering)

        # normalize to [-1, 1] based on max steering angle
        max_steering = self._vehicle.get_physics_control().wheels[0].max_steer_angle
        
        steering = steering/np.radians(max_steering)
        
        return np.clip(steering, -1.0, 1.0)
