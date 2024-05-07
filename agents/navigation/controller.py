# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import numpy as np
import carla
from agents.tools.misc import get_speed
from agents.navigation.pid_lateral_control import PIDLateralController
from agents.navigation.pure_pursuit_lateral_control import PurePursuitLateralController
from agents.navigation.stanley_lateral_control import StanleyLateralController
from agents.navigation.lqr_lateral_control import LQRLateralController
from agents.navigation.mpc_lateral_control import MPCLateralController

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
        self.past_throttle = self._vehicle.get_control().throttle
        self.past_brake = self._vehicle.get_control().brake

        # FOR DEBUGGING
        self._target_speed = 0

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
            self._lat_controller = LQRLateralController(self._vehicle, **config['lateral_controller']['args'])
        elif config['lateral_controller']['name'] == 'MPC':
            self._lat_controller = MPCLateralController(self._vehicle, **config['lateral_controller']['args'])
        else:
            raise ValueError("Lateral controller not recognized: {}".format(config['lateral_controller']['name']))

    def run_step(self, target_speed, waypoints):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed in Km/h
            :param waypoint: target location encoded as a waypoint
            :return: distance (in meters) to the waypoint
        """
        # longitudinal
        # for debugging
        self._target_speed = target_speed

        # reset I after stop
        if self._vehicle.is_at_traffic_light() and get_speed(self._vehicle) < 0.1:
            self._lon_controller._total_error_prev = 0

        acceleration = self._lon_controller.run_step(target_speed)
        print("Total error for i: ", self._lon_controller._total_error_prev)
        # lateral control        
        current_steering = self._lat_controller.run_step(waypoints)
        
        # cannot acc/dec too fast
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            curr_throttle = min(acceleration, self.max_throt)
            curr_brake = 0.0
        else:
            curr_throttle = 0.0
            curr_brake = min(abs(acceleration), self.max_brake)

        if curr_throttle > self.past_throttle + 0.1:
            curr_throttle = self.past_throttle + 0.1
        elif curr_throttle < self.past_throttle - 0.1:
            curr_throttle = max(0, self.past_throttle - 0.1)
        
        if curr_brake > self.past_brake + 0.1:
            curr_brake = self.past_brake + 0.1
        elif curr_brake < self.past_brake - 0.1:
            curr_brake = max(0, self.past_brake - 0.1)

        control.throttle = curr_throttle
        control.brake = curr_brake
        control.steer = current_steering
        control.hand_brake = False
        control.manual_gear_shift = False

        self.past_throttle = curr_throttle #self._vehicle.get_control().throttle #curr_throttle
        self.past_brake = curr_brake #self._vehicle.get_control().brake #curr_brake

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

        # for I
        self._total_error_prev = 0

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
        total_error = self._total_error_prev + error

        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = total_error * self._dt
        else:
            _de = 0.0
            _ie = 0.0
        
        self._total_error_prev = total_error

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
