import numpy as np
from collections import deque

class PurePursuitLateralController():
    """
    PurePursuitLateralController implements lateral control using pure pursuit method.
    """

    def __init__(self, vehicle, wheel_base, max_steer = 1.0, lookahead_gain=0.1):
        self._vehicle = vehicle
        self._wheel_base = wheel_base
        self.max_steer = max_steer
        self._lookahead_gain = lookahead_gain
        self.past_steering = self._vehicle.get_control().steer

        # for debugging purpose
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoints):
        """Execute one step of lateral control to steer the vehicle towards a certain waypoint."""

        # Calculate the look ahead distance
        lookahead_distance = self.get_lookahead_distance()
        lookahead_point, _ = self.find_lookahead_point(self._vehicle.get_location(), waypoints, lookahead_distance)

        # Calculate the steering angle
        steering = self.pure_pursuit_control(lookahead_point, lookahead_distance)

        # regulate steering angle 
        if steering > self.past_steering + 0.1:
            steering = self.past_steering + 0.1
        elif steering < self.past_steering - 0.1:
            steering = self.past_steering - 0.1

        self.past_steering = steering        

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
        alpha = np.arctan2(lookahead_point.transform.location.y - self._vehicle.get_location().y, 
                           lookahead_point.transform.location.x - self._vehicle.get_location().x) \
                            - np.radians(self._vehicle.get_transform().rotation.yaw)
        
        steer = np.arctan2(2.0 * self._wheel_base * np.sin(alpha) / lookahead_distance, 1.0)

        # normalize the steering angle
        steer = steer / self.max_steer

        # save steering error for debug
        error = self.calculate_steering_error(lookahead_point, self._vehicle.get_transform())
        self._e_buffer.append(error)

        return np.clip(steer, -1.0, 1.0)
    

    def calculate_steering_error(self, waypoint, vehicle_transform):
        """
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
            _dot = np.arccos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        
        # cross error
        _cross = np.cross(v_vec, w_vec)
        # if more than pi, then reverse action is needed.
        if _cross[2] < 0:
            _dot *= -1.0

        return _dot

    def change_parameters(self):
        pass



