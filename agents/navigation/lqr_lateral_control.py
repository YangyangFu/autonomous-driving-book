import numpy as np

from vehicle_model import DynamicBicycleModel

class LQRLateralControl():
    """
    LQR optimal control policy based on different vehicle model.
    """

    def __init__(self, vehicle, model_name, model_params, max_steer, dt=0.1):
        
        self._vehicle = vehicle

        # initiate vehicle model
        self.model_name = model_name
        self._initiate_vehichle_model(model_name, model_params)

        # vehicle parameters
        self.max_steer = max_steer

        # LQR parameters
        self.dt = dt
        self.q = [1.0] * self.model.state_dim
        self.r = [1.0] * self.model.control_dim

        # initialize previous state
        self._prev_e = 0.0
        self._prev_th = 0.0

        # initialize previous waypoint to compute curvature
        # if smooth trajectory is provided, this can be removed
        self._temp_waypoint = self._vehicle.get_world().get_map().get_waypoint(self._vehicle.get_location())
        self._prev_waypoint = None

    def _initiate_vehichle_model(self, model_name, model_params):
        if model_name == "dynamic_bicycle":
            self.model = DynamicBicycleModel(**model_params)
        else:
            raise ValueError("Model not supported yet.")

    def run_step(self, waypoints):
        
        # update historical waypoints
        goal_point = waypoints[0]

        if self._temp_waypoint != goal_point:
            self._prev_waypoint = self._temp_waypoint
            self._temp_waypoint = goal_point

        # compute control command 
        steer, _ = self.compute_control_command(self._vehicle, goal_point)

        # bound steer to carla env 
        steer = np.clip(steer, -1.0, 1.0)

        return steer
    
    def extract_trajectory(self):
        pass
    
    
    def extract_current_state(self, ego_pose, ref_pose):
        """
        Extract current state of the vehicle model.
        """
        x = np.zeros((self.model.state_dim, 1))

        if self.model_name == "dynamic_bicycle":
            e = self.compute_lateral_error(ego_pose, ref_pose)
            th = self.normalize_angle(np.radians(ego_pose.rotation.yaw) - np.radians(ref_pose.rotation.yaw))
            de = (e - self._prev_e) / self.dt
            dth = (th - self._prev_th) / self.dt

            x[0, 0] = e
            x[1, 0] = de
            x[2, 0] = th
            x[3, 0] = dth

            self._prev_e = e
            self._prev_th = th

        return x
    
    def normalize_angle(self, angle):
        """
        Normalize angle in radians to [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def update_discrete_matrix(self, velocity, curvature):
        
        # TODO: current velocity or desired velocity?
        self.model.set_velocity(velocity)
        self.model.update_matrix()

        self.model.set_curvature(curvature)
        self.model.update_discrete_matrix()


    def compute_lateral_error(self, ego_pose, ref_pose):
        """
        Compute lateral error based on current position and desired trajectory.
        """
        dx = ego_pose.location.x - ref_pose.location.x
        dy = ego_pose.location.y - ref_pose.location.y
        ref_yaw = np.radians(ref_pose.rotation.yaw)

        lat_error = - np.sin(ref_yaw) * dx + np.cos(ref_yaw) * dy

        return lat_error

    def solve_lqr(self, Ad, Bd, Q, R, M, tolerance=1e-6, max_iter=1000):
        """
        Solve the discrete-time LQR controller by iterating to a fixed point.
        
        min (x'Qx + u'Ru + 2x'Mu)

        :param Ad: discrete-time system matrix
        :param Bd: discrete-time control matrix
        :param Q: cost function matrix
        :param R: control effort matrix
        :param M: cross-term matrix
        :param tolerance: convergence criteria for discrete-time Algebraic Riccati Equation
        :param max_iter: maximum number of iterations

        :return: K: feedback gain matrix

        """
        # check validity of the input matrices
        if (Ad.shape[0] != Ad.shape[1]) or \
            (Bd.shape[0] != Ad.shape[0]) or \
            (Q.shape[0] != Q.shape[1]) or \
            (Q.shape[0] != Ad.shape[0]) or \
            (R.shape[0] != R.shape[1]) or \
            (R.shape[0] != Bd.shape[1]) or \
            (M.shape[0] != Q.shape[0]) or \
            (M.shape[1] != R.shape[1]):
            raise ValueError("Input matrices are not in the correct shape.")

        Ad_T = Ad.T
        Bd_T = Bd.T
        M_T = M.T

        # solve discrete-time Algebraic Riccati Equation
        # calculate matrix difference Riccati equation, initialize P and Q
        P = Q
        num_iter = 0
        diff = np.inf

        while num_iter < max_iter and diff > tolerance:
            P_next = Ad_T @ P @ Ad - (Ad_T @ P @ Bd + M) @ np.linalg.inv(R + Bd_T @ P @ Bd) @ (Bd_T @ P @ Ad + M_T) + Q
            
            # check convergence
            diff = np.abs(P_next - P).max()
            P = P_next
            num_iter += 1
        
        # convergence check
        if num_iter >= max_iter:
            message = "LQR solver did not converge."
        else:
            message = "LQR solver converged in {} iterations with a tolerance of {}.".format(num_iter, tolerance)
        print(message)

        # compute feedback gain matrix
        K = np.linalg.inv(R + Bd_T @ P @ Bd) @ (Bd_T @ P @ Ad + M_T)
        
        return K

    def compute_curvature(self, prev_waypoint, ref_waypoint):
        """
        Compute approximately curvature based on previous and current waypoint.
        """
        
        yaw_prev = np.radians(prev_waypoint.rotation.yaw)
        yaw_ref = np.radians(ref_waypoint.rotation.yaw)

        dx = ref_waypoint.location.x - prev_waypoint.location.x
        dy = ref_waypoint.location.y - prev_waypoint.location.y

        curvature = 2 * np.sin((yaw_ref - yaw_prev)/2) / np.sqrt(dx**2 + dy**2)
        
        # TODO: can curvature be negative? 
        return curvature


    def compute_control_command(self, vehicle, ref_pose):
        """
        Compute the control command based on the LQR optimal control policy.
        - update discrete-time linearized system matrices
        - adjust Q matrix for reverse driving, highspeed, etc
        - solve LQR problem
        - compute feedback control policy for steering angle
        - compute feedforward control policy for steering angle
        - compute feedback control augment for steering angle
        - set control commmand within range
        """


        # udpate matrix based on current vehicle state
        velocity = np.linalg.norm(vehicle.get_velocity(), 2) # m/s
        curvature = self.compute_curvature(self._prev_waypoint, ref_pose)
        self.update_discrete_matrix(velocity, curvature)
        
        # solve LQR problem to get feedback gain
        Q = np.diag(self.q)
        R = np.diag(self.r)
        K = self.solve_lqr(self.model.Ad, self.model.Bd, Q, R)

        # compute feedback control policy
        cur_state = self.extract_current_state(vehicle.get_transform(), ref_pose)
        u = -K @ cur_state

        steer = u[0, 0]
        steer = self.normalize_angle(steer)
        steer = steer / self.max_steer

        # compute feedforward control policy to decrease steady state error

        return steer, cur_state
