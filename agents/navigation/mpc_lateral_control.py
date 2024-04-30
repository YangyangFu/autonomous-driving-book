"""
Model Predictive Control (MPC) Lateral Controller
"""
import cvxpy as cp
import numpy as np 
import carla 
Waypoint = carla.Waypoint

from agents.navigation.vehicle_model import DynamicBicycleModel
from agents.navigation.trajectory import Path, PathPoint, Trajectory, TrajectoryPoint
from agents.navigation.cubic_spiral_generator import CubicSpiral
from agents.navigation.linear_interpolation import LinearInterpolation

class Pose:
    x: float 
    y: float
    yaw: float
    v: float

class MPCState:
    e: float # lateral error
    de: float # lateral error rate
    the: float # heading error
    dthe: float # heading error rate

class MPC:
    def __init__(self, state_dim, control_dim, output_dim, dt, horizon, model_name, model_params, Q, R):
        """
        Initialize MPC controller

        Args:
            state_dim (int): state dimension
            control_dim (int): control dimension
            output_dim (int): output dimension
            dt (float): time step
            horizon (int): prediction horizon
            model_name (str): vehicle model name
            model_params (dict): vehicle model parameters
            Q (np.array): state cost matrix
            R (np.array): control cost matrix
            
        """
        self.state_dim = state_dim
        self.control_dim = control_dim
        self.output_dim = output_dim
        self.dt = dt
        self.horizon = horizon
        self.model_name = model_name
        self.model_params = model_params
        self.Q = Q
        self.R = R
        self.max_steering = 1.0
        self.max_steering_rate = 0.1

        self.model = self._initiate_vehichle_model(model_name, model_params)

        # for estimating state
        self._prev_e = 0.0
        self._prev_th = 0.0

    def _initiate_vehichle_model(self, model_name, model_params):
        """
        Initiate vehicle model
        """
        if model_name == "dynamic_bicycle":
            self.model = DynamicBicycleModel(**model_params)
        else:
            raise ValueError("Model not supported yet.")

    def run(self, init_state: MPCState, reference_trajectory: Trajectory):
        """
        Run MPC controller
        """
        # resample trajectory for with mpc sampling time
        mpc_traj = self.resample_trajectory_by_time(reference_trajectory)

        # calculate initial state: zero or from last step?
        # self.calculate_initial_state()

        # update state for delay compensation
        # self.update_state_for_delay_compensation()

        # update state space matrix
        Ad, Bd, wd, Cd = self.update_state_space_matrix()

        # run optimization to return optimal control inputs
        u = self.run_optimization()

        # apply input limitsv
        u = self.apply_input_limits(u)

        # calculate predicted trajectory
        predicted_trajectory = self.calculate_predicted_trajectory()

        return u, predicted_trajectory

    # since the reference trajectory does not take into account the current velocity of the vehicle,
    # we need calculate the trajectory velocity considering the longitudinal dynamics
    def set_reference_trajectory(self, trajectory: Trajectory):
        """
        Set reference trajectory from the planner
        """
        self.reference_trajectory = trajectory 

    def calculate_initial_state(self, curr_pose: Pose, ref_pose: TrajectoryPoint):
        """
        Calculate initial MPC state based on vehicle pose
        """
        curr_state = np.zeros(self.state_dim)
        e = self._cal_lateral_error(curr_pose, ref_pose)
        th = self._normalize_angle(curr_pose.yaw - ref_pose.path_point.theta)
        de = (e - self._prev_e) / self.dt
        dth = (th - self._prev_th) / self.dt

        curr_state[0] = e
        curr_state[1] = de
        curr_state[2] = th
        curr_state[3] = dth

        self._prev_e = e
        self._prev_th = th

        return curr_state

    def _cal_lateral_error(self, curr_pose: Pose, ref_pose: TrajectoryPoint):
        """
        Calculate lateral error
        """
        error_x = curr_pose.x - ref_pose.path_point.x
        error_y = curr_pose.y - ref_pose.path_point.y
        ref_yaw = ref_pose.path_point.theta
        
        lat_error = -np.sin(ref_yaw) * error_x + np.cos(ref_yaw) * error_y

        return lat_error
    
    def _normalize_angle(self, angle):
        """
        Normalize angle in radians to [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def update_state_for_delay_compensation(self):
        """
        Update state for delay compensation
        """
        pass

    def resample_trajectory_by_time(self, in_traj: Trajectory) -> Trajectory:
        """
        Resample reference trajectory for with mpc sampling time
        """

        # calculate the output time
        out_time = self.dt * np.arange(1, self.horizon + 1, 1)

        # linear interpolation
        out_traj = self._linear_interp_(in_traj.t, in_traj, out_time)

        return out_traj
        
        
    def _linear_interp_(self, in_time, in_traj: Trajectory, out_time) -> Trajectory:
        """
        Linear interpolation given raw trajectory and output equidistant time trajectory
        """

        interp = LinearInterpolation()

        # interpolate x, y, yaw, v, t
        out_x = interp.interpolate(in_time, in_traj.x, out_time)
        out_y = interp.interpolate(in_time, in_traj.y, out_time)
        out_theta = interp.interpolate(in_time, in_traj.theta, out_time)
        out_kappa = interp.interpolate(in_time, in_traj.kappa, out_time)
        out_s = interp.interpolate(in_time, in_traj.s, out_time)
        out_v = interp.interpolate(in_time, in_traj.v, out_time)
        out_t = out_time

        # output interpolated trajectory
        out_traj = Trajectory(Path(out_x, out_y, out_theta, out_kappa, out_s), out_v, out_t)

        return out_traj

    
    def update_state_space_matrix(self, velocity: float, curvature:float):
        """
        Update state space matrix for a given velocity and curvature 
        """
        
        #return Ad, Bd, wd, Cd  
        self.model.set_velocity(velocity)
        self.model.set_curvature(curvature)
        self.model.update_matrix() 
        self.model.update_discrete_matrix(self.dt)

        return self.model.Ad, self.model.Bd, self.model.Wd, self.model.Cd

    def run_optimization(self, init_state: MPCState, mpc_traj: Trajectory):
        """
        Formulate and run optimization to return optimal control inputs
        """
        # define optimization variables
        u = cp.Variable((self.control_dim, self.horizon))

        # define state variables
        x = cp.Variable((self.state_dim, self.horizon+1))

        # define constraints
        # equality constraints
        constraints = []
        x0 = np.array([init_state.e, init_state.de, init_state.the, init_state.dthe])
        constraints += [x[:, 0] == x0]
        
        for i in range(1, self.horizon+1):
            vi = mpc_traj[i].v
            ki = mpc_traj[i].kappa
            Ad, Bd, Wd, Cd = self.update_state_space_matrix(vi, ki)
            constraints += [x[:, i] == Ad @ x[:, i-1] + Bd @ u + Wd]

        # inequality constraints due to input limits
        # umin <= u <= umax
        # dumin*dt <= du <= dumax*dt
        for i in range(self.horizon):
            constraints += [u[:, i] <= self.max_steering], [u[:, i] >= -self.max_steering]
        
        for i in range(self.horizon-1):
            constraints += [u[:, i+1] - u[:, i] <= self.max_steering_rate * self.dt], [u[:, i+1] - u[:, i] >= -self.max_steering_rate * self.dt]


        # define cost function
        cost = 0
        for i in range(1, self.horizon+1):
            cost += cp.quad_form(x[:, i], self.Q) + cp.quad_form(u[:, i-1], self.R)

        # define optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)

        # solve optimization
        prob.solve()

        return u

    def calculate_predicted_trajectory(self, curr_state: MPCState, u, ref_traj: Trajectory) -> Trajectory:
        """
        Calculate predicted trajectory
        """
        predicted_traj = Trajectory()
        
        # initialize
        # (4,1)
        state = np.array([curr_state.e, curr_state.de, curr_state.the, curr_state.dthe]).reshape(-1, 1)

        # iterate over the horizon
        for i in range(self.horizon):
            vi = ref_traj[i].v
            ki = ref_traj[i].kappa
            Ad, Bd, Wd, Cd = self.update_state_space_matrix(vi, ki)
            state = Ad @ state + Bd @ u[:, i] + Wd
            mpc_state = MPCState(e=state[0], de=state[1], the=state[2], dthe=state[3])
            path_point = self.convert_state_to_pathpoint(mpc_state, ref_traj[i])

            predicted_traj.push_back(path_point, vi, self.dt * (i + 1))

        return predicted_traj
    
    def convert_state_to_pathpoint(self, state: MPCState, ref_pose: TrajectoryPoint) -> PathPoint:
        """
        Convert MPC state to PathPoint
        """
        x = ref_pose.path_point.x - state.e * np.sin(ref_pose.path_point.theta)
        y = ref_pose.path_point.y + state.e * np.cos(ref_pose.path_point.theta)
        theta = ref_pose.path_point.theta + state.the
        kappa = ref_pose.path_point.kappa
        s = ref_pose.path_point.s

        return PathPoint(x, y, theta, kappa, s)

        

class MPCLateralControl():
    def __init__(self, vehicle):
        self._vehicle = vehicle
        self._cubic_spiral = CubicSpiral()
        
        pass 

    def run_step(self):
        pass
    

    def _extract_goal(self, route, lookahead_distance_min, lookahead_distance_max, lookahead_time) -> Waypoint:
        """
        Extract goal point from given planning route for the motion planner
        
        """
        pass 

    def generate_mpc_trajectory(self, goal_waypoint) -> Trajectory:
        """
        Generate MPC trajectory
        
        """
        # generate cubic spirals

        # generate velocity profile

        # generate trajectory
        pass 

    def _initiate_vehichle_model(self):
        pass

