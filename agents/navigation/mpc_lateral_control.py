"""
Model Predictive Control (MPC) Lateral Controller
"""
from dataclasses import dataclass
from collections import deque 

import cvxpy as cp
import numpy as np 
import carla 
Waypoint = carla.Waypoint

from agents.navigation.vehicle_model import DynamicBicycleModel
from agents.navigation.trajectory import Path, PathPoint, Trajectory, TrajectoryPoint
from agents.navigation.cubic_spiral_generator import CubicSpiral
from agents.navigation.linear_interpolation import LinearInterpolation
from agents.navigation.velocity_profile_generator import VelocityGenerator

@dataclass
class Pose:
    x: float 
    y: float
    yaw: float
    v: float

@dataclass
class MPCState:
    e: float # lateral error
    de: float # lateral error rate
    the: float # heading error
    dthe: float # heading error rate

class MPC:
    def __init__(self, model_name, model_params, max_steer, max_steer_rate, dt, horizon, Q, R):
        """
        Initialize MPC controller

        Args:
            state_dim (int): state dimension
            control_dim (int): control dimension
            output_dim (int): output dimension
            dt (float): time step in second
            horizon (float): prediction horizon in seconds
            model_name (str): vehicle model name
            model_params (dict): vehicle model parameters
            Q (np.array): state cost matrix
            R (np.array): control cost matrix
            
        """
        # model parameters
        self.model_name = model_name
        self.model_params = model_params
        
        # dimensions
        self._initiate_vehichle_model(model_name, model_params)
        self.state_dim = self.model.state_dim
        self.control_dim = self.model.control_dim
        self.output_dim = self.model.output_dim

        # MPC parameters
        self.dt = dt
        self.horizon = horizon
        self.steps = int(horizon / dt)
        self.Q = np.diag([Q, 0, 0, 0]) # state cost matrix
        self.R = np.diag([R]) # control cost matrix

        # control limits
        self.max_steering = max_steer
        self.max_steering_rate = max_steer_rate

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

    def run(self, curr_pose: Pose, reference_trajectory: Trajectory):
        """
        Run MPC controller
        """
        # resample trajectory for with mpc sampling time
        mpc_traj = self.resample_trajectory_by_time(reference_trajectory)

        # calculate initial state: zero or from last step?
        init_state = self.calculate_initial_state(curr_pose, mpc_traj[0])

        # update state for delay compensation
        # self.update_state_for_delay_compensation()

        # run optimization to return optimal control inputs
        u_opt = self.run_optimization(init_state, mpc_traj)

        # validity check

        # calculate predicted trajectory
        predicted_trajectory = self.calculate_predicted_trajectory(init_state, u_opt, mpc_traj)

        return u_opt, predicted_trajectory

    # since the reference trajectory does not take into account the current velocity of the vehicle,
    # we need calculate the trajectory velocity considering the longitudinal dynamics
    def set_reference_trajectory(self, trajectory: Trajectory):
        """
        Set reference trajectory from the planner
        """
        pass 

    def calculate_initial_state(self, curr_pose: Pose, ref_pose: TrajectoryPoint) -> MPCState:
        """
        Calculate initial MPC state based on vehicle pose
        """
        
        e = self._cal_lateral_error(curr_pose, ref_pose)
        th = self._normalize_angle(curr_pose.yaw - ref_pose.path_point.theta)
        de = (e - self._prev_e) / self.dt
        dth = (th - self._prev_th) / self.dt
        init_state = MPCState(e, de, th, dth)
        
        #init_state.e = e
        #init_state.de = de
        #init_state.the = th
        #init_state.dthe = dth

        self._prev_e = e
        self._prev_th = th

        return init_state

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
        out_time = self.dt * np.arange(1, self.steps + 1, 1)

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
        u = cp.Variable((self.control_dim, self.steps))

        # define state variables
        x = cp.Variable((self.state_dim, self.steps+1))

        # define constraints
        # equality constraints
        constraints = []
        x0 = np.array([init_state.e, init_state.de, init_state.the, init_state.dthe])
        constraints += [x[:, 0] == x0]
        
        for i in range(1, self.steps+1):
            vi = mpc_traj[i].v
            ki = mpc_traj[i].path_point.kappa
            Ad, Bd, Wd, Cd = self.update_state_space_matrix(vi, ki)
            constraints += [x[:, [i]] == Ad @ x[:, [i-1]] + Bd @ u[:, [i-1]] + Wd]

        # inequality constraints due to input limits
        # umin <= u <= umax
        # dumin*dt <= du <= dumax*dt
        for i in range(self.steps):
            constraints += [u[:, i] <= self.max_steering, u[:, i] >= -self.max_steering]
        
        for i in range(self.steps-1):
            constraints += [u[:, i+1] - u[:, i] <= self.max_steering_rate * self.dt, u[:, i+1] - u[:, i] >= -self.max_steering_rate * self.dt]


        # define cost function
        cost = 0
        for i in range(1, self.steps+1):
            cost += cp.quad_form(x[:, [i]], self.Q) + cp.quad_form(u[:, [i-1]], self.R)

        # define optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)

        # solve optimization
        prob.solve()

        # log optimization status
        print("====================== QP result: ")
        print("QP status: ", prob.solver_stats)
        print("The minimum cost is :", prob.value)

        u_opt = u.value     

        return u_opt

    def calculate_predicted_trajectory(self, init_state: MPCState, u:np.array, ref_traj: Trajectory) -> Trajectory:
        """
        Calculate predicted trajectory
        """
        predicted_traj = Trajectory()
        
        # initialize
        # (4,1)
        state = np.array([init_state.e, init_state.de, init_state.the, init_state.dthe]).reshape(-1, 1)

        # iterate over the horizon
        for i in range(self.steps):
            vi = ref_traj[i].v
            ki = ref_traj[i].path_point.kappa
            Ad, Bd, Wd, Cd = self.update_state_space_matrix(vi, ki)
            state = Ad @ state + Bd @ u[:, [i]] + Wd
            mpc_state = MPCState(e=state[0,0], de=state[1,0], the=state[2,0], dthe=state[3,0])
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

        
"""
TODO: move motion planner to behavior planner
"""
class MotionPlanner:
    """
    A simple motion planner for a given goal and velocity target. 

    TODO: Collision checking is not implemented yet.

    """
    def __init__(self, 
                 map: carla.Map, 
                 vehicle: carla.Vehicle, 
                 goal: carla.Waypoint, 
                 velocity_target: float, 
                 num_samples: int = 101,
                 steer_max: float = 1.0):
        self._map = map
        self._vehicle = vehicle
        self._goal = goal
        self._velocity_target = velocity_target # m/s
    
        # initialize curve generator
        self.spiral = CubicSpiral()
        self.num_samples = num_samples 
        
        # vehicle constraints
        self.steer_max = steer_max # rad

    def run(self) -> Trajectory:  
        """
        Run motion planner
        """
        # current pose of vehicle
        start_xyz = self._vehicle.get_transform().location
        start_wp = self._map.get_waypoint(start_xyz, project_to_road=True, lane_type=carla.LaneType.Driving) 
        v_vect = self._vehicle.get_velocity()
        v_vect = np.array([v_vect.x, v_vect.y, 0])
        start_speed = np.linalg.norm(v_vect) # m/s
        
        # generate cubic spirals
        self.spiral.generate_spiral(start_wp, self._goal)
        path = self.spiral.get_sampled_trajectory(self.num_samples)
        
        # generate velocity profile
        velocity_generator = VelocityGenerator(path, start_speed, self._velocity_target, self.steer_max)
        traj = velocity_generator.generate_velocity_profile()

        return traj


class MPCLateralController():
    def __init__(self, vehicle, model_name, model_params, max_steer=1.0, max_steer_rate = 0.1, dt=0.1, horizon=5, Q=0.5, R=1):
        self._vehicle = vehicle
        self._map = self._vehicle.get_world().get_map()
        
        # MPC instance
        self.mpc = MPC(model_name, model_params, max_steer, max_steer_rate, dt, horizon, Q, R)

        # motion planner parameter
        self.lookahead_distance_min = 20 #m
        self.lookahead_distance_max = 150 #m
        self.lookahead_time = horizon #s
        self.velocity_target = None #m/s

        # for debugging
        self._e_buffer = deque(maxlen=10)
    
    def set_velocity_target(self, target):
        self.velocity_target = target

    def run_step(self, waypoints):
        
        # vehicle pose
        v = self._vehicle.get_velocity()
        v = np.array([v.x, v.y])
        v = np.linalg.norm(v)
        ego_transform = self._vehicle.get_transform()
        ego_wp = self._map.get_waypoint(ego_transform.location, 
                                        project_to_road=True, 
                                        lane_type=carla.LaneType.Driving)
        
        ego_pose = Pose(x=ego_transform.location.x, 
                        y=ego_transform.location.y, 
                        yaw = np.radians(ego_transform.rotation.yaw),
                        v = v)
        
        # find goal point from global planner
        goal_point = self._extract_goal(ego_pose, ego_wp, waypoints)

        # generate trajectory
        mplaner = MotionPlanner(self._map, self._vehicle, goal_point, self.velocity_target)
        mpc_traj = mplaner.run()

        # run MPC controller
        init_state = self.mpc.calculate_initial_state(ego_pose, mpc_traj[0])
        uopt = self.mpc.run_optimization(init_state, mpc_traj)

        # only apply the first step
        steer = uopt[:, 0][0]

        # predicted trajectory for debugging purpose
        self._e_buffer.append(init_state.e)

        return steer


    def _extract_goal(self, ego_pose, ego_wp, route) -> carla.Waypoint:
        """
        Extract goal point from given planning route for the motion planner
        
        This is a temporary implementation as normally the goal point should be given by behavior planner.
        Here we assume planner gives a route to the target. 
        The goal point for current step is at max 150 meter away from current position. 
        """
        lookahead_distance = max(self.lookahead_distance_min, min(ego_pose.v * self.lookahead_time, self.lookahead_distance_max))
        
        goal_wp_estimate = ego_wp.next(lookahead_distance)[0]

        # find the closest point in the route to the goal_wp
        min_dist = float('inf')
        for wp, _ in route:
            if self._cal_distance(wp, goal_wp_estimate) < min_dist:
                min_dist = self._cal_distance(wp, goal_wp_estimate)
                goal_wp = wp
        
        return goal_wp
    
    def _cal_distance(self, wp1, wp2):
        return np.sqrt((wp1.transform.location.x - wp2.transform.location.x)**2 + 
                       (wp1.transform.location.y - wp2.transform.location.y)**2)
    

