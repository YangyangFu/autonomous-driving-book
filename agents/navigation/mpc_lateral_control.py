"""
Model Predictive Control (MPC) Lateral Controller
"""
import numpy as np 
import carla 
Waypoint = carla.Waypoint

from agents.navigation.trajectory import Path, Trajectory
from agents.navigation.cubic_spiral_generator import CubicSpiral
from agents.navigation.linear_interpolation import LinearInterpolation

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

        self.model = self._initiate_vehichle_model(model_name, model_params)

    def run(self, state, trajectory):
        """
        Run MPC controller
        """

    # since the reference trajectory does not take into account the current velocity of the vehicle,
    # we need calculate the trajectory velocity considering the longitudinal dynamics
    def set_reference_trajectory(self, trajectory: Trajectory):
        """
        Set reference trajectory from the planner
        """
        self.reference_trajectory = trajectory 

    def calculate_initial_state(self):
        """
        Calculate initial state
        """
        pass

    def update_state_for_delay_compensation(self):
        """
        Update state for delay compensation
        """
        pass

    def resample_trajectory_for_by_time(self, in_traj: Trajectory) -> Trajectory:
        """
        Resample reference trajectory for with mpc sampling time
        """

        # calculate the output time
        out_time = np.arange(in_traj.t[0], in_traj.t[-1], self.dt)

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

    
    def update_state_space_matrix(self):
        """
        Update state space matrix
        """
        
        #return Ad, Bd, wd, Cd  
        pass 

    def run_optimization(self):
        """
        Run optimization to return optimal control inputs
        """
        pass

    def apply_input_limits(self):
        """
        Apply input limits 
        """
        pass

    def calculate_predicted_trajectory(self):
        """
        Calculate predicted trajectory
        """
        pass



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

    def generate_mpc_trajectory(self, goal_waypoint) -> MPCTrajectory:
        """
        Generate MPC trajectory
        
        """
        # generate cubic spirals

        # generate velocity profile

        # generate trajectory
        pass 

    def _initiate_vehichle_model(self):
        pass

