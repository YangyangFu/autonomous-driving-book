"""
Model Predictive Control (MPC) Lateral Controller
"""
import carla 
Waypoint = carla.Waypoint

from agents.navigation.cubic_spiral_generator import CubicSpiral

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
    def set_reference_trajectory(self, trajectory):
        """
        Set reference trajectory
        """
        pass 

    def generate_trajectory_with_velocity(self):
        """
        Generate trajectory with velocity
        """
        pass

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

    def resample_trajectory_for_mpc(self):
        """
        Resample reference trajectory for with mpc sampling time
        """
        pass

    def update_state_space_matrix(self):
        """
        Update state space matrix
        """
        
        return Ad, Bd, wd, Cd 
    
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


class MPCTrajectory:
    """
    Model Predictive Control (MPC) Trajectory
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

