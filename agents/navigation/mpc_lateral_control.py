"""
Model Predictive Control (MPC) Lateral Controller
"""

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


class MPCLateralControl():
    def __init__(self):
        pass 

    def run_step(self):
        pass

    def extract_trajectory(self):

        pass

    def _initiate_vehichle_model(self):
        pass

