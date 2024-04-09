import numpy as np 

class BasicModel:
    def __init__(self, name, state_dim, control_dim, output_dim, wheelbase):
        self.name = name # model name
        self.state_dim = state_dim # state dimension
        self.control_dim = control_dim # action dimension
        self.output_dim = output_dim # output dimension
        self.wheelbase = wheelbase # wheelbase in m
    
    def get_state_dim(self) -> int:
        return self.state_dim
    
    def get_control_dim(self) -> int:
        return self.control_dim

    def get_output_dim(self) -> int:
        return self.output_dim

    def get_wheelbase(self) -> float:
        return self.wheelbase
    
    def set_velocity(self, velocity) -> None:
        self.velocity = velocity

    def set_curvature(self, curvature) -> None:
        self.curvature = curvature


class DynamicBicycleModel(BasicModel):
    def __init__(self, lf, lr, mass, cf, cr):
        super().__init__(name = "dynamic_bicycle", 
                         state_dim = 4, 
                         control_dim = 1, 
                         output_dim = 2, 
                         wheelbase = lf + lr)

        # model specific parameters
        self.lf = lf
        self.lr = lr
        self.mass = mass
        self.cf = cf
        self.cr = cr

        # computed parameters
        self.mass_f = lr / (lf + lr) * mass
        self.mass_r = lf / (lf + lr) * mass

        self.iz = self.lf * self.lf * self.mass_f + self.lr * self.lr * self.mass_r
        print("Inertia: ", self.iz) 
        
    def update_matrix(self):
        """
        Update continuous state space matrix
        
        State space model:
            dx = A*x + B*u + W*r(s)
            y = C*x
        where:
            x: [e, de, th, dth]
            u: [delta]
            y: [e, th]

            A = [[0, 1, 0, 0],
                 [0, -(cf+cr)/m/vx, (cf+cr)/m, (lr*cr-lf*cf)/m/vx],
                 [0, 0, 0, 1],
                 [0, (lr*cr-lf*cf)/iz/vx, (lf*cf-lr*cr)/iz, -(lf^2*cf+lr^2*cr)/iz/vx]]
            
            B = [[0],
                 [cf/m],
                 [0],
                 [lf*cf/iz]]
            W = [[0],
                 [(lr*cr-lf*cf)/m/vx - vx],
                 [0],
                 [-(lf^2*cf+lr^2*cr)/iz/vx]]
            C = [[1, 0, 0, 0],
                 [0, 0, 1, 0]]
        """

        self._A = np.zeros((self.state_dim, self.state_dim))
        self._A[0, 1] = 1
        self._A[1, 1] = -(self.cf + self.cr) / self.mass / self.velocity
        self._A[1, 2] = (self.cf + self.cr) / self.mass
        self._A[1, 3] = (self.lr * self.cr - self.lf * self.cf) / self.mass / self.velocity
        self._A[2, 3] = 1
        self._A[3, 1] = (self.lr * self.cr - self.lf * self.cf) / self.iz / self.velocity
        self._A[3, 2] = (self.lf * self.cf - self.lr * self.cr) / self.iz
        self._A[3, 3] = -(self.lf * self.lf * self.cf + self.lr * self.lr * self.cr) / self.iz / self.velocity

        self._B = np.zeros((self.state_dim, self.control_dim))
        self._B[1, 0] = self.cf / self.mass
        self._B[3, 0] = self.lf * self.cf / self.iz
        
        self._W = np.zeros((self.state_dim, 1))
        self._W[1, 0] = (self.lr * self.cr - self.lf * self.cf) / self.mass / self.velocity - self.velocity
        self._W[3, 0] = -(self.lf * self.lf * self.cf + self.lr * self.lr * self.cr) / self.iz / self.velocity

        self._C = np.zeros((self.output_dim, self.state_dim))
        self._C[0, 0] = 1
        self._C[1, 2] = 1

    def update_discrete_matrix(self, dt):
        """
        Update discrete state space matrix based on continuous state space matrix
        
        State-space::
            x_{k+1} = Ad*x_k + Bd*u_k + Wd
            y_k = Cd*x_k
        
        Discretization::
            Ad = exp(A*dt)
            Bd = A^-1*(Ad - I)*B
            Wd = A^-1*(Ad - I)*W
            Cd = C
        """
        I = np.eye(self.state_dim)
        Ainv = np.linalg.inv(I - dt * 0.5 * self._A)
        self.Ad = Ainv  @ (I + dt * 0.5 * self._A) # bilinear discretization
        self.Bd = (Ainv * dt) @ self._B
        self.Wd = (Ainv * dt * self.curvature * self.velocity) @ self._W
        self.Cd = self._C




    
