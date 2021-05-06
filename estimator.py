import numpy as np
from control.matlab import *
from integrators import get_integrator

class Estimator:
    def __init__(self, x):
        self.x_e = x
        
class LQE(Estimator):
    def __init__(self, A, B, C, G, x, dt):
        self.dt = dt
        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.Q = np.eye(3)
        self.R = 0.001
        self.L = lqe(self.A, self.G, self.C, self.Q, self.R)[0]

        self.intg = get_integrator(self.dt, self.eom)
        
        Estimator.__init__(self, x)
        
    def eom(self, t, x, u):
        x_e_dot = self.A @ self.x_e + self.B * u + self.L * (self.y - self.C @ self.x_e)
        
        return x_e_dot
        
    def update(self, t, u, y):
        self.y = y
        x_e = self.intg.step(t, self.x_e, u)

        return x_e
    
    
        

        
        
        