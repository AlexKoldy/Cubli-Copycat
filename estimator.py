import numpy as np
from control.matlab import *
from integrators import get_integrator

class Estimator:
    def __init__(self, x):
        self.x_e = x
    
'''Uses LQE (Kalman Filter) to estimate state given system parameters'''
class LQE(Estimator):
    def __init__(self, A, B, C, G, x, dt, std):
        '''System parameters'''
        self.std = std
        self.dt = dt
        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.Q = self.std**2 * np.eye(3)
        self.R = self.std**2 * 1e-1
        self.L = lqe(self.A, self.G, self.C, self.Q, self.R)[0]

        self.intg = get_integrator(self.dt, self.eom)
        
        Estimator.__init__(self, x)
    
    '''Equation of motion: x_e_dot = f(t, x_e, u)'''
    def eom(self, t, x, u):
        x_e_dot = self.A @ self.x_e + self.B * u + self.L * (self.y - self.C @ self.x_e)
        
        return x_e_dot
    
    '''Update estimated state and return it to the system'''
    def update(self, t, u, y):
        self.y = y
        self.x_e = self.intg.step(t, self.x_e, u)
        
        return self.x_e
    
    
        

        
        
        