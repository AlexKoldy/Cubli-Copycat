import numpy as np
from control.matlab import *

class Controller:
    def __init__(self, u_max):
        self.u_sat = u_max
    
    def saturate(self, u):
        if self.u_sat > u:
            return self.u_sat
        elif u < -self.u_sat:
            return -self.u_sat
        else:
            return u
        
class LQR(Controller):
    def __init__(self, A, B, u_max):
        self.A = A
        self.B = B
        self.Q = np.array([[0.001, 0, 0],
                           [0, 0.001, 0],
                           [0, 0, 0]])
        self.R = 1
        Controller.__init__(self, u_max)
        
    def update(self, x):
        K = lqr(self.A, self.B, self.Q, self.R)[0]
        u = -K @ x
        #u = self.saturate(u)
        return u
        
                
        