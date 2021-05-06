import numpy as np
from control.matlab import *

class Estimator:
    def __init__(self, x):
        self.x_e = x
        
class LQE(Estimator):
    def __init__(self, A, C, G, w, v, x):
        self.A = A
        self.C = C
        self.G = G
        
        self.Q = np.cov(w, w.T)
        self.R = np.cov(v, v.T)
        
        self.L = lqe(A, G, C, QN, RN)
        
        Estimator.__init__(self, x)
        
        
        
    def update(self):
        
        
        