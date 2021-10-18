import numpy as np
from control.matlab import lqr

class LQR():
	def __init__(self, A, B):
		self.A = A
		self.B = B
		Q_shape = A.shape
		self.Q = self.tune_Q([1, 1, 1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01], Q_shape)
		self.R = 1
			
	def tune_Q(self, parameters, shape):
		if len(parameters) != shape[0]:
			print("Error: number of parameters does not match shape of A matrix")
		else:
			Q = np.eye(shape[0])
			i = 0

			for row in Q:
				row *= parameters[i]

			return Q
	
	def update(self, x):
		K = lqr(self.A, self.B, self.Q, self.R)[0]
		u = -K @ x
		return u
