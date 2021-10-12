import numpy as np
import pybullet as p

from integrators import get_integrator
from controllers import LQR

class Cubli():
	def __init__(self):
		'''URDF path for simulation'''
		self.urdf_path = 'urdf/cubli.urdf'

		'''Robot name & id'''
		self.name = 'Cubli'
		self.id = 0

		'''Robot state'''
		self.q = np.array([[np.pi/4], # housing yaw [rad]
						   [0], # housing pitch [rad]
						   [0], # housing roll [rad]
						   [0], # housing yaw rate [rad/s]
						   [0], # housing pitch rate [rad/s]
						   [0], # housing roll rate [rad/s]
						   [0], # flywheel 1 angular velocity [rad/s]
						   [0], # flywheel 2 angular velocity [rad/s]
						   [0]  # flywheel 3 angular velocity [rad/s]
		]).flatten()

		'''Robot parameters'''
		self.K_m = 25.1 * 10**-3 # motor torque constant [Nm/A]
		self.C_w = 0.5 * 10**-3 # dynamic friction coefficient of wheel [kgm^2/s]
		self.m_h = 1 # mass of housing [kg]
		self.m_w = [1, 1, 1] # masses of each flywheel [kg]
		self.Theta_h = np.eye(3) # housing inertia tensor [kg-m^2]
		self.Theta_w = [np.eye(3), np.eye(3), np.eye(3)] # inertia tensors of each wheel [kg-m^2]

		'''Control & simulation parameters'''
		self.linearize() # establish linearized matrices
		self.dt = 1/100
		self.intg = get_integrator(self.dt, self.f)
		self.controller = LQR(self.A, self.B)
		self.is_linear = True

	'''
	Equation of motion:
	q_dot = f(t, q, u)
	Contains governing equations of the system
	'''
	def f(self, t, q, u):
		'''Gravity'''
		g = -9.81 # acceleration due to gravity [m/s^2]
		
		'''Calculation parameters'''
		M, Theta, Theta_w, Theta_hat = self.parameters()
		
		'''Check if Cubli will be using linearized dynamics'''
		if self.is_linear:
			'''Linearized angular acceleration of robot housing and reaction wheels'''
			q_dot = self.A@self.q + self.B@u
		else:
			'''Nonlinear angular acceleration of robot housing and reaction wheels'''
			omega_h_dot = (1/Theta_hat)*(np.cross(self.Theta*q[3:6], q[3:6]) + M*g + np.cross(Theta_w*q[6:], q[6:]) - (self.K_m*u - self.C_w*q[6:]))
			omega_w_dot = (1/Theta_w)*(self.K_m*u - self.C_w*q[6:] - Theta_w*omega_h_dot)

			q_dot = np.array([[self.q[3]],
							[self.q[4]],
							[self.q[5]],
							[omega_h_dot[0]],
							[omega_h_dot[1]],
							[omega_h_dot[2]],
							[omega_w_dot[0]],
							[omega_w_dot[1]],
							[omega_w_dot[2]]
			]).flatten()

		return q_dot

	'''
	Returns parameters used for calculating dynamics:
	[M, Theta, Theta_w, Theta_hat]
	'''
	def parameters(self):
		'''
		Returns skew-symmetric matrix of vector v such that:
		np.cross(v_1, v_2) = np.dot(skew(v_1), v_2)
		'''
		def symmetric_skew(v):
			v_tilde = np.array([[0, -v[2], v[1]],
							    [v[2], 0, -v[0]],
								[-v[1], v[0], 0]
			])
			return v_tilde

		M = self.m_h*symmetric_skew(self.r_h) + sum([self.m_w[i]*symmetric_skew(self.r_w[i]) for i in range(0, 3)])
		Theta = self.Theta_h - self.m_h*symmetric_skew(self.r_h)**2 + sum([self.Theta_w[i] - self.m_w[i]*symmetric_skew(self.r_w[i])**2 for i in range(0, 3)])
		Theta_w = np.diag([self.Theta_w[0][0, 0], self.Theta_w[1][1, 1], self.Theta_w[2][2, 2]])
		Theta_hat = Theta - Theta_w

		return [M, Theta, Theta_w, Theta_hat]
	
	'''
	Establishes A and B matrices for linear dynamics and control
	'''
	def linearize(self):
		'''Calculation parameters'''
		M, Theta, Theta_w, Theta_hat = self.parameters()

		'''Linearized dynamics'''
		dg_dphi = np.array([[0, np.sin(self.beta_0), 0],
							[0, np.sin(self.beta_0)*np.sin(self.gamma_0), -np.cos(self.beta_0)*np.cos(self.gamma_0)],
							[0, np.sin(self.beta_0)*np.cos(self.gamma_0), np.cos(self.beta_0)*np.sin(self.gamma_0)]
		])
		F = np.array([[0, np.sin(self.gamma_0)/np.cos(self.gamma_0), np.cos(self.gamma_0)/np.cos(self.beta_0)],
					  [0, np.cos(self.gamma_0), -np.sin(self.gamma_0)],
					  [1, np.sin(self.gamma_0)*np.sin(self.beta_0)/np.cos(self.beta_0), np.cos(self.gamma_0)*np.sin(self.beta_0)/np.cos(self.beta_0)]
		])
		'''Establish A & B Matrices: concatenate each "row" first'''
		A = np.empty((9, 9))
		A[:3, :] = np.concatenate((np.zeros((3, 3)), F, np.zeros((3,3))), axis=1)
		A[3:6, :] = np.concatenate((np.linalg.inv(Theta_hat)@M@dg_dphi, np.zeros((3, 3)), self.C_w*np.linalg.inv(Theta_hat)), axis=1)
		A[6:, :] = np.concatenate((-np.linalg.inv(Theta_hat)@M@dg_dphi, np.zeros((3, 3)), -self.C_w*(np.linalg.inv(Theta_hat)- np.linalg.inv(Theta_w))), axis=1)
		B = np.empty((9, 3))
		B[:3, :] = np.zeros((3, 3))
		B[3:6, :] = -np.linalg.inv(Theta_hat)*self.K_m
		B[6:, :] = (np.linalg.inv(Theta_hat) + np.linalg.inv(Theta_w))*self.K_m

		self.A = A
		self.B = B

	def update(self, t):
		#TODO: Implement state estimation here
		'''Get state of robot'''
		_, orien_h = p.getBasePositionAndOrientation(self.id)
		_, velocity_h = p.getBaseVelocity(self.id)
		_, velocity_w_1, _, _ = p.getJointState(self.id, 0)

		print(orien_h)

		'''Get current input to motor'''
		u = self.controller.update(q)

		'''Update state'''
		q = self.intg.step(t, self.q, u)
		omega_w_1 = q[6]
		omega_w_2 = q[7]
		omega_w_3 = q[8]

		#q = p.getEulerFromQuaternion(orien)

		p.setJointMotorControl2(self.id, 0, p.VELOCITY_CONTROL, targetVelocity=0)
		#self.q = self.intg.step(t, self.q, self.u)
