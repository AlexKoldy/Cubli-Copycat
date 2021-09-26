import numpy as np

class Cubli():
	def __init__(self):
		'''URDF path for simulation'''
		self.urdf_path = 'urdf/cubli.urdf'

		'''Robot state'''
		self.q = np.array([[0], # housing yaw [rad]
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
		self.m_h = 0 # mass of housing [kg]
		self.m_w = [0, 0, 0] # masses of each flywheel [kg]
		self.Theta_h = np.zeros((3, 3)) # housing inertia tensor [kg-m^2]
		self.Theta_w = [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3))] # inertia tensors of each wheel [kg-m^2]

	'''
	Equation of motion:
	q_dot = f(t, q, u)
	Contains governing equations of the system
	'''
	def f(self, t, q, u):
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

		'''Calculation parameters'''
		M = self.m_h*symmetric_skew(self.r_h) + sum([self.m_w[i]*symmetric_skew(self.r_w[i]) for i in range(0, 3)])
		Theta = self.Theta_h - self.m_h*symmetric_skew(self.r_h)**2 + sum([self.Theta_w[i] - self.m_w[i]*symmetric_skew(self.r_w[i])**2 for i in range(0, 3)])
		Theta_w = np.diag([self.Theta_w[0][0, 0], self.Theta_w[1][1, 1], self.Theta_w[2][2, 2]])
		Theta_hat = Theta - Theta_w

		'''Angular acceleration of robot housing and reaction wheels'''
		omega_h_dot = (1/Theta_hat)*(np.cross(self.Theta*self.q[3:6], self.q[3:6]) + M*g + np.cross(Theta_w*self.q[6:], self.q[6:]) - (self.K_m*u - self.C_w*self.q[6:]))
		omega_w_dot = (1/Theta_w)*(self.K_m*u - self.C_w*self.q[6:] - Theta_w*omega_h_dot)

		q_dot = np.array([[self.q[3]],
						  [self.q[4]],
						  [self.q[5]],
						  [omega_h_dot[0]],
						  [omega_h_dot[1]],
						  [omega_h_dot[2]],
						  [omega_w_dot[0]],
						  [omega_w_dot[1]],
						  [omega_w_dot[2]]
		])
		return q_dot




# TODO: Move simulation stuff outa here!
import pybullet as p
import time
import pybullet_data

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF("plane.urdf")
cubli_pos_0 = [0.5, 0.5, 0.5]
cubli_orien_0 = p.getQuaternionFromEuler([0, 0, 0])
box_id = p.loadURDF("urdf/cubli.urdf", cubli_pos_0, cubli_orien_0)
for i in range (10000):
	p.stepSimulation()
	time.sleep(1.0/240.0)
cubli_pos_0, cubli_orien_0 = p.getBasePositionAndOrientation(box_id)
print(cubli_pos_0, cubli_orien_0)
p.disconnect()