from cubli import Cubli

import pybullet as p
import time
import pybullet_data

class Simulator():
	def __init__(self, robot):
		'''Initialize the PyBullet simulator'''
		physics_client = p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.setGravity(0, 0, -9.8)
		plane_id = p.loadURDF("plane.urdf")
		print('PyBullet initialized!')

		'''Create robot'''
		self.robot = robot
		robot_pos_0 = [0, 0, 0.107]
		robot_orien_0 = p.getQuaternionFromEuler(list(robot.q[:3]))
		robot_id = p.loadURDF(robot.urdf_path, robot_pos_0, robot_orien_0)
		self.robot.id = robot_id
		print(self.robot.name + 'robot created!')

		'''Simulation parameters'''
		self.t = 0 # time [s]
		self.dt = 1.0/100.0 # time-step [s]

		'''Other parameters for testing'''
		self.is_testing = False

	def simulate(self):
		if self.is_testing:
			target_velocity_slider = p.addUserDebugParameter("Wheel Velocity", -100, 100, 0)
		
		while(1):
			self.update(self.t)
			self.t += self.dt
			p.stepSimulation()
			time.sleep(self.dt)
			
	def update(self, t):
		self.robot.update(t)

	def __call__(self):
		self.simulate()

cubli = Cubli()
simulator = Simulator(cubli)
simulator()