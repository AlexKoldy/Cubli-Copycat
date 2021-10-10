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
		robot_pos_0 = [0, 0, 0]
		robot_orien_0 = p.getQuaternionFromEuler(list(robot.q[:3]))
		self.robot_id = p.loadURDF(robot.urdf_path, robot_pos_0, robot_orien_0)
		print(self.robot.name + 'robot created!')

		'''Other parameters for testing'''
		self.is_testing = False


	def simulate(self):
		if self.is_testing:
			target_velocity_slider = p.addUserDebugParameter("Wheel Velocity", -100, 100, 0)
		
		while(1):
			self.update()

			p.stepSimulation()
			time.sleep(1.0/100.0)
			

	def update(self):
		p.setJointMotorControl2(self.robot_id, 0, p.VELOCITY_CONTROL, targetVelocity=0)


	def __call__(self):
		self.simulate()

cubli = Cubli()
simulator = Simulator(cubli)
simulator()