import numpy as np
from integrators import get_integrator
from control.matlab import *

        
g = 9.8 # m/s^2; gravity

class Cubli():
    def __init__(self,
                 theta_b_0, # rad; tilt angle of the pendulum body
                 theta_b_dot_0, # rad/s; angular rate of the pendulum body
                 theta_w_dot_0): # rad/s; angular rate of the momentum wheel with respect to the body
        '''TODO: Move this out'''    
        self.t = 0 # s; current time
        self.dt = 0.01 # s; timestep

        self.m_b = 0.419 # kg; mass of pendulum body
        self.m_w = 0.204 # kg; mass of wheel
        self.I_b = 3.34 * 10**-3 # kgm^2; moment of inertia of the pendulum body around the pivot point
        self.I_w = 0.57 * 10**-3 # kgm^2; moment of inertia of the wheel and the motor rotor around the rotational axis of the motor
        self.l = 0.085 # m; distance between motor axis and pivot point
        self.l_b = 0.075 # m; distance between the center of mass of the pendulum body and the pivot point
        self.C_b = 1.02 * 10**-3 # kgm^2/s; dynamic friction coefficient of the pendulum body
        self.C_w = 0.05 * 10**-3 # kgm^2/s; dynamic friction coefficient of the wheel
        self.K_m = 25.1 * 10**-3 # Nm/A; torque constant of DC motor
        
        self.I_stall = 23.3 # A; Stall current
        self.omega_w_max = 548.7315162 # rad/s; maximum angular velocity of wheel
        
        self.A = np.array([[0, 1, 0],
                           [((self.m_b*self.l_b + self.m_w*self.l) * g) / (self.I_b + self.m_w*self.l**2), -self.C_b / (self.I_b + self.m_w*self.l**2), self.C_w / (self.I_b + self.m_w*self.l**2)],
                           [(-(self.m_b*self.l_b + self.m_w*self.l) * g) / (self.I_b + self.m_w*self.l**2), self.C_b / (self.I_b + self.m_w*self.l**2), (-self.C_w * (self.I_b + self.I_w + self.m_w*self.l**2)) / (self.I_w * (self.I_b + self.m_w*self.l**2))]])
        self.B = np.array([[0],
                           [-self.K_m / (self.I_b + self.m_w*self.l**2)],
                           [(self.K_m * (self.I_b + self.I_w + self.m_w*self.l**2)) / (self.I_w * (self.I_b + self.m_w*self.l**2))]])

        # State Vector
        self.x = np.array([[theta_b_0], 
                           [theta_b_dot_0], 
                           [theta_w_dot_0]])

        self.intg = get_integrator(self.dt, self.eom)
        
        '''TODO: Place this elsewhere'''
        omega_w = ((2 - 2**(1/2)) * (self.I_w + self.I_b + self.m_w * self.l**2) / (self.I_w**2) * (self.m_b * self.l_b + self.m_w * self.l) * g)**(1/2)
                
    def eom(self, t, x, u, linear):
        if linear == True:
            x_dot = self.A @ x + self.B * u
        elif linear == False:
            theta_b_ddot = ((self.m_b*self.l_b + self.m_w*self.l) * g * np.sin(float(x[0])) - self.K_m*u - self.C_b*x[1] + self.C_w*x[2]) / (self.I_b + self.m_w*self.l**2)
            theta_w_ddot = ((self.I_b + self.I_w + self.m_w*self.l**2) * (self.K_m*u - self.C_w*x[2])) / (self.I_w * (self.I_b + self.m_w*self.l**2)) - ((self.m_b*self.l_b + self.m_w*self.l) * g * np.sin(float(x[0])) - self.C_b*x[1]) / (self.I_b + self.m_w*self.l**2)
            x_dot = np.array([[float(x[1])],
                              [float(x[2])],
                              [float(theta_b_ddot)],
                              [float(theta_w_ddot)]])
            
        return x_dot
        
    def update(self, u, linear):
        '''Check for linear/nonlinear dynamics'''
        if linear == True:
            self.x = self.intg.step(self.t, self.x, u, True)
        elif linear == False:
            x = self.x
            x = np.insert(x, 1, 0, axis = 0)
            x = self.intg.step(self.t, x, u, False)
            theta_b = float(x[0])
            theta_b_dot = float(x[2])
            theta_w_dot = float(x[3])
            self.x = np.array([[theta_b],
                               [theta_b_dot],
                               [theta_w_dot]])

        '''Establish surface for Cubli:
        When the pendulum body is either at -45 degrees or 45 degrees,
        it is laying on a surface'''
        if (self.x[0] < -np.pi / 4):
            self.x[0] = -np.pi / 4
            self.x[1] = 0
        elif (self.x[0] > np. pi / 4):
            self.x[0] = np.pi / 4
            self.x[1] = 0
            
        '''Check for maximum angular velocity of wheel'''
        if float(self.x[2]) > self.omega_w_max:
            self.x[2] = self.omega_w_max
        elif float(self.x[2]) < -self.omega_w_max:
            self.x[2] = -self.omega_w_max
        
        self.t += self.dt

        
            

            
        
    