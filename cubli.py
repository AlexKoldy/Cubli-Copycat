import numpy as np
from integrators import get_integrator
from control.matlab import *
from controller import LQR
from estimator import LQE

'''External parameters'''
g = 9.8 # m/s^2; acceleration due to gravity

class Cubli():
    def __init__(self,
                 theta_b_0, # rad; tilt angle of the pendulum body
                 theta_b_dot_0, # rad/s; angular rate of the pendulum body
                 theta_w_dot_0, # rad/s; angular rate of the momentum wheel with respect to the body
                 linear): # choose whether or not simulation follows linear dynamics
        self.dt = 0.01 # s; timestep
        
        '''States of the system'''
        # True state
        self.x = np.array([[theta_b_0], 
                           [theta_b_dot_0], 
                           [theta_w_dot_0]])
        
        # Estimated state
        self.x_e = np.array([[theta_b_0], 
                           [theta_b_dot_0], 
                           [theta_w_dot_0]])
        
        self.linear = linear # Linear dynamics flag (non-linear dynamics follow actual system dynamics)

        ''' System parameters'''
        self.m_b = 0.419 # kg; mass of pendulum body
        self.m_w = 0.204 # kg; mass of wheel
        self.I_b = 3.34 * 10**-3 # kgm^2; moment of inertia of the pendulum body around the pivot point
        self.I_w = 0.57 * 10**-3 # kgm^2; moment of inertia of the wheel and the motor rotor around the rotational axis of the motor
        self.l = 0.085 # m; distance between motor axis and pivot point
        self.l_b = 0.075 # m; distance between the center of mass of the pendulum body and the pivot point
        self.C_b = 1.02 * 10**-3 # kgm^2/s; dynamic friction coefficient of the pendulum body
        self.C_w = 0.05 * 10**-3 # kgm^2/s; dynamic friction coefficient of the wheel
        self.K_m = 25.1 * 10**-3 # Nm/A; torque constant of DC motor
        
        self.i = 5 # A; maximum current that can be supplied to the motor by a controller
        self.omega_w_max = 548.7315162 # rad/s; maximum angular velocity of wheel
        
        '''Linear dynamics A, B & C matrices for state-space'''
        self.A = np.array([[0, 1, 0],
                           [((self.m_b*self.l_b + self.m_w*self.l) * g) / (self.I_b + self.m_w*self.l**2), -self.C_b / (self.I_b + self.m_w*self.l**2), self.C_w / (self.I_b + self.m_w*self.l**2)],
                           [(-(self.m_b*self.l_b + self.m_w*self.l) * g) / (self.I_b + self.m_w*self.l**2), self.C_b / (self.I_b + self.m_w*self.l**2), (-self.C_w * (self.I_b + self.I_w + self.m_w*self.l**2)) / (self.I_w * (self.I_b + self.m_w*self.l**2))]])
        self.B = np.array([[0],
                           [-self.K_m / (self.I_b + self.m_w*self.l**2)],
                           [(self.K_m * (self.I_b + self.I_w + self.m_w*self.l**2)) / (self.I_w * (self.I_b + self.m_w*self.l**2))]])
        self.C = np.array([1, 0, 0])       
        
        '''Noise'''
        # Process noise
        self.G = np.array([[0, 0, 0],
                           [0, 1 / (self.I_b + self.m_w*self.l**2), 0],
                           [0, 0, 1 / (self.I_b + self.m_w*self.l**2)]])
        self.w = np.random.uniform(-0.1, 0.1, size = (3, 1))
        
        # Sensor noise
        self.v = np.random.uniform(-0.1, 0.1)
        
        self.intg = get_integrator(self.dt, self.eom)
        
        '''Start-up sequence parameters'''
        self.omega_w_jump = ((2 - 2**(1/2)) * (self.I_w + self.I_b + self.m_w * self.l**2) / (self.I_w**2) * (self.m_b * self.l_b + self.m_w * self.l) * g)**(1/2) # rad/s; angular momentum wheel needed to perform jump-up
        self.jumped = False # flag to check whether or not the cubli has attempted a jump-up
        
        '''Controller and Estimator'''
        # LQR Controller
        self.u = 0 # A
        self.controller = LQR(self.A, self.B, self.i)
        self.estimator = LQE(self.A, self.B, self.C, self.G, self.x, self.dt)
    
    '''Equation of motion: x_dot = f(t, x, u)'''
    def eom(self, t, x, u):
        '''Check for linear/nonlinear dynamics'''
        if self.linear == True:
            x_dot = self.A @ x + self.B * u + self.G @ self.w
        elif self.linear == False:
            theta_b_ddot = ((self.m_b*self.l_b + self.m_w*self.l) * g * np.sin(float(x[0])) - self.K_m*u - self.C_b*x[1] + self.C_w*x[2]) / (self.I_b + self.m_w*self.l**2) + (self.w[1] / (self.I_b + self.m_w*self.l**2))
            theta_w_ddot = ((self.I_b + self.I_w + self.m_w*self.l**2) * (self.K_m*u - self.C_w*x[2])) / (self.I_w * (self.I_b + self.m_w*self.l**2)) - ((self.m_b*self.l_b + self.m_w*self.l) * g * np.sin(float(x[0])) - self.C_b*x[1]) / (self.I_b + self.m_w*self.l**2) + (self.w[2] / (self.I_b + self.m_w*self.l**2))
            x_dot = np.array([[float(x[1])],
                              [float(theta_b_ddot)],
                              [float(theta_w_ddot)]])
           
        return x_dot
     
    '''Updates the state of the system given a specific input (typically from a controller)'''
    def update(self, t):        
        '''Update Cubli's controller'''
        u = self.controller.update(self.x)
        self.u = u
        
        '''Check if Cubli has jumped up:
        if not, begin jump sequence'''
        if self.jumped == False:
            self.jump()
        
        '''Update the true state'''
        self.x = self.intg.step(t, self.x, u)
        
        '''Perform estimation on state'''
        y = self.C @ self.x + self.v
        self.x_e = self.estimator.update(t, u, y)

        '''Establish surface for Cubli:
        When the pendulum body is either at -45 degrees or 45 degrees,
        it is laying on a surface'''
        if (self.x[0] < -np.pi / 4):
            self.x[0] = -np.pi / 4
            self.x[1] = 0
        elif (self.x[0] > np. pi / 4):
            self.x[0] = np.pi / 4
            self.x[1] = 0
        else:
            '''Update noise'''
            self.w = np.random.uniform(-0.05, 0.05, size = (3, 1))
            self.v = np.random.uniform(-0.1, 0.1)
            
        '''Check for maximum angular velocity of wheel'''
        if float(self.x[2]) > self.omega_w_max:
            self.x[2] = self.omega_w_max
        elif float(self.x[2]) < -self.omega_w_max:
            self.x[2] = -self.omega_w_max
                
    '''Shoots the cubli up by using conservation of angular momentum'''
    def jump(self):
        if abs(float(self.x[2])) >= self.omega_w_jump:
            self.x[0] = (-np.pi/4) + 0.00001
            self.x[1] = self.I_w * abs(float(self.x[2])) / (self.I_b + self.I_w + self.m_w*self.l**2)
            self.x[2] = 0
            self.jumped = True
                
        

        
        


        
            

            
        
    