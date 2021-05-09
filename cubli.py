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
                 linear = False): # choose whether or not simulation follows linear dynamics
        '''System simulation parameters'''
        self.dt = 0.01 # s; timestep of real-time system
        self.intg = get_integrator(self.dt, self.eom)
        self.p = 0 # counts how many iterations until the controller turns on
        
        # Hardware limitations
        self.dt_s = 0.02 # s; timestep of sampling hardware
        self.num_inc_max = self.dt_s / self.dt # number of real-time increments before a sample can be taken
        self.num_inc = 0 # increment counter
        
        '''States of the system'''
        # True state
        self.x = np.array([[theta_b_0], 
                           [theta_b_dot_0], 
                           [theta_w_dot_0]])
        
        # Estimated state
        self.x_e = np.array([[theta_b_0], 
                           [theta_b_dot_0], 
                           [theta_w_dot_0]])
        
        ''' System parameters'''
        # Mechanical parameters
        self.m_b = 0.419 # kg; mass of pendulum body
        self.m_w = 0.204 # kg; mass of wheel
        self.I_b = 3.34 * 10**-3 # kgm^2; moment of inertia of the pendulum body around the pivot point
        self.I_w = 0.57 * 10**-3 # kgm^2; moment of inertia of the wheel and the motor rotor around the rotational axis of the motor
        self.l = 0.085 # m; distance between motor axis and pivot point
        self.l_b = 0.075 # m; distance between the center of mass of the pendulum body and the pivot point
        self.C_b = 1.02 * 10**-3 # kgm^2/s; dynamic friction coefficient of the pendulum body
        self.C_w = 0.05 * 10**-3 # kgm^2/s; dynamic friction coefficient of the wheel
        self.K_m = 25.1 * 10**-3 # Nm/A; torque constant of DC motor
        
        # Saturation parameters
        self.i = 5 # A; maximum current that can be supplied to the motor by a controller
        self.omega_w_max = 548.7315162 # rad/s; maximum angular velocity of wheel
        
        # Start-up sequence parameters
        self.omega_w_jump = ((2 - 2**(1/2)) * (self.I_w + self.I_b + self.m_w * self.l**2) / (self.I_w**2) * (self.m_b * self.l_b + self.m_w * self.l) * g)**(1/2) # rad/s; angular momentum wheel needed to perform jump-up
        self.jumped = False # flag to check whether or not the cubli has attempted a jump-up
        
        '''Linear dynamics A, B & C matrices for state-space'''
        self.linear = linear # Linear dynamics flag (non-linear dynamics follow actual system dynamics)
        self.A = np.array([[0, 1, 0],
                           [((self.m_b*self.l_b + self.m_w*self.l) * g) / (self.I_b + self.m_w*self.l**2), -self.C_b / (self.I_b + self.m_w*self.l**2), self.C_w / (self.I_b + self.m_w*self.l**2)],
                           [(-(self.m_b*self.l_b + self.m_w*self.l) * g) / (self.I_b + self.m_w*self.l**2), self.C_b / (self.I_b + self.m_w*self.l**2), (-self.C_w * (self.I_b + self.I_w + self.m_w*self.l**2)) / (self.I_w * (self.I_b + self.m_w*self.l**2))]])
        self.B = np.array([[0],
                           [-self.K_m / (self.I_b + self.m_w*self.l**2)],
                           [(self.K_m * (self.I_b + self.I_w + self.m_w*self.l**2)) / (self.I_w * (self.I_b + self.m_w*self.l**2))]])
        self.C = np.array([1, 0, 0])       
        
        '''Noise'''
        self.std_noise = 0.01 # Process and sensor noise standard deviation
        
        # Process noise
        self.G = np.array([[0, 0, 0],
                           [0, 1 / (self.I_b + self.m_w*self.l**2), 0],
                           [0, 0, 1 / (self.I_b + self.m_w*self.l**2)]])
        self.w = np.random.normal(0, self.std_noise, size = (3, 1)) # Nm; disturbance torques
        
        # Sensor noise
        self.v = np.random.normal(0, self.std_noise)
                        
        '''Controller and Estimator (LQG)'''
        self.control_on = False # flag to check if controller and estimator have been turned on
        
        # LQR Controller
        self.u = 0 # A; imput current
        self.controller = LQR(self.A, self.B, self.i)
        
        # LQE Estimator
        self.estimator = LQE(self.A, self.B, self.C, self.G, self.x, self.dt, self.std_noise)
    
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
     
    '''Updates the state of the system'''
    def update(self, t):        
        '''Check if Cubli has jumped up:
        if not, begin jump sequence
            - Continue with feedforward until body angle is approximately 0, then turn on estimator and controller'''
        if self.jumped == False:
            ''''Update Cubli's input using feedforward'''
            self.u = 5
    
            '''Perform jump-up'''
            self.jump()
            
            '''Update the true state and estimated state
                - estimated state is updated here so estimation begins at approximately the right location
                (after jump-up sequence)'''
            self.x = self.intg.step(t, self.x, self.u)
            self.x_e = self.x
            
            self.p += 1
                    
        elif (abs(self.x[0]) > 0.03 and self.control_on == False):
            ''''Update Cubli's input using feedforward'''
            self.u = -1.75
            
            '''Update the true state and estimated state
                - estimated state is updated here so estimation begins at approximately the right location
                (after jump-up sequence)'''
            self.x = self.intg.step(t, self.x, self.u)
            self.x_e = self.x
            
            self.p += 1
            
        else:
            '''Check if the hardware can sample'''
            if (self.num_inc == 0):
                if (self.control_on == False):
                    self.x_e = self.x
                    self.control_on = True
                else:
                    '''Estimate the state'''
                    self.estimate(t)
                
                '''Update Cubli's input using LQR controller
                    - controller state input should be estimated state (x_e)'''
                self.u = self.controller.update(self.x_e)
                self.num_inc += 1
            else:
                self.num_inc += 1
            if (self.num_inc == self.num_inc_max):
                self.num_inc = 0

            '''Update the true state'''
            self.x = self.intg.step(t, self.x, self.u)
            
                         
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
            self.w = np.random.normal(0, self.std_noise, size = (3, 1))
            self.v = np.random.uniform(0, self.std_noise)
            
        '''Check for maximum angular velocity of wheel'''
        if float(self.x[2]) > self.omega_w_max:
            self.x[2] = self.omega_w_max
        elif float(self.x[2]) < -self.omega_w_max:
            self.x[2] = -self.omega_w_max
                
    '''Estimates the state based on current time and input
        - uses output (y) for estimation'''
    def estimate(self, t):
        y = self.C @ self.x + self.v
        self.x_e = self.estimator.update(t, self.u, y)
        
    '''Shoots the cubli up by using conservation of angular momentum'''
    def jump(self):
        '''Check if wheel is spinning fast enough
            - Set body angle slighty off ground (to bipass surface check)
            - convert angular momentum of wheel into angular momentum of sytem body (including wheel)
            - To emulate a hard stop, set angular velocity of wheel to 0'''
        if abs(float(self.x[2])) >= self.omega_w_jump:
            self.x[0] = (-np.pi/4) + 0.00001
            self.x[1] = self.I_w * abs(float(self.x[2])) / (self.I_b + self.I_w + self.m_w*self.l**2)
            self.x[2] = 0
            self.jumped = True
                
        

        
        


        
            

            
        
    