from cubli import Cubli
from controller import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.close("all") 

t_sim = 4 # s
t = 0 # s
show_plot = True
u = 0
#dt_controller = dt * 2 # s
t_delay = 0 # s

'''Create new Cubli object and create controller object for it'''
c = Cubli(-np.pi / 4, 0, 0, True)
dt = c.dt

'''Establish list of state values, times, and inputs'''
theta_b_history = [] # degrees; list of pendulum body angles
theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
theta_b_dot_history = [] # RPM; list of pendulum body angular velocities
theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM 
theta_w_dot_history = [] # RPM; list of wheel angular velocities
theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM

theta_b_e_history = []
theta_b_dot_e_history = []
theta_w_dot_e_history = []

u_history = [] # A; list of inputs
u_history.append(0)
t_history = [] # s
t_history.append(t)

'''Setup to animate cubli in 2D'''
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
plt.axis('square')
plt.axvline(0, linestyle = 'dashed')
ax.set_xlim(-1.125, 1.125)
ax.set_ylim(0, 1.5)
edge_b_1, = ax.plot([0, np.sin(c.x[0] - np.pi / 4)], [0, np.cos(c.x[0] - np.pi / 4)], color = "black")
edge_b_2, = ax.plot([np.sin(c.x[0] - np.pi / 4), np.sqrt(2) * np.sin(c.x[0])], [np.cos(c.x[0] - np.pi / 4), np.sqrt(2) * np.cos(c.x[0])], color = "black")
edge_b_3, = ax.plot([-np.cos(c.x[0] + (3 / 4) * np.pi), np.sqrt(2) * np.sin(c.x[0])], [np.sin(c.x[0] + (3 / 4) * np.pi), np.sqrt(2) * np.cos(c.x[0])], color = "black")
edge_b_4, = ax.plot([0, -np.cos(c.x[0] + (3 / 4) * np.pi)], [0, np.sin(c.x[0] + (3 / 4) * np.pi)], color = "black")

'''Update the controller and the Cubli
    - Stores list of states and inputs
    - Animates the 2D simulation'''    
def update(cubli): 
    u = cubli.u
    cubli.update(t)

    
    ''''Append lists for plotting'''
    u_history.append(u)
    theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
    theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM
    theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM
    theta_b_e_history.append(float(c.x_e[0]) * 180 / np.pi)
    theta_b_dot_e_history.append(float(c.x_e[1]) * 9.549297)
    theta_w_dot_e_history.append(float(c.x_e[2]) * 9.549297)
    t_history.append(t)
    
    '''Animate Cubli in 2D'''
    edge_b_1.set_xdata([0, np.sin(c.x[0] - np.pi / 4)])
    edge_b_1.set_ydata([0, np.cos(c.x[0] - np.pi / 4)])
    edge_b_2.set_xdata([np.sin(c.x[0] - np.pi / 4), np.sqrt(2) * np.sin(c.x[0])])
    edge_b_2.set_ydata([np.cos(c.x[0] - np.pi / 4), np.sqrt(2) * np.cos(c.x[0])])
    edge_b_3.set_xdata([-np.cos(c.x[0] + (3 / 4) * np.pi), np.sqrt(2) * np.sin(c.x[0])])
    edge_b_3.set_ydata([np.sin(c.x[0] + (3 / 4) * np.pi), np.sqrt(2) * np.cos(c.x[0])])
    edge_b_4.set_xdata([0, -np.cos(c.x[0] + (3 / 4) * np.pi)])
    edge_b_4.set_ydata([0, np.sin(c.x[0] + (3 / 4) * np.pi)])
    fig.canvas.draw()
    fig.canvas.flush_events()

def plot():
    plt.figure()
    plt.title('Pendulum Body Angle')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [degrees]')
    plt.plot(t_history, theta_b_linear_history, label = 'Linear')
    plt.plot(t_history, theta_b_nonlinear_history, label = 'Non-linear')
    plt.plot(t_history[:-1], theta_b_e_history, label = 'Estimation')
    plt.legend()
    plt.show()

    plt.figure()
    plt.title('Pendulum Body Angular Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [RPM]')
    plt.plot(t_history, theta_b_dot_linear_history, label = 'Linear')
    plt.plot(t_history, theta_b_dot_nonlinear_history, label = 'Non-linear')
    plt.plot(t_history[:-1], theta_b_dot_e_history, label = 'Estimation')
    plt.legend()
    plt.show()

    plt.figure()
    plt.title('Wheel Angular Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [RPM]')
    plt.plot(t_history, theta_w_dot_linear_history, label = 'Linear')
    plt.plot(t_history, theta_w_dot_nonlinear_history, label = 'Non-linear')
    plt.plot(t_history[:-1], theta_w_dot_e_history, label = 'Estimation')
    plt.legend()
    plt.show()

    plt.figure()
    plt.title('Input Current')
    plt.xlabel('Time [s]')
    plt.ylabel('Current [A]')
    plt.plot(t_history[1:], u_linear_history[1:], label = 'Linear')
    plt.plot(t_history[1:], u_nonlinear_history[1:], label = 'Non-linear')
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(t_history[:-1], theta_b_e_history, label = 'Estimation')
    plt.plot(t_history[:-1], theta_b_dot_e_history, label = 'Estimation')
    plt.show()
    
'''Simulation
    - Run once with linear dynamics
    - Run once with nonlinear dynamics'''
while t < t_sim:
    update(c)
    t += dt
'''Store linear lists then restablish list of state values, times, and inputs'''
theta_b_linear_history = theta_b_history
theta_b_dot_linear_history = theta_b_dot_history
theta_w_dot_linear_history = theta_w_dot_history
u_linear_history = u_history

t = 0
c = Cubli(-np.pi / 4, 0, 0, False)
dt = c.dt
theta_b_history = [] # degrees; list of pendulum body angles
theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
theta_b_dot_history = [] # RPM; list of pendulum body angular velocities
theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM 
theta_w_dot_history = [] # RPM; list of wheel angular velocities
theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM

theta_b_e_history = []
theta_b_dot_e_history = []
theta_w_dot_e_history = []

u_history = [] # A; list of inputs
u_history.append(0)
t_history = [] # s
t_history.append(t)

while t < t_sim:
    update(c)
    t += dt
theta_b_nonlinear_history = theta_b_history
theta_b_dot_nonlinear_history = theta_b_dot_history
theta_w_dot_nonlinear_history = theta_w_dot_history
u_nonlinear_history = u_history

if (show_plot == True):
    plot()



    
    