from cubli import Cubli
from controller import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.close("all") 

dt = 0.01
t_sim = 10
t = 0
show_plot = True

'''Create new Cubli object and create controller object for it'''
c = Cubli(-np.pi / 16 + 0.001, 0, 0, False)
controller = LQR(c.A, c.B, c.I)

'''Establish list of state values, times, and inputs'''
theta_b_history = [] # degrees; list of pendulum body angles
theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
theta_b_dot_history = [] # RPM; list of pendulum body angular velocities
theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM 
theta_w_dot_history = [] # RPM; list of wheel angular velocities
theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM
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
def update():
    u = controller.update(c.x)
    # Delay
    c.update(t, u)
    u_history.append(u)
    theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
    theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM
    theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM
    t_history.append(t)
    
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
    fig = plt.figure()
    plt.title('Pendulum Body Angle')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [degrees]')
    plt.plot(t_history, theta_b_history)
    plt.show()

    plt.figure()
    plt.title('Pendulum Body Angular Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [RPM]')
    plt.plot(t_history, theta_b_dot_history)
    plt.show()

    plt.figure()
    plt.title('Wheel Angular Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [RPM]')
    plt.plot(t_history, theta_w_dot_history, label = '$\omega_w$')
    plt.legend()
    plt.show()

    plt.figure()
    plt.title('Input Current')
    plt.xlabel('Time [s]')
    plt.ylabel('Current [A]')
    plt.plot(t_history[1:], u_history[1:], label = 'u')
    plt.show()
    
'''Simulation'''  
while t < t_sim:
    update()
    t += dt

plt.close("all")
if (show_plot == True):
    plot()

 
'''
fig, (ax1, ax2, ax3) = plt.subplots(3)
ax1.set_title('Pendulum Body Angle')
ax1.set(xlabel = 'Time [s]', ylabel = r'$\theta$ [degrees]')
ax1.plot(t_history, theta_b_history)

ax2.set_title('Pendulum Body Angular Velocity')
ax2.set(xlabel = 'Time [s]', ylabel = '$\omega$ [RPM]')
ax2.plot(t_history, theta_b_dot_history)

ax3.set_title('Wheel Angular Velocity')
ax3.set(xlabel = 'Time [s]', ylabel = '$\omega$ [RPM]')
ax3.plot(t_history, theta_w_dot_history)

plt.tight_layout()
'''

    
    