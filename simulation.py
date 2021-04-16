from cubli import Cubli
from controller import *
import numpy as np
import matplotlib.pyplot as plt

plt.close("all") 

'''TODO: Restyle and fix simulation code'''

dt = 0.001
t_sim = 20
t = 0

c = Cubli(-0.001, 0, 0)
controller = LQR(c.A, c.B, c.I_stall)

theta_b_history = [] # degrees; list of pendulum body angles
theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
theta_b_dot_history = [] # RPM; list of pendulum body angular velocities
theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM 
theta_w_dot_history = [] # RPM; list of wheel angular velocities
theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM
t_history = [] # s
t_history.append(t)

while t < t_sim:
    u = controller.update(c.x)
    c.update(u, True)
    theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
    theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM
    theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM
    t += dt # increment timestep
    t_history.append(t)

'''Plots'''
plt.figure()
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
plt.plot(t_history, theta_w_dot_history)
plt.show()

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

