from cubli import Cubli
import numpy as np
import matplotlib.pyplot as plt

plt.close("all") 

'''TODO: Restyle and fix simulation code'''

dt = 0.001
t_sim = 20
t = 0

c = Cubli(-np.pi / 4, 0, 0)
theta_b_history = [] # degrees; list of pendulum body angles
theta_b_dot_history = [] # RPM; list of pendulum body angular velocities 
theta_w_dot_history = [] # RPM; list of wheel angular velocities

t_history = [] # s

while t < t_sim:
    theta_b_history.append(float(c.x[0]) * 180 / np.pi) # convert radians to degrees
    theta_b_dot_history.append(float(c.x[1]) * 9.549297) # convert radians per second to RPM
    theta_w_dot_history.append(float(c.x[2]) * 9.549297) # convert radians per second to RPM
    c.update(False)
    t_history.append(t)
    t += dt # increment timestep

plt.figure()
plt.title('Pendulum Body Angle')
plt.xlabel('Time [s]')
plt.xlim(0, 1)
plt.ylabel('Angle [degrees]')
plt.plot(t_history, theta_b_history)
plt.show()

plt.figure()
plt.title('Pendulum Body Angular Velocity')
plt.xlabel('Time [s]')
plt.xlim(0, 1)
plt.ylabel('Angular Velocity [RPM]')
plt.plot(t_history, theta_b_dot_history)
plt.show()

plt.figure()
plt.title('Wheel Angular Velocity')
plt.xlabel('Time [s]')
plt.xlim(0, 1)
plt.ylabel('Angular Velocity [RPM]')
plt.plot(t_history, theta_w_dot_history)
plt.show()