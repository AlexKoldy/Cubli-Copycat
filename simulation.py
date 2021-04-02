from cubli import Cubli
import numpy as np
import matplotlib.pyplot as plt
dt = 0.01
t_sim = 100
t = 0

c = Cubli(np.pi / 4, 0, 15)
theta_history = []
t_history = []

while t < t_sim:
    c.update()
    theta_history.append(c.x[0])
    t_history.append(t)

plt.figure()
plt.plot(t_history, theta_history)