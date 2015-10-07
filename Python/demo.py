
import numpy as np
import posq

source = np.array([0, 0, 0])
target = np.array([-1, 0, np.pi/2])
direction = 1
resolution = 0.1
base = 0.4
init_t = 0.1
max_speed = 1.0


traj, speedvec, vel, inct = posq.integrate(source, target, direction,
                                           resolution, base,
                                           init_t, max_speed, nS=0)


import matplotlib.pyplot as plt

plt.figure(figsize=(10, 10))
plt.plot(traj[:, 0], traj[:, 0])
plt.show()
