
import numpy as np
import posq

source = np.array([0, 0, 0])
target = np.array([1, 1, -np.pi/6])
direction = 1
resolution = 0.1
base = 0.4
init_t = 0.1
max_speed = 1.0


traj, speedvec, vel, inct = posq.integrate(source, target, direction,
                                           resolution, base,
                                           init_t, max_speed, nS=0)


import matplotlib.pyplot as plt
plt.style.use('fivethirtyeight')

plt.figure(figsize=(10, 10))
plt.plot(traj[0, 0], traj[0, 0], ls='', marker='8', color='b',
         markersize=15, label='start')
plt.plot(traj[-1, 0], traj[-1, 0], ls='', marker='8', color='g',
         markersize=15, label='goal')

ax = plt.axes()
for wp in traj:
    vx, vy = np.cos(wp[2]), np.sin(wp[2])
    ax.arrow(wp[0], wp[1], 0.1*vx, 0.1*vy, head_width=0.02,
             head_length=0.02, lw=0.4, fc='brown', ec='brown')

plt.xlim((-0.2, 1.2))
plt.ylim((-0.2, 1.2))
# plt.savefig('posq_demo.pdf')
plt.show()
