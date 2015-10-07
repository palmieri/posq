
from __future__ import division

import numpy as np


def integrate(xstart, xend, direction, deltaT, base, initT, vmax, nS=0):
    """ POSQ Integration procedure to general full trajectory """
    assert xstart.shape == xend.shape, 'Expect similar vector sizes'
    assert xstart.size == xend.size == 3, 'Expect 1D array (x, y, theta)'

    vel = np.zeros(shape=(1, 2))
    sl, sr = 0, 0
    old_sl, old_sr = 0, 0
    xvec = np.zeros(shape=(1, 3))  # pose vectors for trajectory
    speedvec = np.zeros(shape=(1, 2))  # velocities during trajectory
    encoders = [0, 0]
    t = initT  # initialize global timer
    ti = 0  # initialize local timer
    eot = 0  # initialize end-of-trajectory flag
    xnow = [xstart[0], xstart[1], xstart[2]]
    old_beta = 0

    while not eot:
        # Calculate distances for both wheels
        dSl = sl - old_sl
        dSr = sr - old_sr
        dSm = (dSl + dSr) / 2
        dSd = (dSr - dSl) / base

        # Integrate robot position
        xnow[0] = xnow[0] + dSm * np.cos(xnow[2] + dSd / 2.0)
        xnow[1] = xnow[1] + dSm * np.sin(xnow[2] + dSd / 2.0)
        xnow[2] = normangle(xnow[2] + dSd, -np.pi)

        # implementation of the controller
        vl, vr, eot, vm, vd, old_beta = step(ti, xnow, xend, direction,
                                             old_beta, vmax, base)
        vel = np.row_stack((vel, [vm, vd]))
        speeds = np.array([vl, vr])
        speedvec = np.row_stack((speedvec, speeds))
        xvec = np.row_stack((xvec, xnow))

        # Increase timers
        ti = ti + deltaT
        t = t + deltaT

        # Increase accumulated encoder values
        # simulated encoders of robot
        delta_dist1 = speeds[0] * deltaT
        delta_dist2 = speeds[1] * deltaT
        encoders[0] += delta_dist1
        encoders[1] += delta_dist2

        # Keep track of previous wheel positions
        old_sl = sl
        old_sr = sr

        # noise on the encoders
        sl = encoders[0] + nS * np.random.uniform(0, 1)
        sr = encoders[1] + nS * np.random.uniform(0, 1)

    inct = t  # at the end of the trajectory the time elapsed is added

    print(xvec)

    return xvec, speedvec, vel, inct


def step(t, xnow, xend, direction, old_beta, vmax, base):
    """ POSQ single step """
    k_v = 3.8
    k_rho = 1    # Condition: k_alpha + 5/3*k_beta - 2/pi*k_rho > 0 !
    k_alpha = 6
    k_beta = -1
    rho_end = 0.00510      # [m]

    if t == 0:
        old_beta = 0

    # extract coordinates
    xc, yc, tc = xnow[0], xnow[1], xnow[2]
    xe, ye, te = xend[0], xend[1], xend[2]

    # rho
    dx = xe - xc
    dy = ye - yc
    rho = np.sqrt(dx**2 + dy**2)
    f_rho = rho
    if f_rho > (vmax / k_rho):
        f_rho = vmax / k_rho

    # alpha
    alpha = normangle(np.arctan2(dy, dx) - tc, -np.pi)

    # direction (forward or backward)
    if direction == 1:
        if alpha > np.pi / 2:
            f_rho = -f_rho                   # backwards
            alpha = alpha - np.pi
        elif alpha <= -np.pi / 2:
            f_rho = -f_rho                   # backwards
            alpha = alpha + np.pi
    elif direction == -1:                  # arrive backwards
        f_rho = -f_rho
        alpha = alpha + np.pi
        if alpha > np.pi:
            alpha = alpha - 2 * np.pi

    # phi, beta
    phi = te - tc
    phi = normangle(phi, -np.pi)
    beta = normangle(phi - alpha, -np.pi)
    if abs(old_beta - beta) > np.pi:           # avoid instability
        beta = old_beta
    old_beta = beta

    vm = k_rho * np.tanh(f_rho * k_v)
    vd = (k_alpha * alpha + k_beta * beta)
    eot = (rho < rho_end)

    # Convert speed to wheel speeds
    vl = vm - vd * base / 2
    if abs(vl) > vmax:
        vl = vmax * np.sign(vl)

    vr = vm + vd * base / 2
    if abs(vr) > vmax:
        vr = vmax * np.sign(vr)

    return vl, vr, eot, vm, vd, old_beta


def normangle(theta, start=0):
    """ Normalize an angle to be in the range :math:`[0, 2\pi]`

    Parameters
    -----------
    theta : float
        input angle to normalize

    start: float
        input start angle (optional, default: 0.0)

    Returns
    --------
    res : float
        normalized angle or :math:`\infty`

    """
    if theta < np.inf:
        while theta >= start + 2 * np.pi:
            theta -= 2 * np.pi
        while theta < start:
            theta += 2 * np.pi
        return theta
    else:
        return np.inf
