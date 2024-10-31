from casadi import *
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import sys
from mpl_toolkits.mplot3d import Axes3D
from time import time
from decimal import Decimal


def dlodynamics_3D(STATE0, STATE1, Length, PHYSICAL, para0):
    global Rf, Rt, Re, Dg
    global n, N, s0, s1, ds, lx, ly, lz, state0, state1, L
    L = Length

    # Flexural coefficient Default = 1
    Rf = PHYSICAL[0]
    # Torsional coefficient Default = 1
    Rt = PHYSICAL[1]
    # Extension coefficient Default = 0
    Re = PHYSICAL[2]
    # Gravial coefficient Default = 0
    Dg = PHYSICAL[3]

    N = 19
    s0 = 0
    s1 = L
    ds = (s1 - s0) / float(N)
    kmax = 2
    n = 2 * kmax + 2

    state0 = STATE0
    state1 = STATE1

    lx = state1[0] - state0[0]
    ly = state1[1] - state0[1]
    lz = state1[2] - state0[2]
    ax = state1[3] - state0[3]
    ay = state1[4] - state0[4]
    az = state1[5] - state0[5]

    opti = casadi.Opti()
    x = opti.variable(4 * n)
    opti.set_initial(x, para0)

    opti.minimize(costfun(x))
    opti.subject_to(constraints(x)[0])
    opti.subject_to(constraints(x)[1])
    opti.subject_to(constraints(x)[2])

    opti.subject_to(constraints(x)[3])
    opti.subject_to(constraints(x)[4])
    opti.subject_to(constraints(x)[5])

    opti.subject_to(constraints(x)[6])
    opti.subject_to(constraints(x)[7])
    opti.subject_to(constraints(x)[8])

    opti.solver('ipopt')
    sol = opti.solve()
    para1 = sol.value(x)

    [p_dat, PHI_dat, T_dat] = plotDLO(para1)
    return p_dat, PHI_dat, T_dat, para1


def costfun(param):
    global n, N, s0, s1, ds, Rf, Rt, Re, Dg, state0
    Uflex = 0.0
    Utor = 0.0
    Uext = 0.0
    Ugrav = 0.0

    a_phi = param[0:n]
    a_theta = param[n:2 * n]
    a_psi = param[2 * n:3 * n]
    a_epsilon = param[3 * n:4 * n]

    for i in range(N + 1):
        # Compute the flexural energy
        s = ds * i
        theta = f_evaluate(s, a_theta, state0[4])
        dtheta = f_derivate(s, a_theta)
        dphi = f_derivate(s, a_phi)
        kappa_2 = dtheta ** 2 + (dphi ** 2) * (ca.sin(theta) ** 2)
        Uflex = Uflex + Rf * kappa_2 * ds

        # Compute torsional energy
        dpsi = f_derivate(s, a_psi)
        omega_2 = (dphi * ca.cos(theta) + dpsi) ** 2
        Utor = Utor + Rt * omega_2 * ds

        # Compute the extensional energy
        if Re != 0.0:
            epsilon = f_evaluate(s, a_epsilon, 0.0)
            Uext = Uext + Re * (epsilon ** 2) * ds

        # Compute gravitational energy:
        # 1) Firstly, compute the current position along x
        # 2) Secondly, compute the energy
        if Dg != 0.0:
            x = state0[0]
            epsilon_l = 0.0
            for j in range(int(np.round(float(Decimal(str(s)) / Decimal(str(ds))))) + 1):
                sl = ds * j
                if Re != 0.0:
                    epsilon_l = f_evaluate(sl, a_epsilon, 0.0)

                theta_l = f_evaluate(sl, a_theta, state0[4])
                phi_l = f_evaluate(sl, a_phi, state0[3])
                x = x + (1 + epsilon_l) * ca.sin(theta_l) * ca.cos(phi_l) * ds

            Ugrav = Ugrav + Dg * x * ds
    C = 0.5 * Uflex + 0.5 * Utor + 0.5 * Uext + Ugrav
    return C


def constraints(param):
    global n, s0, s1, ds, lx, ly, lz, state0, state1, Re
    a_phi = param[0:n]
    a_theta = param[n:2 * n]
    a_psi = param[2 * n:3 * n]
    a_epsilon = param[3 * n:4 * n]

    # Constraints at s = s0
    # Orientations
    con0 = (state0[3] - f_evaluate(s0, a_phi, state0[3]) == 0)
    con1 = (state0[4] - f_evaluate(s0, a_theta, state0[4]) == 0)
    con2 = (state0[5] - f_evaluate(s0, a_psi, state0[5]) == 0)

    # Constraints at s = s1
    # Orientations
    con3 = (state1[3] - f_evaluate(s1, a_phi, state0[3]) == 0)
    con4 = (state1[4] - f_evaluate(s1, a_theta, state0[4]) == 0)
    con5 = (state1[5] - f_evaluate(s1, a_psi, state0[5]) == 0)

    # Positions
    p = CalcPosition(s1, param)
    con6 = (lx - p[0, 0] + state0[0] == 0)
    con7 = (ly - p[1, 0] + state0[1] == 0)
    con8 = (lz - p[2, 0] + state0[2] == 0)

    # if Re != 0.0:
    #     con9 = (f_evaluate(s0, a_epsilon, 0.0) == 0)
    #     con10 = (f_evaluate(s1, a_epsilon, 0.0) == 0)
    # else:
    #     con9 = (0 == 0)
    #     con10 = (0 == 0)

    return con0, con1, con2, con3, con4, con5, con6, con7, con8


def plotDLO(param):
    global s0, ds, s1, L
    P_dat = None
    PHI_dat = None
    T_dat = None

    for i in range(N + 1):
        s = ds * i
        P = CalcPosition(s, param)
        PHI = CalcOrientation(s, param)
        T = RotAxeAngle('x', PHI[0]).dot(RotAxeAngle('y', PHI[1])).dot(RotAxeAngle('z', PHI[2]))
        T[0:3, 3] = P.reshape(1, 3)

        if P_dat is None:
            P_dat = P
        else:
            P_dat = np.hstack((P_dat, P))

        if PHI_dat is None:
            PHI_dat = PHI
        else:
            PHI_dat = np.hstack((PHI_dat, PHI))

        if T_dat is None:
            T_dat = T
        else:
            T_dat = np.vstack((T_dat, T))

    P_dat = np.transpose(P_dat)
    PHI_dat = np.transpose(PHI_dat)
    return np.array(P_dat), np.array(PHI_dat), np.array(T_dat)


def f_evaluate(s, a, f0):
    global L, n
    e = np.arange(n, dtype=np.float)
    e[0] = 1
    e[1] = s
    for i in range(2, n):
        if i % 2 == 0.0:
            k = (i - 0) / 2.0
            e[i] = ca.sin(2 * ca.pi * k * s / L)
        if i % 2 == 1.0:
            k = (i - 1) / 2.0
            e[i] = ca.cos(2 * ca.pi * k * s / L)

    f = f0
    for i in range(n):
        f = f + a[i] * e[i]

    return f


def f_derivate(s, a):
    global L, n
    e = np.arange(n, dtype=np.float)
    e[0] = 0
    e[1] = 1
    for i in range(2, n):
        if i % 2 == 0.0:
            k = (i - 0) / 2.0
            e[i] = (2 * ca.pi * k / L) * ca.cos(2 * ca.pi * k * s / L)

        if i % 2 == 1.0:
            k = (i - 1) / 2.0
            e[i] = -(2 * ca.pi * k / L) * ca.sin(2 * ca.pi * k * s / L)

    df = 0
    for i in range(n):
        df = df + a[i] * e[i]
    return df


def CalcPosition(s, param):
    global n, s0, ds, state0, Re
    a_phi = param[0:n]
    a_theta = param[n:2 * n]
    a_epsilon = param[3 * n:4 * n]
    p = np.array([[state0[0]],
                  [state0[1]],
                  [state0[2]]])
    epsilon_l = 0.0

    for i in range(int(np.round(float(Decimal(str(s)) / Decimal(str(ds))))) + 1):
        sc = ds * i
        if Re != 0.0:
            epsilon_l = f_evaluate(sc, a_epsilon, 0.0)

        theta_l = f_evaluate(sc, a_theta, state0[4])
        phi_l = f_evaluate(sc, a_phi, state0[3])

        zeta = np.array([[ca.sin(theta_l) * ca.cos(phi_l)],
                         [ca.sin(theta_l) * ca.sin(phi_l)],
                         [ca.cos(theta_l)]])

        p = p + (1 + epsilon_l) * zeta * ds
    return p


def CalcOrientation(s, param):
    global n, state0
    a_phi = param[0:n]
    a_theta = param[n:2 * n]
    a_psi = param[2 * n:3 * n]
    phi = f_evaluate(s, a_phi, state0[3])
    theta = f_evaluate(s, a_theta, state0[4])
    psi = f_evaluate(s, a_psi, state0[5])

    PHI = np.array([[phi],
                    [theta],
                    [psi]])
    return PHI


def RotAxeAngle(axe, angle):
    if axe == 'x':
        T = np.array([[1, 0, 0, 0],
                      [0, np.cos(angle), -np.sin(angle), 0],
                      [0, np.sin(angle), np.cos(angle), 0],
                      [0, 0, 0, 1]])

    if axe == 'y':
        T = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                      [0, 1, 0, 0],
                      [-np.sin(angle), 0, np.cos(angle), 0],
                      [0, 0, 0, 1]])

    if axe == 'z':
        T = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                      [np.sin(angle), np.cos(angle), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
    return T


if __name__ == '__main__':
    # lx = 0.4
    # ly = 0.4
    # lz = -0.3
    # ax = np.deg2rad(0)
    # ay = np.deg2rad(180)
    # az = np.deg2rad(0)
    # state1 = (state0 + np.array([lx, ly, lz, ax, ay, az]))

    plt.ion()
    plt.show()
    start = time()
    theta = np.linspace(0, 2*np.pi, num=100)
    para0 = np.ones((4 * 6)) * 1.0
    for i in range(np.size(theta)):
        state0 = np.array([-0.1 + cos(theta[i]) * 0.2, -0.1 + sin(theta[i]) * 0.2, 0.0, 0, 0, 0])
        state1 = np.array([0.3, 0.3, -0.3, np.deg2rad(0), np.deg2rad(180), np.deg2rad(0)])
        p_dat, PHI_dat, T_dat, para1 = dlodynamics_3D(state0, state1, 2, np.array([1, 1, 0, 0], dtype=float), para0)
        para0 = para1

        ax = plt.axes(projection='3d')
        ax.scatter(p_dat[:, 0], p_dat[:, 1], p_dat[:, 2], color='red', linewidth=5, marker='.')
        ax.set_xlim3d(-0.8, 1)
        ax.set_ylim3d(-0.8, 1)
        ax.set_zlim3d(-0.8, 1)
        plt.pause(0.001)
