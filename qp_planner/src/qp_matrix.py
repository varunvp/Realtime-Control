#!/usr/bin/python
import quadprog, math
from numpy import array
import numpy as np
from scipy.linalg import block_diag
from math import sin, cos, radians, degrees

def quadprog_solve_qp(H, h, A=None, b=None, C=None, d=None):
    qp_H = .5 * (H + H.T)  # make sure H is symmetric
    qp_h = -h
    if C is not None:
        qp_C = -np.vstack([C, A]).T
        qp_d = -np.hstack([d, b])
        meq = C.shape[0]
    else:  # no equality constraint
        qp_C = -A.T
        qp_d = -b
        meq = 0

    # print qp_H
    # print qp_h
    # print qp_C
    # print qp_d
    return quadprog.solve_qp(qp_H, qp_h, qp_C, qp_d, meq)[0]


def qp_q_dot_des(q_act, q_des, q_origin, q_limit, q_kp, q_kb):
    # cost function matrix is given here   e.g. u^T H u
    H = array([[1000., 0.], [0., 1.]])
    h = array([0., 0.])  # cost function vector    e.g. h^T u

    # stability constraints
    kp = q_kp
    Va = q_act - q_des
    Vb = -kp * Va * Va

    # # safety constraints
    limit = q_limit  # in kms for position and radians for angles - very high
    q_rel = q_act - q_origin
    Ba = - 2. * q_rel  # derivative of angle_limit - x^2
    Bb = -q_kb * (limit * limit - q_rel * q_rel)  # - (angle_limit^2 - x^2)

    # inequality constraints are given here Au <= b
    A = array([[-1, Va], [0, -Ba]])
    b = array([Vb, -Bb])
    # A = array([[-1, Va]])
    # b = array([Vb])

    u_in = quadprog_solve_qp(H, h, A, b)

    return array([u_in[1]])


def qp_q_dot_des_array(q_act, q_des, q_origin, q_limit, q_kp, q_kb):
    n = len(q_act)
    H = array([[1000., 0.], [0., 1.]])
    h = array([0., 0.])  # cost function vector    e.g. h^T u

    big_H = np.kron(np.eye(n), H)
    big_h = np.zeros((2.0 * n), np.float)

    big_A = np.empty((2, 2), np.float)
    big_b = np.empty((2, 1), np.float)

    for i in range(0, n):
   #  		get_Va_Vb(q_act[i], q_des[i], nsteps, time_horizon)


   #  		get_Ba_Bb(q_act[i], q_limit[i], nsteps, time_horizon)

			# get_terminal_constraints(q_act[i],q_des[i],nsteps, time_horizon)
	    kp = q_kp[i]
	    Va = q_act[i] - q_des[i]
	    Vb = -kp * Va * Va

	    limit = q_limit[i]
	    q_rel = q_act[i] - q_origin[i]
	    Ba = -2 * q_rel
	    Bb = -q_kb[0] * (limit * limit - q_rel * q_rel)

	    A = array([[-1, Va], [0, -Ba]])
	    b = array([Vb, -Bb])

	    if (i == 0):
	        big_A = A
	        big_b = b

	    elif(i != 0):
	        big_A = block_diag(big_A, A)
	        big_b = np.append(big_b, b)

	    # print(big_H)
	    # print(big_h)
	    # print(big_A)
	    # print(big_b)
    u_in_temp = quadprog_solve_qp(big_H, big_h, big_A, big_b)

    u_in = []

    for i in range(1, 2 * n, 2):
        u_in = np.append(u_in, u_in_temp[i])

    return u_in
    # print(u_in)

def qp_non_holonomic_solver(act, des, obs, kp, kb):
    # H = array([[1000., 0.], [0., 1.]])
    # h = array([0., 0.])

    # stability constraints
    # Va = ((act[0] - des[0]) * cos(radians(act[2])) + (act[1] - des[1]) * sin(radians(act[2])))
    # Vb = - kp / 2 * (((act[0] - des[0]) ** 2 + (act[1] - des[1]) ** 2))

    # safety constraints
    # Ba = ((act[0] - obs[0]) * cos(radians(act[2])) + (act[1] - obs[1]) * sin(radians(act[2])))
    # Bb = - kb / 2 * (((act[0] - obs[0]) ** 2 + (act[1] - obs[1]) ** 2) - obs[2] ** 2)
    
    # inequality constraints are given here Au <= b
    # A = np.array([[-1, Va], [0, -Ba]])
    # b = np.array([Vb, -Bb])

    H = array([[1000., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    h = array([0., 0., 0.])  # cost function vector    e.g. h^T u

    Va = [0., 0.]
    Vb = [0., 0.]
    Ba = [0., 0.]
    Bb = [0., 0.]

    # stability constraints
    # Va[0] = - 1 * (des[0] - act[0])
    # Va[1] = - sin(radians(des[2] - act[2])) / kp

    # # safety constraints
    # Vb[0] = - 0.5 * ((des[0] - act[0]) ** 2 + (des[1] - act[1]) ** 2)
    # Vb[1] = - ((1 - cos(radians(des[2] - act[2]))) / kp)

    # # inequality constraints are given here Au <= b
    # Ba[0] = ((act[0] - obs[0]) * cos(radians(des[2] - act[2])) + (act[1] - obs[1]) * sin(radians(des[2] - act[2])))
    # Bb[0] = - kb / 2 * (((act[0] - obs[0]) ** 2 + (act[1] - obs[1]) ** 2) - obs[2] ** 2)
    # Ba[1] = 0
    # Bb[1] = 0

    # A = np.array([[-1, Va[0], Va[1]], [0, -Ba[0], -Ba[1]]])
    # b = np.array([(Vb[0] + Vb[1]) * 0.01, (-Bb[0] + (-Bb[1]))])

    # stability constraints
    Va[0] = - 2 * ((des[0] - act[0]) * cos(radians(act[2])) + (des[1] - act[1]) * sin(radians(act[2])))
    Va[1] = - sin(radians(des[2] - act[2]))#- 2 * (des[2] - act[2])

    # safety constraints
    Vb[0] = - kp * ((des[0] - act[0]) ** 2 + (des[1] - act[1]) ** 2)
    Vb[1] = - kp * (1- cos(radians(des[2] - act[2]))) #(des[2] - act[2]) ** 2

    # inequality constraints are given here Au <= b
    Ba[0] = 0 * ((act[0] - obs[0]) * cos(radians(des[2] - act[2])) + (act[1] - obs[1]) * sin(radians(des[2] - act[2])))
    Bb[0] = - kb / 2 * 0 * (((act[0] - obs[0]) ** 2 + (act[1] - obs[1]) ** 2) - obs[2] ** 2)
    Ba[1] = 0
    Bb[1] = 0

    A = np.array([[-1, Va[0], 0], [-1, 0, Va[1]], [0, -Ba[0], -Ba[1]]])
    b = np.array([Vb[0], Vb[1], - Bb[0] + (-Bb[1])])


    u_in = quadprog_solve_qp(H, h, A, b)

    print(u_in[0])
    if __name__ == "__main__":
        print A
        print b
        print u_in

    return u_in[1], u_in[2]

    
if __name__ == "__main__":
    # actual = [0., 0., 0.]
    # desired = [3., 5., 7.]
    # origin = [0., 0., 0.]
    # limit = [5., 5., 5.]
    # kp = [1., 1., 1.]
    # kb = [10., 10., 10.]

    # qp_q_dot_des_array(actual, desired, origin, limit, kp, kb)
    
    act = [5.268, -3.84, -42]
    des = [1.487, 0, 0.]
    obs = [0, 0, 0.]
    kp = 1
    kb = 10

    qp_non_holonomic_solver(act, des, obs, kp, kb)