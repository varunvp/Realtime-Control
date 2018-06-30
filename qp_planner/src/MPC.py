#!/usr/bin/python
import quadprog
import numpy
from numpy import array
import numpy as np
from scipy.linalg import block_diag
import qp_matrix
import time

def MPC_solver(actual=0., desired=0., limit=1000, origin=0, nsteps=10.,interval=0.1):
	"""MPC which uses Quadratic Programming solver
	
	Keyword Arguments:
		actual {float} -- The current value (default: {0.})
		desired {float} -- The desired value (default: {0.})
		nsteps {float} -- Number of steps (default: {10.})
		interval {float} -- Time Interval (default: {0.1})
	
	Returns:
		float -- Solution
	"""
	timer = time.time()
	big_I = np.eye(2*nsteps)			#G
	big_0 = np.zeros(2*nsteps)
	
	dyn_A, dyn_b = MPC_dynamic_constraints(actual, desired, nsteps, interval)
	term_A, term_b = MPC_terminal_constraints(nsteps, actual, desired)
	
	#Concatenate dynamic and terminal constraints
	big_A_eq = np.row_stack((dyn_A, term_A))
	big_b_eq = np.concatenate((dyn_b, term_b))

	#Inequality constraints
	big_Ba_ineq, big_Bb_ineq = MPC_boundary_constraints(nsteps, limit, desired, origin)

	# print(np.shape(big_I))
	# print(np.shape(big_0))
	# print(np.shape(big_A_eq))
	# print(np.shape(big_b_eq))

	#Relaxation
	big_H, big_h, big_A_eq, big_b_eq, big_Ba_ineq, big_Bb_ineq = MPC_relaxation(big_I, big_0, big_A_eq, big_b_eq, big_Ba_ineq, big_Bb_ineq, interval, nsteps)

	# print(big_H)
	# print((big_h))
	# print((big_A_eq))
	# print((big_b_eq))
	# print((big_Ba_ineq))
	# print((big_Bb_ineq))
	#Calculate solution
	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba_ineq, big_Bb_ineq, big_A_eq, big_b_eq)
	print(u_in[nsteps])
	print(timer - time.time())
	return u_in[nsteps]

def MPC_dynamic_constraints(actual=0., desired=0.,nsteps=10.,interval=0.1):
	global big_A, big_b

	# [1 -1  0 t 0 0]
	# [0  1 -1 0 t 0] 
	# [0  0  1 0 0 t]
	big_A = np.column_stack((np.eye(nsteps , dtype=float) + np.eye(nsteps, nsteps , 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	#C

	big_b = np.zeros(nsteps, dtype=float)

	return big_A,big_b


def MPC_terminal_constraints(nsteps, actual, desired):
	dyn_A = np.zeros((1,2*nsteps), dtype=float)
	dyn_A[0][0] = 1
	dyn_b = np.array([actual-desired])

	return dyn_A, dyn_b
	

def MPC_boundary_constraints(nsteps, limit, desired, origin):
	pos_constraint = np.row_stack( (np.eye(nsteps), np.eye(nsteps) * -1) )
	vel_constraint = np.zeros((2 * nsteps, nsteps))
	Ba = np.column_stack((pos_constraint, vel_constraint))
	Bb = np.concatenate((np.ones(nsteps) * (limit + origin - desired), np.ones(nsteps) * (limit - origin + desired)))
	# print(Ba)
	# print(Bb)
	return Ba, Bb


def MPC_relaxation(big_I, big_0, big_A_eq, big_b_eq, big_Ba_ineq, big_Bb_ineq, interval, nsteps):
	H = block_diag([1000], big_I)
	h = np.concatenate(([0], big_0))
	big_A_eq = np.column_stack((np.ones(nsteps + 1) * interval, big_A_eq))
	big_Ba_ineq = block_diag([1], big_Ba_ineq)
	big_Bb_ineq = np.concatenate(([1], big_Bb_ineq))
	# big_b_eq = np.concatenate(([0], big_b_eq))
	return H, h, big_A_eq, big_b_eq, big_Ba_ineq, big_Bb_ineq

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	MPC_solver(0, 1, 100, 0)