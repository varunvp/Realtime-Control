#!/usr/bin/python
import quadprog
import numpy
from numpy import array
import numpy as np
from scipy.linalg import block_diag
import qp_matrix

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
	big_H = np.eye(2*nsteps)			#G
	big_h = np.zeros(2*nsteps)		
	
	big_Aeq, big_beq = MPC_dynamic_constraints(actual, desired,nsteps,interval)
	dyn_A, dyn_b = MPC_terminal_constraints(nsteps, actual, desired, big_Aeq,big_beq)
	
	big_A = np.row_stack((big_Aeq, dyn_A))
	big_b = np.concatenate((big_beq, dyn_b))

	big_Ba, big_Bb=MPC_boundary_constraints(nsteps, limit, desired, origin)
	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba, big_Bb, C=big_A, d=big_b)
	# print(u_in[nsteps])
	return u_in[nsteps]

def MPC_dynamic_constraints(actual=0., desired=0.,nsteps=10.,interval=0.1):
	global big_A, big_b

	big_A = np.column_stack((np.eye(nsteps , dtype=float) + np.eye(nsteps, nsteps , 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	#C
	big_b = np.zeros(nsteps, dtype=float)

	return big_A,big_b


def MPC_terminal_constraints(nsteps, actual, desired, A_eq, b_eq):
	global big_A, big_b

	dyn_A = np.zeros((1,2*nsteps), dtype=float)
	dyn_A[0][0] = 1
	dyn_b = np.array([actual-desired])
	
	# big_A = block_diag(big_A, lhs_matrix)

	return dyn_A, dyn_b
	

def MPC_boundary_constraints(nsteps, limit, desired, origin):
	pos_constraint = np.row_stack( (np.eye(nsteps), np.eye(nsteps) * -1) )
	vel_constraint = np.zeros((2 * nsteps, nsteps))
	Ba = np.column_stack((pos_constraint, vel_constraint))
	# Bb = np.concatenate((np.ones(nsteps) * (limit - actual), np.zeros(nsteps)))
	Bb = np.concatenate((np.ones(nsteps) * (limit + origin - desired), np.ones(nsteps) * (limit - origin + desired)))
	# print(Ba)
	# print(Bb)
	return Ba, Bb


if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	MPC_solver(0, 0, 100, 0)