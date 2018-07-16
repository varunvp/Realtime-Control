#!/usr/bin/python
import quadprog
import numpy
from numpy import array
import numpy as np
from scipy.linalg import block_diag
import qp_matrix
import time

prev_nsteps = 0
prev_interval = 0
big_I = big_0= big_A_eq= dyn_A= dyn_b= term_A= term_b= pos_constraint= vel_constraint = np.empty(2)
big_H= big_h= big_A_eq= big_Ba_ineq = np.empty(2)

def MPC_solver(actual=0., desired=0., limit=1000, origin=0, nsteps=10.,interval=0.1, variables = None, ret_points = False):
	"""MPC which uses Quadratic Programming solver
	
	Keyword Arguments:
		actual {float} -- The current value (default: {0.})
		desired {float} -- The desired value (default: {0.})
		nsteps {float} -- Number of steps (default: {10.})
		interval {float} -- Time Interval (default: {0.1})
		variables {dict} -- Returns cached variables (default: {None})
		ret_points {bool} -- Enable to return points in {variables}, under "points" (default: {False})
	
	Returns:
		float -- Solution
	"""
	while True:
		delta = 0.5
		global prev_nsteps, prev_interval, big_I, big_0, dyn_A, dyn_b, term_A, term_b, pos_constraint, vel_constraint, big_H, big_h, big_A_eq, big_Ba_ineq
		if variables:
			big_A_eq = variables.get("big_A_eq")
			big_Ba_ineq = variables.get("big_Ba_ineq")
			big_H = variables.get("big_H")
			big_h = variables.get("big_h")
			prev_nsteps = variables.get("prev_nsteps")
			prev_interval = variables.get("prev_interval")


		timer = time.time()

		if nsteps != prev_nsteps:
			big_I = np.eye(2*nsteps)			#G
			big_0 = np.zeros(2*nsteps)
		
		#Get dynamic & terminal constraints
		if (nsteps != prev_nsteps or interval != prev_interval):
			# [1 -1  0 t 0 0]
			# [0  1 -1 0 t 0] 
			# [0  0  1 0 0 t]
			dyn_A = np.column_stack((np.eye(nsteps , dtype=float) + np.eye(nsteps, nsteps , 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	#C
			dyn_b = np.zeros(nsteps, dtype=float)
			term_A = np.zeros((1,2*nsteps), dtype=float)

			#Concatenate dynamic and terminal LHS constraint
			big_A_eq = np.row_stack((dyn_A, term_A))


		term_b = np.array([actual-desired])
		
		#Concatenate dynamic and terminal RHS constraint
		big_b_eq = np.concatenate((dyn_b, term_b))

		#Inequality constraints(Boundary constraints)
		if nsteps != prev_nsteps:
			positive_pos_constraint = np.eye(nsteps, nsteps) * -delta + np.eye(nsteps, nsteps, 1)
			negative_pos_constraint = np.eye(nsteps, nsteps) * delta - np.eye(nsteps, nsteps, 1)
			pos_constraint = np.row_stack( (positive_pos_constraint, negative_pos_constraint) )
			vel_constraint = np.zeros((2 * nsteps, nsteps))
			big_Ba_ineq = np.column_stack((pos_constraint, vel_constraint))

		#######################################################################################
		solution = variables.get("solution")				#This is the solution from the previous calculation
		#Check 'solution' for None
		#Chaitanya's code here. Put variables that won't change with nstep's or time interval's change inside the above if, for caching
		####################################################################################### 

		# big_Bb_ineq = np.concatenate((np.ones(nsteps) * (limit + origin - desired), np.ones(nsteps) * (limit - origin + desired)))
		big_Bb_ineq = np.concatenate((np.ones(nsteps) * ((origin + limit - desired) * (1 - delta)), np.ones(nsteps) * (-(origin - limit - desired) *  (1 - delta))))

		#Relaxation
		if (nsteps != prev_nsteps or interval != prev_interval):
			big_H = block_diag([1000], big_I)
			big_h = np.concatenate(([0], big_0))
			big_A_eq = np.column_stack((np.zeros(nsteps + 1), big_A_eq))
			big_A_eq[nsteps-1][0] = -1
			big_A_eq[nsteps][1] = 1
			big_Ba_ineq = np.column_stack((np.transpose(np.zeros(2*nsteps)), big_Ba_ineq))
			big_Ba_ineq[nsteps-1][0] = 1
			big_Ba_ineq[2 * nsteps-1][0] = -1


		u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba_ineq, big_Bb_ineq, big_A_eq, big_b_eq)

		if (nsteps != prev_nsteps or interval != prev_interval):
			variables = {"big_A_eq": big_A_eq, "big_Ba_ineq": big_Ba_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "solution": u_in}

			prev_nsteps = nsteps
			prev_interval = interval

		# print(u_in)
		# print(time.time() - timer)

	return u_in[nsteps+1], variables

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	u_in, update_var = MPC_solver(actual=0, desired=3, limit=100, origin=0, nsteps=3, ret_points=True)
	# print(update_var.get("points"))
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)