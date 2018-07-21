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

def MPC_solver(actual=0., desired=0., pos_limit=1000, origin=0, nsteps=10.,interval=0.1, ret_points = False, vel_limit = 10000, variables = None):
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
	delta = 0.5 						#gain for barrier (h(k) >= -delta*h(k-1))
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
		big_I = np.eye(2*nsteps)			
		big_0 = np.zeros(2*nsteps)
	
	#Get dynamic & terminal constraints
	if (nsteps != prev_nsteps or interval != prev_interval):
		# [1 -1  0 t 0 0]
		# [0  1 -1 0 t 0] 
		# [0  0  1 0 0 t]
		dyn_A = np.column_stack((np.eye(nsteps , dtype=float) + np.eye(nsteps, nsteps , 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	#C
		dyn_b = np.zeros(nsteps, dtype=float)
		term_A = np.zeros((2*nsteps), dtype=float)

		#Concatenate dynamic and terminal constraint
		big_A_eq = np.row_stack((dyn_A, term_A))


	term_b = np.array([actual-desired])
	# term_b = np.array([0])
	
	#Concatenate dynamic and terminal constraint
	big_b_eq = np.concatenate((dyn_b, term_b))

	#Inequality constraints(Boundary & velocity constraints)
	if nsteps != prev_nsteps:
		positive_pos_constraint = np.eye(nsteps, nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		negative_pos_constraint = np.eye(nsteps, nsteps) * delta - np.eye(nsteps, nsteps, 1)
		pos_constraint = np.row_stack( (positive_pos_constraint, negative_pos_constraint) )
		positive_vel_constraint = np.eye(nsteps)
		negative_vel_constraint = np.eye(nsteps) * -1
		# vel_constraint = np.zeros((2 * nsteps, nsteps))
		vel_constraint = np.row_stack((positive_vel_constraint, negative_vel_constraint))
		big_Ba_ineq = block_diag(pos_constraint, vel_constraint)

	# big_Bb_ineq = np.concatenate((np.ones(nsteps) * (pos_limit + origin - desired), np.ones(nsteps) * (pos_limit - origin + desired)))
	max_vel_limit = np.ones(2 * nsteps) * vel_limit
	big_Bb_ineq = np.concatenate((np.ones(nsteps) * ((origin + pos_limit - desired) * (1 - delta)), np.ones(nsteps) * (-(origin - pos_limit - desired) *  (1 - delta))))
	big_Bb_ineq = np.concatenate((big_Bb_ineq, max_vel_limit))

	#Relaxation
	if (nsteps != prev_nsteps or interval != prev_interval):
		big_H = block_diag([1000], big_I)
		big_h = np.concatenate(([0], big_0))
		big_A_eq = np.column_stack((np.zeros(nsteps + 1), big_A_eq))
		big_A_eq[nsteps-1][0] = -1
		big_A_eq[nsteps][1] = 1
		#Add additional column for relaxation
		big_Ba_ineq = np.column_stack((np.transpose(np.zeros(np.size(big_Ba_ineq, 0))), big_Ba_ineq))
		big_Ba_ineq[nsteps-1][0] = 1
		big_Ba_ineq[2 * nsteps-1][0] = -1
		
	# print(big_H)
	# print((big_h))
	# print((big_Ba_ineq))
	# print((big_Bb_ineq))
	# print((big_A_eq))
	# print((big_b_eq))
	# print(np.shape(big_H))
	# print(np.shape(big_h))
	# print(np.shape(big_Ba_ineq))
	# print(np.shape(big_Bb_ineq))
	# print(np.shape(big_A_eq))
	# print(np.shape(big_b_eq))
	
	#Calculate solution
	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba_ineq, big_Bb_ineq, big_A_eq, big_b_eq)

	if (nsteps != prev_nsteps or interval != prev_interval):
		if ret_points == True:
			variables = {"big_A_eq": big_A_eq, "big_Ba_ineq": big_Ba_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "points": u_in[1:nsteps+1]}

		else:
			variables = {"big_A_eq": big_A_eq, "big_Ba_ineq": big_Ba_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval}

		prev_nsteps = nsteps
		prev_interval = interval

	# print(u_in)
	# print(time.time() - timer)

	return u_in[nsteps+1], variables

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	u_in, update_var = MPC_solver(actual=0, desired=3, pos_limit=100, origin=0, nsteps=3, ret_points=True, vel_limit = 2)
	# print(update_var.get("points"))
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)