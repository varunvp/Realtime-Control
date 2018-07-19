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

def MPC_solver(r_actual=0., r_desired=0., r_limit=1000, r_origin=0, t_actual=0., t_desired=0., t_origin=0, nsteps=10.,interval=0.1, ret_points = False, lin_vel_limit = 10000, ang_vel_limit = 10000, variables = None):
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
	# while True:
	delta = 0.5
	global prev_nsteps, prev_interval
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

		#Concatenate dynamic and terminal LHS constraint
		r_A_eq = np.row_stack((dyn_A, term_A))

	r_term_b = np.array([r_actual - r_desired])
	theta_term_b = np.array([t_actual - t_desired])
	
	#Concatenate dynamic and terminal RHS constraint
	r_b_eq = np.concatenate((dyn_b, r_term_b))
	theta_b_eq = np.concatenate((dyn_b, theta_term_b))
	big_b_eq = np.concatenate((r_b_eq, theta_b_eq))

	#Inequality constraints(Boundary constraints)
	if nsteps != prev_nsteps:
		pos_r_constraint = np.eye(nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		neg_r_constraint = np.eye(nsteps) * delta - np.eye(nsteps, nsteps, 1)
		pos_constraint = np.row_stack( (pos_r_constraint, neg_r_constraint) )

		pos_theta_constraint = np.eye(nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		neg_theta_constraint = np.eye(nsteps) * delta - np.eye(nsteps, nsteps, 1)
		theta_pos_constraint = np.row_stack((pos_theta_constraint, neg_theta_constraint))

		pos_r_vel_constraint = pos_theta_vel_constraint = np.eye(nsteps)
		neg_r_vel_constraint = neg_theta_vel_constraint = np.eye(nsteps) * -1

		vel_constraint = np.row_stack((pos_r_vel_constraint, neg_r_vel_constraint))
		r_Ba_ineq = block_diag(pos_constraint, vel_constraint)
		theta_vel_constraints = np.row_stack((pos_theta_vel_constraint, neg_theta_vel_constraint))
		theta_Ba_ineq = block_diag(theta_pos_constraint, theta_vel_constraints)
		# big_Ba_ineq = block_diag(r_Ba_ineq, theta_Ba_ineq)

	#######################################################################################
	# solution = variables.get("solution")				#This is the solution from the previous calculation
	#Check 'solution' for None
	#Chaitanya's code here. Put variables that won't change with nstep's or time interval's change inside the above if, for caching
	#######################################################################################

	lin_vel_limit_vec = np.ones(2 * nsteps) * lin_vel_limit			#
	ang_vel_limit_vec = np.ones(2 * nsteps) * ang_vel_limit
	r_Bb_ineq = np.concatenate((np.ones(nsteps) * ((r_origin + r_limit - r_desired) * (1 - delta)), np.ones(nsteps) * (-(r_origin - r_limit - r_desired) *  (1 - delta)), lin_vel_limit_vec))
	#t_limit = 400 is assumed as arbitrary value above 360 
	theta_Bb_ineq = np.concatenate((np.ones(nsteps) * ((t_origin + 370 - t_desired) * (1 - delta)), np.ones(nsteps) * (-(t_origin - 370 - t_desired) *  (1 - delta)), ang_vel_limit_vec))
	big_Bb_ineq = np.concatenate((r_Bb_ineq, theta_Bb_ineq))

	#Relaxation
	if (nsteps != prev_nsteps or interval != prev_interval):
		big_H = block_diag([1000], big_I, [1000], big_I)
		big_h = np.concatenate(([0], big_0, [0], big_0))
		r_A_eq = np.column_stack((np.zeros(nsteps + 1), r_A_eq))
		r_A_eq[nsteps-1][0] = -1
		r_A_eq[nsteps][1] = 1
		theta_A_eq = r_A_eq
		big_A_eq = block_diag(r_A_eq, theta_A_eq)

		r_Ba_ineq = np.column_stack((np.transpose(np.zeros(np.size(r_Ba_ineq, 0))), r_Ba_ineq))
		r_Ba_ineq[nsteps-1][0] = 1
		r_Ba_ineq[2 * nsteps-1][0] = -1

		theta_Ba_ineq = np.column_stack((np.transpose(np.zeros(np.size(theta_Ba_ineq, 0))), theta_Ba_ineq))
		theta_Ba_ineq[nsteps-1][0] = 1
		theta_Ba_ineq[2 * nsteps-1][0] = -1

		big_Ba_ineq = block_diag(r_Ba_ineq, theta_Ba_ineq)

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

	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba_ineq, big_Bb_ineq, big_A_eq, big_b_eq)

	if (nsteps != prev_nsteps or interval != prev_interval):
		variables = {"big_A_eq": big_A_eq, "big_Ba_ineq": big_Ba_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "solution": u_in}

		prev_nsteps = nsteps
		prev_interval = interval

	# print(u_in)
	# print(time.time() - timer)

	return u_in[nsteps+1], u_in[3 * nsteps+2], variables

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	lin_u, ang_u, update_var = MPC_solver(r_actual=0, r_desired=.001, r_limit=2.828, r_origin=0, t_actual = 361, t_desired = 314, t_origin = .9529, nsteps=3, lin_vel_limit = .1, ang_vel_limit = .1)
	# print(update_var.get("points"))
	# print lin_u, ang_u
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)