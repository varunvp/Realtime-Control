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
big_barrier_cons_A = None 

# def MPC_solver(r_actual=0., r_desired=0., r_limit=1000, r_origin=0, t_actual=0., t_desired=0., t_origin=0, nsteps=10.,interval=0.1, ret_points = False, lin_vel_limit = 10000, ang_vel_limit = 10000, variables = None):
def MPC_solver(r_actual=0., r_desired=0., r_destination=0., r_limit=1000, r_origin=0, t_actual=0., t_desired=0.,t_destination=0.,t_origin=0, t_limit = 1000, nsteps=10.,interval=0.1, **kwargs):
	"""MPC which uses Quadratic Programming solver
	
	Keyword Arguments:
		actual {float} -- The current value (default: {0.})
		desired {float} -- The desired value (default: {0.})
		nsteps {float} -- Number of steps (default: {10.})
		interval {float} -- Time Interval (default: {0.1})
		variables {dict} -- Returns cached variables (default: {None})
		ret_points {bool} -- Enable to return points in {variables}, under "points" (default: {False})
	
	Returns:
		float -- Solution v1(0)
		float -- Solution v2(0)
		dict -- Cached variables

	"""
	delta = 0.5
	d = 10000
	ret_points = kwargs.pop("ret_points", False)
	lin_vel_limit = kwargs.pop("lin_vel_limit", 10000)
	ang_vel_limit = kwargs.pop("ang_vel_limit", 10000)
	variables = kwargs.pop("variables", None)
	obstacles = kwargs.pop("obstacles", None)
	if obstacles != None:
		obs_x = obstacles[0]
		obs_y = obstacles[1]
		obs_r = obstacles[2]
	vehicle_r = kwargs.pop("vehicle_r", 0)

	if(kwargs):
		raise TypeError('Unexpected **kwargs: %r' % kwargs)

	global prev_nsteps, prev_interval, big_barrier_cons_A
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
	theta_Bb_ineq = np.concatenate((np.ones(nsteps) * ((t_origin + t_limit - t_desired) * (1 - delta)), np.ones(nsteps) * (-(t_origin - t_limit - t_desired) *  (1 - delta)), ang_vel_limit_vec))
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
	# # print((big_A_eq))
	# print((big_b_eq))
	# print(np.shape(big_H))
	# print(np.shape(big_h))
	# print(np.shape(big_Ba_ineq))
	# print(np.shape(big_Bb_ineq))
	# print(np.shape(big_A_eq))
	# print(np.shape(big_b_eq))

	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba_ineq, big_Bb_ineq, big_A_eq, big_b_eq)
	
	traj = u_in
	#Here r des is x des, t des is y des

	if obstacles != None:
		obs_x = np.subtract(r_destination, obs_x)
		obs_y = np.subtract(t_destination, obs_y)
		obs_r = np.add(obs_r, vehicle_r)
		# obs_x = np.array([r_destination - 4, r_destination - 1, r_destination - 2])
		# obs_y = np.array([t_destination - 3, t_destination - 3, t_destination - 1])
		# obs_r = np.array([1.5 + 0.05, 1 + 0.05, 0.75 + 0.05])  # 3m plus 5 cm to account for extra margins due to robot dimensions
		ii = 0

		while(d > 0.01 and ii < 10):
			ii = ii + 1
			##########################Obstacle####################
			# def get_obstacle_constraints(obs_x, obs_y, obs_r, traj, nsteps):
			x_in = traj[1:nsteps+1]
			y_in = traj[2 * nsteps + 2:2 * nsteps + 1 + nsteps + 1 ]
			# print(np.shape(x_in), np.shape(y_in))
			
			A_ineq_d = big_Ba_ineq
	 		B_ineq_d = big_Bb_ineq - np.dot(A_ineq_d, traj)

			A_eq_d = big_A_eq
			b_eq_d = big_b_eq - np.dot(big_A_eq, traj)
				
			for j in range(0, 3):
				barrier_cons_A = np.zeros((nsteps, 4 * nsteps + 2))
				barrier_cons_B = np.zeros(nsteps)

				for i in range(1, nsteps-1):
					h = obs_r[j] **2 - (x_in[i] - obs_x[j])**2 - (y_in[i] - obs_y[j])**2

					barrier_cons_A[i][i+1] = -2 * (x_in[i] - obs_x[j])
					barrier_cons_A[i][2 * nsteps + 1 + i + 1] = -2 * (y_in[i]- obs_y[j])
					barrier_cons_B[i] = -h		

				A_ineq_d = np.vstack((A_ineq_d, barrier_cons_A))
		 		B_ineq_d = np.concatenate((B_ineq_d, barrier_cons_B))
	 		
			d_traj_out = qp_matrix.quadprog_solve_qp(big_H, big_h, A_ineq_d, B_ineq_d, A_eq_d, b_eq_d)
			traj = traj + d_traj_out
			d = np.linalg.norm(d_traj_out)
			# print(d)

	# print(ii, d)
	# print(time.time() - timer)

	# if (nsteps != prev_nsteps or interval != prev_interval):
	if(True):
		variables = {"big_A_eq": big_A_eq, "big_Ba_ineq": big_Ba_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "solution": traj}

		prev_nsteps = nsteps
		prev_interval = interval

	# print(traj)
	# print(np.shape(u_in))

	return traj[nsteps+1], traj[3 * nsteps+2], variables

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	lin_u, ang_u, update_var = MPC_solver(r_actual=10, r_desired=0, r_limit=200, r_origin=0, t_actual = 9, t_desired = 0, t_origin = 0,t_limit = 200 , nsteps=20, variables=None)
	# print(update_var.get("points"))
	# print lin_u, ang_u
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)
	# MPC_solver(0, 3, 100, 0, 10, variables=update_var)