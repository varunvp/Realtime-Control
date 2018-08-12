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
big_I = big_0= big_A_eq= dyn_A= dyn_b= term_A= term_b= x_pos_constraint= x_vel_constraint = np.empty(2)
big_H= big_h= big_A_eq= big_Ba_ineq = np.empty(2)
big_barrier_cons_A = None 

# def MPC_solver(x_actual=0., x_desired=0., x_limit=1000, x_origin=0, y_actual=0., y_desired=0., y_origin=0, nsteps=10.,interval=0.1, ret_points = False, lin_vel_limit = 10000, ang_vel_limit = 10000, variables = None):
def MPC_solver(x_actual=0., x_desired=0., x_destination=0., x_limit=1000, x_origin=0, y_actual=0., y_desired=0., y_destination=0.,y_origin=0, y_limit = 1000, nsteps=10.,interval=0.1, **kwargs):
	"""MPC which uses Quadratic Programming solver
	
	Keyword Arguments:
		actual {float} -- The current value relative to the desired (default: {0.})
		desired {float} -- The desired value (default: {0.})
		nsteps {float} -- Number of steps (default: {10.})
		interval {float} -- Time Interval (default: {0.1})
		variables {dict} -- Returns cached variables (default: {None})
		ret_points {bool} -- Enable to return points in {variables}, under "points" (default: {False})
		lin_vel_limit {float} -- Linear velocity limit (default: {10000})
		ang_vel_limit {float} -- Angular velocity limit (default: {10000})
		obstacles {list} -- List of obstacle specs [[obs_x], [obs_y], [obs_r]] (default: {None})
		vehicle_r {float} -- Radius of vehicle (default: {0})
	
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

	if obstacles != None and len(obstacles) != 0:
		obs_x = obstacles[0]
		obs_y = obstacles[1]
		obs_r = obstacles[2]
	vehicle_r = kwargs.pop("vehicle_r", 0)

	if(kwargs):
		raise TypeError('Unexpected **kwargs: %r' % kwargs)

	global prev_nsteps, prev_interval, big_barrier_cons_A
	#Retrieve cached variables, if any
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
		x_A_eq = np.row_stack((dyn_A, term_A))

	x_term_b = np.array([x_actual])
	y_term_b = np.array([y_actual])
	
	#Concatenate dynamic and terminal RHS constraint
	x_b_eq = np.concatenate((dyn_b, x_term_b))
	y_b_eq = np.concatenate((dyn_b, y_term_b))
	big_b_eq = np.concatenate((x_b_eq, y_b_eq))

	#Inequality constraints(Boundary constraints)
	if nsteps != prev_nsteps:
		pos_x_constraint = np.eye(nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		neg_x_constraint = np.eye(nsteps) * delta - np.eye(nsteps, nsteps, 1)
		x_pos_constraint = np.row_stack( (pos_x_constraint, neg_x_constraint) )

		pos_y_constraint = np.eye(nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		neg_y_constraint = np.eye(nsteps) * delta - np.eye(nsteps, nsteps, 1)
		y_pos_constraint = np.row_stack((pos_y_constraint, neg_y_constraint))

		pos_r_vel_constraint = pos_theta_vel_constraint = np.eye(nsteps)
		neg_r_vel_constraint = neg_theta_vel_constraint = np.eye(nsteps) * -1

		x_vel_constraint = np.row_stack((pos_r_vel_constraint, neg_r_vel_constraint))
		x_Ba_ineq = block_diag(x_pos_constraint, x_vel_constraint)
		y_vel_constraints = np.row_stack((pos_theta_vel_constraint, neg_theta_vel_constraint))
		y_Ba_ineq = block_diag(y_pos_constraint, y_vel_constraints)

	lin_vel_limit_vec = np.ones(2 * nsteps) * lin_vel_limit			#
	ang_vel_limit_vec = np.ones(2 * nsteps) * ang_vel_limit
	r_Bb_ineq = np.concatenate((np.ones(nsteps) * ((x_origin + x_limit) * (1 - delta)), np.ones(nsteps) * (-(x_origin - x_limit) *  (1 - delta)), lin_vel_limit_vec))
	theta_Bb_ineq = np.concatenate((np.ones(nsteps) * ((y_origin + y_limit) * (1 - delta)), np.ones(nsteps) * (-(y_origin - y_limit) *  (1 - delta)), ang_vel_limit_vec))
	big_Bb_ineq = np.concatenate((r_Bb_ineq, theta_Bb_ineq))

	#Relaxation
	if (nsteps != prev_nsteps or interval != prev_interval):
		big_H = block_diag([1000], big_I, [1000], big_I)
		big_h = np.concatenate(([0], big_0, [0], big_0))
		x_A_eq = np.column_stack((np.zeros(nsteps + 1), x_A_eq))
		x_A_eq[nsteps-1][0] = -1
		x_A_eq[nsteps][1] = 1
		y_A_eq = x_A_eq
		big_A_eq = block_diag(x_A_eq, y_A_eq)


		x_Ba_ineq = np.column_stack((np.transpose(np.zeros(np.size(x_Ba_ineq, 0))), x_Ba_ineq))
		x_Ba_ineq[nsteps-1][0] = 1
		x_Ba_ineq[2 * nsteps-1][0] = -1

		y_Ba_ineq = np.column_stack((np.transpose(np.zeros(np.size(y_Ba_ineq, 0))), y_Ba_ineq))
		y_Ba_ineq[nsteps-1][0] = 1
		y_Ba_ineq[2 * nsteps-1][0] = -1

		big_Ba_ineq = block_diag(x_Ba_ineq, y_Ba_ineq)

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
	
	traj = u_in
	#Successive convexification for obstacle avoidance
	if obstacles != None and len(obstacles) != 0:
		obs_x = np.subtract(x_destination, obs_x)
		obs_y = np.subtract(y_destination, obs_y)
		obs_r = np.add(obs_r, vehicle_r)
		iterations = 0

		while(d > 0.01 and iterations < 10):
			iterations = iterations + 1
			
			x_in = traj[1:nsteps+1]
			y_in = traj[2 * nsteps + 2:2 * nsteps + 1 + nsteps + 1 ]
			
			A_ineq_d = big_Ba_ineq
	 		B_ineq_d = big_Bb_ineq - np.dot(A_ineq_d, traj)

			A_eq_d = big_A_eq
			b_eq_d = big_b_eq - np.dot(big_A_eq, traj)
				
			for j in range(0, len(obstacles[0])):
				barrier_cons_A = np.zeros((nsteps, 4 * nsteps + 2))
				barrier_cons_B = np.zeros(nsteps)
				h0 = obs_r[j] **2 - (x_in[0] - obs_x[j])**2 - (y_in[0] - obs_y[j])**2

				for i in range(1, nsteps):
					h = obs_r[j] **2 - (x_in[i] - obs_x[j])**2 - (y_in[i] - obs_y[j])**2
					h_prev = obs_r[j] **2 - (x_in[i-1] - obs_x[j])**2 - (y_in[i-1] - obs_y[j])**2
					gamma = 0.2

					barrier_cons_A[i][i+1] = -2 * (x_in[i] - obs_x[j])
					barrier_cons_A[i][1 + 2 * nsteps + 1 + i] = -2 * (y_in[i]- obs_y[j])
					
					barrier_cons_A[i][i] = gamma*2 * (x_in[i-1] - obs_x[j])
					barrier_cons_A[i][1 + 2 * nsteps + i] = gamma* 2 * (y_in[i-1]- obs_y[j])
					# barrier_cons_B[i] = - h #- 0.9 * h0
					
					barrier_cons_B[i] = - h + gamma * h_prev
					
					# barrier_cons_B[i] = - h + 0.9 * h0
					# h0 = h

				A_ineq_d = np.vstack((A_ineq_d, barrier_cons_A))
		 		B_ineq_d = np.concatenate((B_ineq_d, barrier_cons_B))
	 		
			d_traj_out = qp_matrix.quadprog_solve_qp(big_H, big_h, A_ineq_d, B_ineq_d, A_eq_d, b_eq_d)
			traj += d_traj_out
			d = np.linalg.norm(d_traj_out)

	# print(iterations, d)
	print(time.time() - timer)

	if (nsteps != prev_nsteps or interval != prev_interval):
	# if(True):
		variables = {"big_A_eq": big_A_eq, "big_Ba_ineq": big_Ba_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "solution": traj}

		prev_nsteps = nsteps
		prev_interval = interval

	else:
		variables = {"solution": traj}

	return traj[nsteps+1], traj[3 * nsteps+2], variables

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	#Some calls for standalone testing of solver
	lin_u, ang_u, update_var = MPC_solver(x_actual=0, x_destination=2, x_limit=200, x_origin=0, y_actual = 0, y_destination = 0, y_origin = 0,y_limit = 200 , nsteps=20, variables=None)

	# MPC_solver(0, 2, 100, 0, 0, .5, 100, 0, 10, variables=update_var)
	# MPC_solver(0, 3, 100, 0, 1, 4, 100, 0, 10, variables=update_var)