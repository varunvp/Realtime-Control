#!/usr/bin/python
import qp_matrix, time, math
from numpy import array
import numpy as np
from scipy.linalg import block_diag

prev_nsteps = 0
prev_interval = 0
big_I = big_0= big_A_eq= dyn_A= dyn_b= term_A= term_b= x_pos_constraint= x_vel_constraint = np.empty(2)
big_H= big_h= big_A_eq= big_Ba_ineq = np.empty(2)
big_barrier_cons_A = None 
counter = 0
max_time = 0.
min_time = 1000.

def project_to_obstacles(x_in, y_in, x_obs, y_obs, r_obs, nsteps):
	x_proj = np.zeros(np.shape(x_in))
	y_proj = np.zeros(np.shape(y_in))

	for i in range(0, nsteps):
		#dist = sqrt(x^2 - y^2)
		dist = math.sqrt((x_in[i] - x_obs)**2 + (y_in[i] - y_obs)**2)
		#x_proj = x_o + r_o / dist * (x(i) - x_o)
		x_proj[i] = x_obs + r_obs / dist * (x_in[i] - x_obs)
		#y_proj = y_o + r_o / dist * (y(i) - y_o)
		y_proj[i] = y_obs + r_obs / dist * (y_in[i] - y_obs)

	return x_proj, y_proj

def MPC_solver(init_pose, current_pose, final_pose, x_limit=1000, y_limit = 1000, nsteps=10.,interval=0.1, **kwargs):
	"""MPC which uses Quadratic Programming solver
	
	Keyword Arguments:
		actual {float} -- The current position relative to the desired (default: {0.})
		destination {float} -- The desired position (default: {0.})
		limit {float} -- Box constraint limits (default: {0.})
		nsteps {float} -- Number of steps (default: {10.})
		interval {float} -- Time Interval (default: {0.1})
		variables {dict} -- Returns cached variables (default: {None})
		ret_points {bool} -- Enable to return points in {variables}, under "points" (default: {False})
		x_vel_limit {float} -- Linear velocity limit (default: {10000})
		y_vel_limit {float} -- Angular velocity limit (default: {10000})
		obstacles {list} -- List of obstacle specs [[x_obs], [y_obs], [r_obs]] (default: {None})
		r_vehicle {float} -- Radius of vehicle (default: {0})
	
	Returns:
		float -- Solution v1(0)
		float -- Solution v2(0)
		dict -- Cached variables

	"""
	x_origin, y_origin, theta_init = init_pose
	x_actual, y_actual, theta_actual = current_pose
	x_destination, y_destination, theta_destination = final_pose

	delta = 0.5
	d = 10000

	ret_points = kwargs.pop("ret_points", False)
	x_vel_limit = kwargs.pop("x_vel_limit", 10000)
	y_vel_limit = kwargs.pop("y_vel_limit", 10000)
	variables = kwargs.pop("variables", None)
	obstacles = kwargs.pop("obstacles", None)

	if obstacles != None and len(obstacles) != 0:
		x_obs = obstacles[0]
		y_obs = obstacles[1]
		r_obs = obstacles[2]
		vx_obs = obstacles[3]
		vy_obs = obstacles[4]

	r_vehicle = kwargs.pop("r_vehicle", 0)

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
		dyn_A = np.column_stack((np.eye(nsteps , dtype=float) + np.eye(nsteps, nsteps , 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	
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
		#LHS
		#Positive and negative velocity constraints for x and y velocities
		#v(n) <= B
		pos_x_vel_constraint = pos_y_vel_constraint = np.eye(nsteps)
		neg_x_vel_constraint = neg_y_vel_constraint = np.eye(nsteps) * -1

		#Positive and negative box constraints for x and y positions
		#x(n) +/- delta * x(n+1) <= B
		pos_x_constraint = np.eye(nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		neg_x_constraint = np.eye(nsteps) * delta - np.eye(nsteps, nsteps, 1)
		x_pos_constraint = np.row_stack( (pos_x_constraint, neg_x_constraint) )

		#y(n) +/- delta * y(n+1) <= B
		pos_y_constraint = np.eye(nsteps) * -delta + np.eye(nsteps, nsteps, 1)
		neg_y_constraint = np.eye(nsteps) * delta - np.eye(nsteps, nsteps, 1)
		y_pos_constraint = np.row_stack((pos_y_constraint, neg_y_constraint))

		#Constraints concatenation
		x_vel_constraint = np.row_stack((pos_x_vel_constraint, neg_x_vel_constraint))
		x_Ba_ineq = block_diag(x_pos_constraint, x_vel_constraint)
		y_vel_constraints = np.row_stack((pos_y_vel_constraint, neg_y_vel_constraint))
		y_Ba_ineq = block_diag(y_pos_constraint, y_vel_constraints)

	#RHS
	#Positive and negative velocity constraints for x and y velocities
	#Ax <= +/-v_lim
	x_vel_limit_vec = np.ones(2 * nsteps) * x_vel_limit
	y_vel_limit_vec = np.ones(2 * nsteps) * y_vel_limit

	#Positive and negative box constraints for x and y positions
	#Ax = +/-((x_orig + x_lim) * (1 - delta))
	x_Bb_ineq = np.concatenate((np.ones(nsteps) * ((x_origin + x_limit) * (1 - delta)), np.ones(nsteps) * (-(x_origin - x_limit) *  (1 - delta)), x_vel_limit_vec))
	y_Bb_ineq = np.concatenate((np.ones(nsteps) * ((y_origin + y_limit) * (1 - delta)), np.ones(nsteps) * (-(y_origin - y_limit) *  (1 - delta)), y_vel_limit_vec))
	big_Bb_ineq = np.concatenate((x_Bb_ineq, y_Bb_ineq))

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

	#Obstacle fre path
	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_Ba_ineq, big_Bb_ineq, big_A_eq, big_b_eq)

	traj = u_in
	#Successive convexification for obstacle avoidance
	if obstacles != None and len(obstacles) != 0:
		x_obs = np.subtract(x_destination, x_obs)
		y_obs = np.subtract(y_destination, y_obs)
		r_obs = np.add(r_obs, r_vehicle)
		iterations = 0

		while(d > 0.1 and iterations < 10):
			iterations = iterations + 1
			
			x_in = traj[1:nsteps+1]
			y_in = traj[2 * nsteps + 2:2 * nsteps + 1 + nsteps + 1 ]
			
			#Ai*d <= Be - Ai * x
			A_ineq_d = big_Ba_ineq
	 		B_ineq_d = big_Bb_ineq - np.dot(A_ineq_d, traj)

	 		#Ae*d <= be - Ae * x
			A_eq_d = big_A_eq
			b_eq_d = big_b_eq - np.dot(big_A_eq, traj)
				
			# x_proj = np.zeros(np.shape(x_in))
			# y_proj = np.zeros(np.shape(y_in))

			for j in range(0, len(obstacles[0])):
				barrier_cons_A = np.zeros((nsteps, 4 * nsteps + 2))
				barrier_cons_B = np.zeros(nsteps)

				# x_proj, y_proj = project_to_obstacles(x_in, y_in, x_obs[j], y_obs[j], r_obs[j], nsteps)

				# for i in range(0, nsteps):
				# dist = math.sqrt((x_in[0] - x_obs[j])**2 + (y_in[0] - y_obs[j])**2)
				# x_proj[0] = x_obs[j] + r_obs[j] / dist * (x_in[0] - x_obs[j])
				# y_proj[0] = y_obs[j] + r_obs[j] / dist * (y_in[0] - y_obs[j])

				for i in range(1, nsteps):
					prediction_time = - i * interval

					x_proj, y_proj = project_to_obstacles(x_in, y_in, x_obs[j] + vx_obs * prediction_time, y_obs[j] + vy_obs * prediction_time, r_obs[j], nsteps)

					#h = r**2 - x(i)**2 - y(i)**2
					h = r_obs[j] **2 - (x_in[i] - (x_obs[j] + vx_obs * prediction_time))**2 - (y_in[i] - (y_obs[j] + vy_obs * prediction_time))**2
					#h_prev = r**2 - x(i-1)**2 - y(i-1)**2
					h_prev = r_obs[j] **2 - (x_in[i-1] - (x_obs[j] + vx_obs * prediction_time))**2 - (y_in[i-1] - (y_obs[j] + vy_obs * prediction_time))**2
					gamma = 0.2

					# dist = math.sqrt((x_in[i] - x_obs[j])**2 + (y_in[i] - y_obs[j])**2)
					# x_proj[i] = x_obs[j] + r_obs[j] / dist * (x_in[i] - x_obs[j])
					# y_proj[i] = y_obs[j] + r_obs[j] / dist * (y_in[i] - y_obs[j])

					#Ai <= -2 * (x(i) - x_o)
					barrier_cons_A[i][i+1] = -2 * (x_proj[i] - (x_obs[j] + vx_obs * prediction_time))
					#Ai <= -2 * (y(i) - y_o)
					barrier_cons_A[i][1 + 2 * nsteps + 1 + i] = -2 * (y_proj[i] - (y_obs[j] + vy_obs * prediction_time))
					
					#Ai <= gamma * 2 * (x(i-1) - x_o)
					barrier_cons_A[i][i] = gamma * 2 * (x_proj[i-1] - (x_obs[j] + vx_obs * prediction_time))
					#Ai <= gamma * 2 * (y(i-1) - y_o)
					barrier_cons_A[i][1 + 2 * nsteps + i] = gamma * 2 * (y_proj[i-1] - (y_obs[j] + vy_obs * prediction_time))
					# barrier_cons_B[i] = - h #- 0.9 * h0

					#h(k+1) >= gamma * h(k)
					barrier_cons_B[i] = - h + gamma * h_prev
					# print barrier_cons_A, barrier_cons_B

				A_ineq_d = np.vstack((A_ineq_d, barrier_cons_A))
		 		B_ineq_d = np.concatenate((B_ineq_d, barrier_cons_B))
	 		
			d_traj_out = qp_matrix.quadprog_solve_qp(big_H, big_h, A_ineq_d, B_ineq_d, A_eq_d, b_eq_d)
			traj += d_traj_out
			d = np.linalg.norm(d_traj_out)

	# print(iterations, d)
	# print big_H
	# print big_h
	# print A_ineq_d
	# print B_ineq_d
	# print A_eq_d
	# print b_eq_d
	# print x_A_eq
	

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
	# lin_u, ang_u, update_var = MPC_solver(x_actual=.7, x_destination=.7, x_limit=200, x_origin=0, y_actual = .7, y_destination = .7, y_origin = 0,y_limit = 200 , nsteps=3, interval = .05 ,variables=None, obstacles = [[.5],[.5],[.1]], x_vel_limit = 2, y_vel_limit = 2)
	lin_u, ang_u, update_var = MPC_solver(init_pose=[0,0,0],current_pose=[.7,.7,0],final_pose=[.7,.7,0],  x_actual=.7, x_destination=.7, x_limit=200, x_origin=0, y_actual = .7, y_destination = .7, y_origin = 0,y_limit = 200 , nsteps=3, interval = .05 ,variables=None, obstacles = [[.5],[.5],[.1]], x_vel_limit = 2, y_vel_limit = 2)
	# MPC_solver(0, 2, 100, 0, 0, .5, 100, 0, 10, variables=update_var)
	# MPC_solver(0, 3, 100, 0, 1, 4, 100, 0, 10, variables=update_var)


