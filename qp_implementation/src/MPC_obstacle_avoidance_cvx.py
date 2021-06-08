#!/usr/bin/python3
import cvxpy as cp
import time, math
from numpy import array
import numpy as np
from scipy.linalg import block_diag
from time import perf_counter

prev_nsteps = 0
prev_interval = 0
big_I = big_0= big_A_eq= dyn_A= dyn_b= term_A= term_b= x_pos_constraint= x_vel_constraint = np.empty(2)
big_H= big_h= big_A_eq= big_A_ineq = np.empty(2)
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
	d_norm = 10000

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
	print(obstacles)
	if(kwargs):
		raise TypeError('Unexpected **kwargs: %r' % kwargs)

	global prev_nsteps, prev_interval, big_barrier_cons_A
	#Retrieve cached variables, if any
	if variables:
		big_A_eq = variables.get("big_A_eq") 
		big_A_ineq = variables.get("big_A_ineq")
		big_H = variables.get("big_H")
		big_h = variables.get("big_h")
		prev_nsteps = variables.get("prev_nsteps")
		prev_interval = variables.get("prev_interval")

	timer = time.time()

	x_term_offset = 2 * nsteps + 1

	if nsteps != prev_nsteps:
		# [1 0 0    0]
		# [0 1 0    0] 
		# [0 0 1000 0]				Relaxation for x_k, k = N
		# [0 0 0    1]
		big_I = np.eye(2*nsteps + 1)
		big_I[nsteps][nsteps] = 1000
		# [0 0 0].T
		big_0 = np.zeros(2*nsteps + 1)
	
	#Get dynamic & terminal constraints
	if (nsteps != prev_nsteps or interval != prev_interval):
		# [1 -1  0  0 t 0 0]
		# [0  1 -1  0 0 t 0] 
		# [0  0  1 -1 0 0 t]
		dyn_A = np.column_stack((np.eye(nsteps, nsteps+1, dtype=float) + np.eye(nsteps, nsteps+1, 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	
		dyn_b = np.zeros(nsteps, dtype=float)

		# [0  0  0 0 0 0]
		term_A = np.zeros((2*nsteps+1), dtype=float)

		#Row stack dynamic and terminal LHS constraint
		x_A_eq = np.row_stack((dyn_A, term_A))

	#x_0 = X_s - X_d (input in this form while calling this function)
	x_term_b = np.array([x_actual])
	y_term_b = np.array([y_actual])
	
	#Concatenate dynamic and terminal RHS constraint
	x_b_eq = np.concatenate((dyn_b, x_term_b))
	y_b_eq = np.concatenate((dyn_b, y_term_b))
	big_B_eq = np.concatenate((x_b_eq, y_b_eq))

	#Inequality constraints(Boundary constraints)
	if nsteps != prev_nsteps:
		#LHS
		#Positive and negative velocity constraints for x and y velocities
		#v(n) <= B
		pos_x_vel_constraint = pos_y_vel_constraint = np.eye(nsteps)
		neg_x_vel_constraint = neg_y_vel_constraint = np.eye(nsteps) * -1

		#Positive and negative box constraints for x and y positions
		# -/+ x(n) * delta +/- x(n+1) <= B
		pos_x_constraint = np.eye(nsteps,nsteps+1) * -delta + np.eye(nsteps, nsteps+1, 1)
		neg_x_constraint = np.eye(nsteps,nsteps+1) * delta - np.eye(nsteps, nsteps+1, 1)
		x_pos_constraint = np.row_stack( (pos_x_constraint, neg_x_constraint) )

		#y(n) * delta +/- y(n+1) <= B
		pos_y_constraint = np.eye(nsteps,nsteps+1) * -delta + np.eye(nsteps, nsteps+1, 1)
		neg_y_constraint = np.eye(nsteps,nsteps+1) * delta - np.eye(nsteps, nsteps+1, 1)
		y_pos_constraint = np.row_stack((pos_y_constraint, neg_y_constraint))

		#Constraints concatenation
		x_vel_constraint = np.row_stack((pos_x_vel_constraint, neg_x_vel_constraint))
		x_A_ineq = block_diag(x_pos_constraint, x_vel_constraint)
		y_vel_constraints = np.row_stack((pos_y_vel_constraint, neg_y_vel_constraint))
		y_A_ineq = block_diag(y_pos_constraint, y_vel_constraints)

	#RHS
	#Positive and negative velocity constraints for x and y velocities
	#Ax <= +/-v_lim
	x_vel_limit_vec = np.ones(2 * nsteps) * x_vel_limit
	y_vel_limit_vec = np.ones(2 * nsteps) * y_vel_limit

	#Positive and negative box constraints for x and y positions
	#Ax <= +/-((x_orig - x_destination + x_lim) * (1 - delta))
	x_B_ineq = np.concatenate((np.ones(nsteps) * ((x_origin - x_destination + x_limit) * (1 - delta)), np.ones(nsteps) * (-(x_origin - x_destination - x_limit) *  (1 - delta)), x_vel_limit_vec))
	y_B_ineq = np.concatenate((np.ones(nsteps) * ((y_origin - y_destination + y_limit) * (1 - delta)), np.ones(nsteps) * (-(y_origin - y_destination - y_limit) *  (1 - delta)), y_vel_limit_vec))
	big_B_ineq = np.concatenate((x_B_ineq, y_B_ineq))

	#Relaxation
	if (nsteps != prev_nsteps or interval != prev_interval):
		big_H = block_diag(big_I, big_I)
		big_h = np.concatenate((big_0, big_0))

		#For x_0, terminal constraint
		x_A_eq[nsteps][0] = 1

		y_A_eq = x_A_eq

		big_A_eq = block_diag(x_A_eq, y_A_eq)
		big_A_ineq = block_diag(x_A_ineq, y_A_ineq)

	print("Q\n",big_H)
	print("R\n",big_h)
	print("Inequality LHS\n",big_A_ineq)
	print("Inequality RHS\n",big_B_ineq)
	print("Equality LHS\n",big_A_eq)
	print("Equality RHS\n",big_B_eq)
	# print(np.shape(big_H))
	# print(np.shape(big_h))
	# print(np.shape(big_A_ineq))
	# print(np.shape(big_B_ineq))
	# print(np.shape(big_A_eq))
	# print(np.shape(big_B_eq))

	t1_start = perf_counter()
	# s = cp.Variable(4*nsteps+2)
	state_cont_pair = cp.Variable(4*nsteps + 2)
	prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(state_cont_pair, big_H) + big_h.T @ state_cont_pair),
                 [big_A_ineq @ state_cont_pair <= big_B_ineq,
                  big_A_eq @ state_cont_pair == big_B_eq])
	prob.solve()
	t1_stop = perf_counter()

	#Format for solution for N steps is [x_0,..., x_{N+1}, vx_0, ... vx_N, y_0,..., y_{N+1}, vy_0, ... vy_N]
	obs_free_traj = state_cont_pair.value
	gamma = 0.2

	print("Obs free soln.\n",obs_free_traj)

	#Successive convexification for obstacle avoidance
	if obstacles != None and len(obstacles) != 0:
		#Transform x_obs wrt to destination
		x_obs = -np.subtract(x_destination, x_obs)
		y_obs = -np.subtract(y_destination, y_obs)
		r_obs = np.add(r_obs, r_vehicle)
		iterations = 0

		y_offset = 2 * nsteps + 1

		while((d_norm >= 0.1) and (iterations <= 10)):
			iterations = iterations + 1
			
			x_prev = obs_free_traj[0:nsteps + 1]
			y_prev = obs_free_traj[y_offset:y_offset + nsteps + 1]

			#Ai*d <= Bi - Ai * x
			A_ineq_d = big_A_ineq
			B_ineq_d = big_B_ineq - big_A_ineq @ obs_free_traj

			#Ae*d = be - Ae * x
			A_eq_d = big_A_eq
			b_eq_d = big_B_eq - big_A_eq @ obs_free_traj
			
			print("A, B when running succ. cvx.")
			print("Inequality LHS\n",big_A_ineq)
			print("Inequality RHS\n",big_B_ineq)
			print("Equality LHS\n",big_A_eq)
			print("Equality RHS\n",big_B_eq)

			soc_constraints = []
				
			for j in range(0, len(obstacles[0])):
				# x_proj, y_proj = project_to_obstacles(x_prev, y_prev, x_obs[j], y_obs[j], r_obs[j], nsteps)

				for k in range(0,nsteps):
					#s_prev
					s_p = np.vstack((x_prev[k], y_prev[k]))

					#s_{k+1}
					s_k_plus_1 = cp.atoms.vstack((x_prev[k+1], y_prev[k+1]))

					#s_k 
					s_k = cp.atoms.vstack((x_prev[k], y_prev[k]))

					#s_o
					s_o = np.vstack((x_obs[j], y_obs[j]))
					print("s_o\n",s_o)

					#s_prev - s_o
					sp_minus_so = s_p - s_o

					#LHS, ||s_{k+1} - s_obs||^2
					CAk_norm = cp.atoms.norm(s_k_plus_1 - s_o)
					LHS = cp.atoms.sum_squares(CAk_norm)

					#RHS, r^2 * (1-gamma) + gamma * ||s_p - s_o||^2 - 2 * gamma * (sp - so).T * sp + 2 * gamma * (sp - so).T * sk
					RHS = r_obs ** 2 * (1 - gamma) + gamma * np.square(np.linalg.norm(sp_minus_so)) - 2 * gamma * (sp_minus_so.T @ s_p) + 2 * gamma * (sp_minus_so.T @ s_k)
					
					soc_constraints.append(LHS <= RHS)

					print(soc_constraints[0])
				
			prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(state_cont_pair, big_H) + big_h.T @ state_cont_pair),
		                 [A_ineq_d @ state_cont_pair <= B_ineq_d,
		                  A_eq_d @ state_cont_pair == b_eq_d] + soc_constraints)
			# prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(state_cont_pair, big_H) + big_h.T @ state_cont_pair),soc_constraints)
			prob.solve()
			d_traj_out = state_cont_pair.value
			print("Solution:\t",state_cont_pair.value)

			obs_free_traj += d_traj_out
			d_norm = np.linalg.norm(obs_free_traj)

			print("Status:\t", prob.status)
			print("d_norm:\t",d_norm)

	# print(iterations, d)

	obs_traj = obs_free_traj

	if (nsteps != prev_nsteps or interval != prev_interval):
		variables = {"big_A_eq": big_A_eq, "big_A_ineq": big_A_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "solution": obs_traj}

		prev_nsteps = nsteps
		prev_interval = interval

	else:
		variables = {"solution": obs_traj}

	return obs_traj[nsteps+1], obs_traj[3 * nsteps+2], variables

if __name__ == "__main__":
	np.set_printoptions(precision=3, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	#Some calls for standalone testing of solver, current pose is the pose of robot wrt to destination, and final_pose is wrt global frame. 
	#Obstacles array format = [x_obs, y_obs, r_obs, vx_obs, vy_obs]
	lin_u, ang_u, update_var = MPC_solver(init_pose=[0,0,0],current_pose=[-10,0,0],final_pose=[10,0,0], x_limit = 100, y_limit = 100, nsteps=1, interval = 1 ,variables=None, obstacles = [[50],[50],[.1],[0.1],[0.1]], x_vel_limit = 2, y_vel_limit = 2)
