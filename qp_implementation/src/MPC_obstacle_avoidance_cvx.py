#!/usr/bin/python3
import cvxpy as cp
import time, math, qp_matrix
from numpy import array
import numpy as np
from scipy.linalg import block_diag
from time import perf_counter
import matplotlib.pyplot as plt
import MPC_obstacle_avoidance, pdb
from matplotlib.offsetbox import (TextArea, DrawingArea, OffsetImage, AnnotationBbox)
from matplotlib.cbook import get_sample_data

prev_nsteps = 0
prev_interval = 0
big_I = big_0= big_A_eq= dyn_A= dyn_b= term_A= term_b= x_pos_constraint= x_vel_constraint = np.empty(2)
big_H= big_h= big_A_eq= big_A_ineq = np.empty(2)
big_barrier_cons_A = None 
total_time = counter = 0.

def project_to_obstacles(x_in, y_in, x_obs, y_obs, r_obs, nsteps):
	x_proj = np.zeros(np.shape(x_in))
	y_proj = np.zeros(np.shape(y_in))

	for i in range(0, nsteps+1):
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
	t1_start = perf_counter()
	x_origin, y_origin, theta_init = init_pose
	x_actual, y_actual, theta_actual = current_pose
	x_destination, y_destination, theta_destination = final_pose

	x_actual = current_pose[0] - x_destination
	y_actual = current_pose[1] - y_destination

	delta = 0.5
	d_norm = 10000

	ret_points = kwargs.pop("ret_points", False)
	x_vel_limit = kwargs.pop("x_vel_limit", 10000)
	y_vel_limit = kwargs.pop("y_vel_limit", 10000)
	variables = kwargs.pop("variables", None)
	obstacles = kwargs.pop("obstacles", None)
	u_in = kwargs.pop("feasible_sol", None)
	gamma = kwargs.pop("gamma", 0.2)

	if obstacles != None and len(obstacles) != 0:
		x_obs = obstacles[0]
		y_obs = obstacles[1]
		r_obs = obstacles[2]
		vx_obs = obstacles[3]
		vy_obs = obstacles[4]

	r_vehicle = kwargs.pop("r_vehicle", 0)

	if(kwargs):
		raise TypeError('Unexpected **kwargs: %r' % kwargs)

	global prev_nsteps, prev_interval, big_barrier_cons_A, counter, total_time
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

	# print("Q\n",big_H)
	# print("R\n",big_h)
	# print("Inequality LHS\n",big_A_ineq)
	# print("Inequality RHS\n",big_B_ineq)
	# print("Equality LHS\n",big_A_eq)
	# print("Equality RHS\n",big_B_eq)
	# print(np.shape(big_H))
	# print(np.shape(big_h))
	# print(np.shape(big_A_ineq))
	# print(np.shape(big_B_ineq))
	# print(np.shape(big_A_eq))
	# print(np.shape(big_B_eq))


	#Format for solution for N steps is [x_0,..., x_{N+1}, vx_0, ... vx_N, y_0,..., y_{N+1}, vy_0, ... vy_N]

	if(u_in is None):
		u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, big_A_ineq, big_B_ineq, big_A_eq, big_B_eq)

	seed_traj = u_in.copy()

	# gamma = 0.2

	y_offset = 2 * nsteps + 1
	x_prev_free = seed_traj[0:nsteps + 1] + x_destination
	y_prev_free = seed_traj[y_offset:y_offset + nsteps + 1] + y_destination
	# print("Obs free soln.\n",obs_free_traj)

	tau_k = 1
	tau_max = 1e7
	mu = 4
	delta_violation = 1e-5
	iterations_max = 1000

	if(variables):
		prob = variables.get("prob")
		state_cont_pair = prob.variables()[0]

		prob.parameters()[0].value = big_A_ineq
		prob.parameters()[1].value = big_B_ineq
		prob.parameters()[2].value = big_A_eq
		prob.parameters()[3].value = big_B_eq

	else:
		state_cont_pair = cp.Variable(4*nsteps + 2)
		tau = cp.Parameter((1,))
		slack = cp.Variable((len(obstacles[0]),nsteps),nonneg=True)
		ineq_lhs, ineq_rhs, eq_lhs, eq_rhs = cp.Parameter(np.shape(big_A_ineq)), cp.Parameter(np.shape(big_B_ineq)), cp.Parameter(np.shape(big_A_eq)), cp.Parameter(np.shape(big_B_eq))

		ineq_lhs.value = big_A_ineq
		ineq_rhs.value = big_B_ineq
		eq_lhs.value = big_A_eq
		eq_rhs.value = big_B_eq

		objective = (1/2)*cp.quad_form(state_cont_pair, big_H) + big_h.T @ state_cont_pair + tau * cp.sum(slack)
		prob = cp.Problem(cp.Minimize(objective),
                 [ineq_lhs @ state_cont_pair <= ineq_rhs,eq_lhs @ state_cont_pair == eq_rhs])

	x_prev = x_prev_free #.copy()
	y_prev = y_prev_free #.copy()

	#Successive convexification for obstacle avoidance
	if obstacles != None and len(obstacles) != 0:
		#Transform x_obs wrt to destination
		x_obs = -np.subtract(x_destination, x_obs)
		y_obs = -np.subtract(y_destination, y_obs)
		r_obs = np.add(r_obs, r_vehicle)
		iterations = 0

		while((d_norm >= 0.1) and (iterations < iterations_max)):
			t3_start = perf_counter()
			iterations = iterations + 1
			
			soc_constraints = []

			#s_prev, k + 1
			s_p_plus_1 = np.concatenate((x_prev[1:], y_prev[1:]))
				
			for j in range(0, len(obstacles[0])):
				x_obs_motion = x_obs[j] + np.arange(0, nsteps, 1) * interval * vx_obs[j]
				y_obs_motion = y_obs[j] + np.arange(0, nsteps, 1) * interval * vy_obs[j]

				obs_motion = np.concatenate((x_obs_motion, y_obs_motion))

				#s_k
				s_k = cp.hstack((state_cont_pair[:nsteps], state_cont_pair[y_offset:y_offset+nsteps]))

				#s_{k+1}
				s_k_plus_1 = cp.hstack((state_cont_pair[1:nsteps+1], state_cont_pair[y_offset + 1: y_offset + nsteps + 1]))
				# pdb.set_trace()
				#||s_{k+1} - s_obs||^2
				sq_norm_term = np.square(np.linalg.norm(np.reshape(np.concatenate((x_prev[1:] - x_obs_motion, y_prev[1:] - y_obs_motion)), (nsteps,2), order='F'), axis=1))

				#(s_prev, k + 1) - s_o
				sp_plus_1_minus_so = np.column_stack((np.diag(x_prev[1:] - x_obs_motion), np.diag(y_prev[1:] - y_obs_motion)))

				if(variables):
					prob.parameters()[4+j*4+0].value = obs_motion
					prob.parameters()[4+j*4+1].value = sq_norm_term
					prob.parameters()[4+j*4+2].value = sp_plus_1_minus_so
					prob.parameters()[4+j*4+3].value = s_p_plus_1

				else:
					sq_norm_param, sp_plus_1_minus_so_param, s_p_plus_1_param, so_param = cp.Parameter(sq_norm_term.shape), cp.Parameter(sp_plus_1_minus_so.shape), cp.Parameter(s_p_plus_1.shape), cp.Parameter(obs_motion.shape)

					so_param.value = obs_motion
					sq_norm_param.value = sq_norm_term
					sp_plus_1_minus_so_param.value = sp_plus_1_minus_so
					s_p_plus_1_param.value = s_p_plus_1

					# print("sp")
					# print(sq_norm_term)
					# print(sp_plus_1_minus_so_param.value)

					s_k_minus_so = cp.reshape(s_k - so_param,(nsteps,2)) 

					#LHS, gamma * ||s_{k+1} - s_obs||^2
					LHS = gamma * cp.sum(cp.square(s_k_minus_so), axis = 1, keepdims=False)

					#RHS, r^2 * (gamma - 1) + gamma * ||s_p_plus_1 - s_o||^2 - 2 * gamma * (sp - so).T * sp + 2 * gamma * (sp - so).T * sk
					RHS = r_obs[j] ** 2 * (gamma - 1) + sq_norm_param - 2 * sp_plus_1_minus_so_param @ s_p_plus_1_param + 2 * sp_plus_1_minus_so_param @ s_k_plus_1

					# print("LHS\n",LHS)
					# print("RHS\n",RHS)

					soc_constraints.append(LHS <= RHS + slack[j])
					# pdb.set_trace()
					# print("first sq_norm_term shape\t", sq_norm_param.value.shape)


			if(variables):
				pass

			else:
				prob = cp.Problem(prob.objective, prob.constraints + soc_constraints)

			# print("Solution:\t",state_cont_pair.value)
			# print("Status:\t", prob.status)
			tau_k = np.min([mu * tau_k, tau_max])
			tau.value = [tau_k]
			prob.solve(verbose=False)

			if(prob.status in [cp.OPTIMAL,cp.OPTIMAL_INACCURATE]):

				obs_free_traj = state_cont_pair.value

				x_prev = obs_free_traj[0:nsteps + 1]
				y_prev = obs_free_traj[y_offset:y_offset + nsteps + 1]

				plt.plot(x_prev + x_destination,y_prev + y_destination,'bo-')


				# x_prev, y_prev = project_to_obstacles(x_prev, y_prev, x_obs, y_obs, r_obs, nsteps)

				plt.plot(x_prev + x_destination,y_prev+y_destination,'rx-')
				# obs_free_traj = np.concatenate((obs_free_traj_x,obs_free_traj_y))


				# print(obs_free_traj)

			else:
				print(prob.status)
				# pdb.set_trace()

			t3_stop = perf_counter()
			current_time =  t3_stop - t3_start
			print("SOC MPC instant time\t",t3_stop - t3_start)

			print("Cost value \t",prob.objective.value)


			if(counter > 100000):
			    #print("Counters cleared")
			    total_time = 0.
			    counter = 0

			total_time += current_time
			counter = counter + 1
			avg_time = total_time / counter

			sum_slack = cp.sum(slack).value
			print("Sum slack\t",sum_slack)
			print("tau\t", tau_k)


			if(iterations != 1):
				if(prev_obj_value - prob.objective.value < 1e-4) and cp.sum(slack).value <= delta_violation:
					print("Diff\t",prob.objective.value - prev_obj_value )
					# print("Cost converged")
					# pdb.set_trace()
					break

			prev_obj_value = prob.objective.value

	print("SOC MPC average time\t",avg_time)
	t1_stop = perf_counter()
	print("SOC MPC total time\t", t1_stop - t1_start)

	if __name__ == "__main__":
		# x_prev = obs_traj[0:nsteps + 1] + x_destination
		# y_prev = obs_traj[y_offset:y_offset + nsteps + 1] + y_destination

		plt.xlim(-4,4)
		# plt.ylim(-15,5)
		plt.plot(x_prev_free, y_prev_free, '-o', label='Initial MPC')
		plt.plot(x_prev, y_prev,'-x', label='SOC MPC')
		plt.gca().set_aspect('equal')
		theta = np.linspace(0, 2 * 3.14159, 100)

		circle_x = x_obs[len(x_obs)-1] + r_obs[len(x_obs)-1] * np.cos(theta) + x_destination
		circle_y = y_obs[len(x_obs)-1] + r_obs[len(x_obs)-1] * np.sin(theta) + y_destination

		# circle_x_2 = x_obs[1] + r_obs[1] * np.cos(theta)
		# circle_y_2 = y_obs[1] + r_obs[1] * np.sin(theta)

		plt.plot(circle_x, circle_y)
		# plt.plot(circle_y_2,circle_x_2)
		final_circle_x = x_obs[len(x_obs)-1] + vx_obs[len(x_obs)-1] * nsteps * interval + r_obs[0] * np.cos(theta) + x_destination
		final_circle_y = y_obs[len(x_obs)-1] + vy_obs[len(x_obs)-1] * nsteps * interval + r_obs[0] * np.sin(theta) + y_destination

		plt.plot(final_circle_x, final_circle_y)

		# plt.legend()
		plt.show()

	# if (nsteps != prev_nsteps or interval != prev_interval):
	variables = {"big_A_eq": big_A_eq, "big_A_ineq": big_A_ineq, "big_H": big_H, "big_h": big_h, "prev_nsteps": prev_nsteps, "prev_interval": prev_interval, "solution": obs_traj, "prob": prob, "obs_free_sol": u_in}

	prev_nsteps = nsteps
	prev_interval = interval

	# else:
	# 	variables = {"solution": obs_traj}

	return obs_traj[nsteps+1], obs_traj[3 * nsteps+2], variables

if __name__ == "__main__":
	np.set_printoptions(precision=3, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	#Some calls for standalone testing of solver, current pose is the pose of robot wrt to destination, and final_pose is wrt global frame. 
	
	update_var = {}
	update_var_2 = {}
	gamma = .2
	# obs = [[2.5,5],[0.1,-0.1],[1,1],[0,0],[0,0]]
	obs = [[0],[4],[0.5],[0],[0]]
	init_soln = None
	final_pos = [-0.5,6,0]

	all_mpc_time_start = perf_counter()
	# Obstacles array format = [x_obs, y_obs, r_obs, vx_obs, vy_obs]
	# _, _, update_var_2 = MPC_obstacle_avoidance.MPC_solver(init_pose=[0,0,0],current_pose=[0,0,0],final_pose=final_pos, x_limit = 100000, y_limit = 100000, nsteps=15, interval = 0.1, obstacles = obs, gamma=gamma, r_vehicle = 0.5, variables=update_var)
	# init_soln = update_var_2.get("solution")

	lin_u, ang_u, update_var = MPC_solver(init_pose=[0,0,0],current_pose=[0,0,0],final_pose=final_pos, x_limit = 100000, y_limit = 100000, nsteps=15, interval = 0.1,variables=update_var, obstacles = obs, gamma=gamma, feasible_sol=init_soln, r_vehicle = 0.5)
	all_mpc_time_stop = perf_counter()

	print("Total time\t", all_mpc_time_stop - all_mpc_time_start)
	# for i in range(0,3):
	# 	lin_u, ang_u, update_var = MPC_solver(init_pose=[0,0,0],current_pose=[0,0,0],final_pose=[10,0,0], x_limit = 100000, y_limit = 100000, nsteps=15, interval = 0.1,variables=update_var, obstacles = [[5],[-1+i],[3],[1],[-1]])
