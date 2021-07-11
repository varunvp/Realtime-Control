#!/usr/bin/python3
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt


class MPC_problem(object):
    def __init__(self, n_steps=10, gamma=0.2, obstacles=None,
            cvxpy_args={'solver': 'ECOS'}):
        # Dynamics --  x_{t+1} = x_t + u_t * sampling_time
        self.SAMPLING_TIME = 0.5
        self.point_mass_A = np.eye(2)
        self.point_mass_B = np.eye(2) * self.SAMPLING_TIME
        self.n_steps = n_steps

        # Limits
        MIN_VELOCITY = -0.5
        MAX_VELOCITY = 0.5
        MIN_X = -10
        MIN_Y = -10
        MAX_X = 10
        MAX_Y = 10
        PENALTY_ON_TERMINAL_STATE = 1000

        self.states_colwise = cp.Variable((2, self.n_steps + 1))
        self.inputs_colwise = cp.Variable((2, self.n_steps))
        self.current_state = cp.Parameter((2,))
        self.target = cp.Parameter((2,))

        # Constraint definition --- obstacle ignoring
        dynamics_const = [self.states_colwise[:, t+1] \
                          == self.point_mass_A @ self.states_colwise[:, t]
                                + self.point_mass_B @ self.inputs_colwise[:, t]
                          for t in range(0, n_steps)]
        input_const = [self.inputs_colwise >= MIN_VELOCITY,
                       self.inputs_colwise <= MAX_VELOCITY]
        state_const = [self.states_colwise[0, :] >= MIN_X,
                       self.states_colwise[0, :] <= MAX_X,
                       self.states_colwise[1, :] >= MIN_Y,
                       self.states_colwise[1, :] <= MAX_Y]
        const = [self.states_colwise[:, 0] == self.current_state] \
                + dynamics_const + input_const + state_const

        # Objective definition --- obstacle ignoring
        target_matrix = cp.reshape(self.target, (2, 1))
        tracking_error = self.states_colwise[:, 1:] \
                         - cp.kron(np.ones((1, n_steps)), target_matrix)
        objective = cp.Constant(0)
        q = np.ones((self.n_steps,))
        q[-1] = PENALTY_ON_TERMINAL_STATE
        for time_index in range(self.n_steps):
            objective += q[time_index] \
                         * cp.sum_squares(tracking_error[:, time_index])
        objective += cp.sum_squares(self.inputs_colwise)

        # Initial seed
        self.obstacle_ignoring_prob = cp.Problem(cp.Minimize(objective), const)
        self.initial_seed_trajectory = None
        self.cvxpy_args = cvxpy_args

        # SUCCESSIVE CONVEXIFICATION --- PREPARATION
        if obstacles is not None:
            self.gamma = gamma
            self.obstacles = self.process_obstacle_list(obstacles)
            soc_constraints = []
            self.tau = cp.Parameter((1,), name='tau')
            self.slack = cp.Variable((len(self.obstacles), self.n_steps),
                nonneg=True)
            self.obs_motion_list = [cp.Parameter((2, self.n_steps))
                                    for _ in range(len(self.obstacles))]
            self.RHS_linear_term_m_list = [cp.Parameter((self.n_steps, 2))
                                           for _ in range(len(self.obstacles))]
            self.RHS_linear_term_c_list = [cp.Parameter((self.n_steps,))
                                           for _ in range(len(self.obstacles))]
            for obstacle_index in range(len(self.obstacles)):
                # RHS_linear_term_m_list from k=1 to N
                RHS = cp.hstack((
                    self.RHS_linear_term_m_list[obstacle_index][k] \
                    @ self.states_colwise[:, k + 1] \
                    + self.RHS_linear_term_c_list[obstacle_index][k]
                    for k in range(self.n_steps)))
                # Obs_motion is the obstacle trajectory from k=1 to N
                # Here, obs_motion_list k=0 actually means k=1
                LHS = self.gamma * cp.sum(cp.square(
                    self.states_colwise[:, :-1]
                    - self.obs_motion_list[obstacle_index]), axis=0,
                    keepdims=False)
                # Enforcing Barrier function for every k, given i and i+1
                soc_constraints.append(LHS <= RHS + self.slack[obstacle_index])

            obstacle_considering_const = const + soc_constraints
            obstacle_considering_objective = objective \
                                             + self.tau * cp.sum(self.slack)
            self.obstacle_considering_prob \
                = cp.Problem(cp.Minimize(obstacle_considering_objective),
                    obstacle_considering_const)
        else:
            self.obstacles = None

    @staticmethod
    def process_obstacle_list(obstacle_list):
        if obstacle_list is not None:
            obstacle_x = obstacle_list[0]
            obstacle_y = obstacle_list[1]
            obstacle_r = obstacle_list[2]
            obstacle_v_x = obstacle_list[3]
            obstacle_v_y = obstacle_list[4]
            return [[np.array((obstacle_x[index], obstacle_y[index])),
                     np.array((obstacle_v_x[index], obstacle_v_y[index])),
                     obstacle_r[index]] for index in range(len(obstacle_x))]
        else:
            return None

    def solve_obstacle_ignoring_problem(self, init_pose, final_pose):
        self.current_state.value = np.array(init_pose[:2])
        self.target.value = np.array(final_pose[:2])
        self.obstacle_ignoring_prob.solve(**self.cvxpy_args)

        if self.obstacle_ignoring_prob.status in [cp.OPTIMAL,
                                                  cp.OPTIMAL_INACCURATE]:
            # Update the trajectory
            initial_seed_trajectory = self.states_colwise.value
            # plt.plot(initial_seed_trajectory[0, :],
            #     initial_seed_trajectory[1, :], 'rx-')
            # plt.draw()
            # plt.pause(1)
            return initial_seed_trajectory
        else:
            raise RuntimeError("CVPXY didn't solve the obstacle ignoring "
                               "trajectory. Terminated with status: "
                               f"{self.obstacle_ignoring_prob.status:s}")

    def solve_obstacle_considering_problem(self, init_pose,
            final_pose):
        self.solve_obstacle_ignoring_problem(init_pose, final_pose)
        if self.obstacles is None:
            return self.inputs_colwise.value[0, 0], \
                   self.inputs_colwise.value[1, 0], \
                   self.states_colwise.value[0, :], \
                   self.states_colwise.value[1, :]
        else:
            states_colwise_prev = self.states_colwise.value

        tau_k = 1
        TAU_MAX = 1e5
        MU = 4
        ITERATIONS_MAX = 100
        DELTA_VIOLATION = 1e-3
        RELATIVE_CHANGE_IN_COST_TOLERANCE = 1e-1
        SUM_SLACK_TOL = 1e-5

        # SUCCESSIVE CONVEXIFICATION --- EXECUTION
        not_converged = True
        prev_obj_value = np.inf
        prev_sum_slack = np.inf
        iteration_counter = 0
        while not_converged:
            if self.obstacles is not None:
                for obstacle_index in range(len(self.obstacles)):
                    specific_obstacle = self.obstacles[obstacle_index]
                    # obstacles = [ obstacle_location: array-like, obstacle_velocity:
                    # array-like, radius]
                    # 1. Compute x_obs, y_obs for t=1 to n_steps (ignore initial state)
                    obs_motion_value = np.zeros((2, self.n_steps))
                    current_obs_motion_state = specific_obstacle[0]
                    obs_velocity = specific_obstacle[1]
                    obs_radius = specific_obstacle[2]
                    for t_minus_1 in range(self.n_steps):
                        obs_motion_value[:, t_minus_1] \
                            = self.point_mass_A @ current_obs_motion_state \
                              + self.point_mass_B @ obs_velocity
                        current_obs_motion_state = obs_motion_value[:, t_minus_1]
                    # 2. Compute RHS_linear_term_m value 2 * (x_{k+1}^i - c_k)
                    # Terms are arranged row-wise for each k
                    RHS_linear_term_m_value \
                        = (2 * (states_colwise_prev[:, 1:] - obs_motion_value)).T
                    # 3. Compute RHS_linear_term_c value
                    # \|x_{k+i}^i - c_k\|^2 -  2 * (x_{k+1}^i - c_k) @ x_{k+1}^i +
                    # r^2 (\gamma - 1)
                    RHS_linear_term_c_second_term = np.zeros((self.n_steps,))
                    for k in range(self.n_steps):
                        # 2 * (x_{k+1} ^ i - c_k) @ x_{k + 1} ^ i
                        # 1 means zero
                        RHS_linear_term_c_second_term[k] \
                            = - RHS_linear_term_m_value[k, :] \
                              @ states_colwise_prev[:, k + 1]
                    RHS_linear_term_c_value \
                        = cp.sum(cp.square(states_colwise_prev[:, 1:] -
                                         obs_motion_value), axis=0).value \
                          + RHS_linear_term_c_second_term\
                          + (obs_radius ** 2) * (self.gamma - 1)

                    # Get the relevant parameters
                    self.RHS_linear_term_m_list[obstacle_index].value \
                        = RHS_linear_term_m_value
                    self.RHS_linear_term_c_list[obstacle_index].value \
                        = RHS_linear_term_c_value
                    self.obs_motion_list[obstacle_index].value \
                        = obs_motion_value

            # Update tau and tau_k
            self.tau.value = [tau_k]

            # Solve the problem
            self.obstacle_considering_prob.solve(**self.cvxpy_args)
            if self.obstacle_considering_prob.status in [cp.OPTIMAL,
                                                    cp.OPTIMAL_INACCURATE]:
                # Update the trajectory
                states_colwise_prev = self.states_colwise.value
                # plt.plot(states_colwise_prev[0, :], states_colwise_prev[1, :],
                #     'b*-')
                # plt.draw()
                # plt.pause(1)
            else:
                raise RuntimeError("CVPXY didn't solve in the successive "
                                   f"convexification. Terminated with status: "
                                   f"{self.obstacle_considering_prob.status:s}")

            # Update not_converged
            cost_difference = prev_obj_value \
                              - self.obstacle_considering_prob.objective.value
            sum_slack = cp.sum(self.slack).value
            print(f'\n\nIteration: {iteration_counter:d}')
            print(f'Tau {tau_k:1.2f}')
            print(f'Sum slack {sum_slack:1.2f}')
            print("Diff\t", cost_difference)
            if np.abs(cost_difference) \
                    < RELATIVE_CHANGE_IN_COST_TOLERANCE * prev_obj_value \
                    and (sum_slack <= DELTA_VIOLATION \
                         or np.abs(sum_slack - prev_sum_slack) <=SUM_SLACK_TOL):
                not_converged = False

            # Prepare for the next iteration
            prev_obj_value = self.obstacle_considering_prob.objective.value
            prev_sum_slack = sum_slack
            tau_k = np.min([MU * tau_k, TAU_MAX])
            iteration_counter += 1

            if iteration_counter > ITERATIONS_MAX:
                print('Breaking because iteration limit reached')
                break

        return self.inputs_colwise.value[0, 0], \
               self.inputs_colwise.value[1, 0], \
               self.states_colwise.value[0, :], \
               self.states_colwise.value[1, :]


if __name__ == "__main__":
    plt.figure()
    plt.gca().set_aspect('equal')
    final_pose = [0.1, 6, 0]
    plt.scatter(final_pose[0], final_pose[1], 30, color='g', marker='o')
    theta = np.linspace(0, 2 * np.pi, 100)
    obstacle_list = [[0, 0, -2],
                     [4, 1, 2.5],
                     [1, 1, 1],
                     [0, 0, 0],
                     [0, 0, 0]]
    for obstacle_x, obstacle_y, obstacle_r in zip(obstacle_list[0],
            obstacle_list[1], obstacle_list[2]):
        circle_x = obstacle_x + obstacle_r * np.cos(theta)
        circle_y = obstacle_y + obstacle_r * np.sin(theta)
        plt.plot(circle_x, circle_y)
    plt.xlim([-5, 5])
    plt.ylim([-1, 8])
    plt.draw()
    plt.pause(0.1)
    MPC_prob_obj = MPC_problem(n_steps=60, gamma=0.2, obstacles=obstacle_list,
        cvxpy_args={'solver':'ECOS'})
    import time

    init_pose_list = [[0, 0, 0], [0.1, 0.2, 0], [-2.5, 2.5, 0], [-1.5, 3, 0],
                      [1.5, -1.5, 0], [2.5, -2.5, 0], [2.5, 2, 0],
                      [0.1, 0.2, 0], [-2.5, 2.5, 0], [0, 0, 0]]
    prob_index = [1, 2, 3, 4, 5, 6, 7, 1, 2, 3]
    n_init_pose = len(init_pose_list)
    duration = [0] * n_init_pose
    for index in range(n_init_pose):
        init_pose = init_pose_list[index]
        start_time = time.time()
        _, _, x_prev, y_prev = MPC_prob_obj.solve_obstacle_considering_problem(
            init_pose=init_pose, final_pose=final_pose)
        duration[index] = time.time() - start_time
        plt.plot(x_prev, y_prev, 'b+-')
    plt.draw()
    plt.pause(1)

    plt.figure()
    plt.stem(duration)
    plt.xticks(range(len(prob_index)), prob_index)
    plt.xlabel('Problem instance')
    plt.ylabel('Compute time (s)')
    plt.show()
