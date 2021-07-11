#!/usr/bin/python3
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt


def process_obstacle_list(obstacle_list):
    obstacle_x = obstacle_list[0]
    obstacle_y = obstacle_list[1]
    obstacle_r = obstacle_list[2]
    obstacle_v_x = obstacle_list[3]
    obstacle_v_y = obstacle_list[4]
    return [[np.array((obstacle_x[index], obstacle_y[index])),
             np.array((obstacle_v_x[index], obstacle_v_y[index])),
             obstacle_r[index]] for index in range(len(obstacle_x))]


def MPC_solver(init_pose, final_pose, n_steps=10, gamma=0.2, obstacles=None,
        cvxpy_args={'solver': 'ECOS'}): #, randomize_initial_trajectory=True):
    obstacles = process_obstacle_list(obstacles)
    # Dynamics --  x_{t+1} = x_t + u_t * sampling_time
    SAMPLING_TIME = 0.1
    point_mass_A = np.eye(2)
    point_mass_B = np.eye(2) * SAMPLING_TIME

    # Limits
    MIN_VELOCITY = -1
    MAX_VELOCITY = 1
    MIN_X = -10
    MIN_Y = -10
    MAX_X = 10
    MAX_Y = 10
    TARGET = np.array(final_pose[:2])
    CURRENT_STATE = np.array(init_pose[:2])
    PENALTY_ON_TERMINAL_STATE = 1000

    states_colwise = cp.Variable((2, n_steps + 1), name='state_vector')
    inputs_colwise = cp.Variable((2, n_steps), name='input_vector')
    dynamics_const = [states_colwise[:, t+1] \
                      == point_mass_A @ states_colwise[:, t]
                            + point_mass_B @ inputs_colwise[:, t]
                      for t in range(0, n_steps)]
    input_const = [inputs_colwise >= MIN_VELOCITY,
                   inputs_colwise <= MAX_VELOCITY]
    state_const = [states_colwise[0, :] >= MIN_X,
                   states_colwise[0, :] <= MAX_X,
                   states_colwise[1, :] >= MIN_Y,
                   states_colwise[1, :] <= MAX_Y]
    const = [states_colwise[:, 0] == CURRENT_STATE] + dynamics_const \
             + input_const + state_const

    tracking_error = states_colwise - np.tile(TARGET, (n_steps + 1, 1)).T
    objective = cp.Constant(0)
    q = np.ones((n_steps +1,))
    q[-1] = PENALTY_ON_TERMINAL_STATE
    for time_index in range(n_steps + 1):
        objective += q[time_index] \
                     * cp.sum_squares(tracking_error[:, time_index])
    objective += cp.sum_squares(inputs_colwise)

    # Initial seed
    obstacle_ignoring_prob = cp.Problem(cp.Minimize(objective), const)
    obstacle_ignoring_prob.solve(**cvxpy_args)

    if obstacle_ignoring_prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        # Update the trajectory
        states_colwise_prev = states_colwise.value
        # if randomize_initial_trajectory:
        #     states_colwise_prev += np.random.uniform(low=-1, high=1,
        #         size=(2, n_steps + 1))
        plt.plot(states_colwise_prev[0, :], states_colwise_prev[1, :], 'rx-')
        plt.draw()
        plt.pause(1)
    else:
        raise RuntimeError("CVPXY didn't solve the obstacle ignoring "
                           "trajectory. Terminated with status: "
                           f"{obstacle_ignoring_prob.status:s}")

    # SUCCESSIVE CONVEXIFICATION --- PREPARATION
    tau = cp.Parameter((1,), name='tau')
    soc_constraints = []
    slack = cp.Variable((len(obstacles), n_steps), nonneg=True)
    obs_motion_list = [cp.Parameter((2, n_steps))
                       for _ in range(len(obstacles))]
    RHS_linear_term_m_list = [cp.Parameter((n_steps, 2))
                              for _ in range(len(obstacles))]
    RHS_linear_term_c_list = [cp.Parameter((n_steps,))
                              for _ in range(len(obstacles))]

    if obstacles is not None:
        for obstacle_index in range(len(obstacles)):
            # RHS_linear_term_m_list from k=1 to N
            RHS = cp.hstack((
                RHS_linear_term_m_list[obstacle_index][k] \
                        @ states_colwise[:, k + 1] \
                  + RHS_linear_term_c_list[obstacle_index][k]
                for k in range(n_steps)))
            # Obs_motion is the obstacle trajectory from k=1 to N
            # Here, obs_motion_list k=0 actually means k=1
            LHS = gamma * cp.sum(cp.square(
                states_colwise[:, :-1] - obs_motion_list[obstacle_index]),
                axis=0, keepdims=False)
            # Enforcing Barrier function for every k, given i and i+1
            soc_constraints.append(LHS <= RHS + slack[obstacle_index])

    obstacle_considering_const = const + soc_constraints
    obstacle_considering_objective = objective + tau * cp.sum(slack)
    obstacle_considering_prob \
        = cp.Problem(cp.Minimize(obstacle_considering_objective),
            obstacle_considering_const)

    tau_k = 1
    TAU_MAX = 1e5
    MU = 3
    ITERATIONS_MAX = 1000
    DELTA_VIOLATION = 1e-5

    # SUCCESSIVE CONVEXIFICATION --- EXECUTION
    not_converged = True
    prev_obj_value = np.inf
    iteration_counter = 0
    while not_converged:
        if obstacles is not None:
            for obstacle_index in range(len(obstacles)):
                specific_obstacle = obstacles[obstacle_index]
                # obstacles = [ obstacle_location: array-like, obstacle_velocity:
                # array-like, radius]
                # 1. Compute x_obs, y_obs for t=1 to n_steps (ignore initial state)
                obs_motion_value = np.zeros((2, n_steps))
                current_obs_motion_state = specific_obstacle[0]
                obs_velocity = specific_obstacle[1]
                obs_radius = specific_obstacle[2]
                for t_minus_1 in range(n_steps):
                    obs_motion_value[:, t_minus_1] \
                        = point_mass_A @ current_obs_motion_state \
                          + point_mass_B @ obs_velocity
                    current_obs_motion_state = obs_motion_value[:, t_minus_1]
                # 2. Compute RHS_linear_term_m value 2 * (x_{k+1}^i - c_k)
                # Terms are arranged row-wise for each k
                RHS_linear_term_m_value \
                    = (2 * (states_colwise_prev[:, 1:] - obs_motion_value)).T
                # 3. Compute RHS_linear_term_c value
                # \|x_{k+i}^i - c_k\|^2 -  2 * (x_{k+1}^i - c_k) @ x_{k+1}^i +
                # r^2 (\gamma - 1)
                RHS_linear_term_c_second_term = np.zeros((n_steps,))
                for k in range(n_steps):
                    # 2 * (x_{k+1} ^ i - c_k) @ x_{k + 1} ^ i
                    # 1 means zero
                    RHS_linear_term_c_second_term[k] \
                        = - RHS_linear_term_m_value[k, :] \
                          @ states_colwise_prev[:, k + 1]
                RHS_linear_term_c_value \
                    = cp.sum(cp.square(states_colwise_prev[:, 1:] -
                                     obs_motion_value), axis=0).value \
                      + RHS_linear_term_c_second_term\
                      + (obs_radius ** 2) * (gamma - 1)

                # Get the relevant parameters
                RHS_linear_term_m_list[obstacle_index].value \
                    = RHS_linear_term_m_value
                RHS_linear_term_c_list[obstacle_index].value \
                    = RHS_linear_term_c_value
                obs_motion_list[obstacle_index].value = obs_motion_value

        # Update tau and tau_k
        tau.value = [tau_k]

        # Solve the problem
        obstacle_considering_prob.solve(**cvxpy_args)
        if obstacle_considering_prob.status in [cp.OPTIMAL,
                                                cp.OPTIMAL_INACCURATE]:
            # Update the trajectory
            states_colwise_prev = states_colwise.value
            plt.plot(states_colwise_prev[0, :], states_colwise_prev[1, :],
                'b*-')
            plt.draw()
            plt.pause(1)
        else:
            raise RuntimeError("CVPXY didn't solve in the successive "
                               f"convexification. Terminated with status: "
                               f"{obstacle_considering_prob.status:s}")

        # Update not_converged
        cost_difference = prev_obj_value \
                          - obstacle_considering_prob.objective.value
        sum_slack = cp.sum(slack).value
        print(f'\n\nIteration: {iteration_counter:d}')
        print(f'Tau {tau_k:1.2f}')
        print(f'Sum slack {sum_slack:1.2f}')
        print("Diff\t", cost_difference)
        if np.abs(cost_difference) < 1e-4 and sum_slack <= DELTA_VIOLATION:
            not_converged = False

        # Prepare for the next iteration
        prev_obj_value = obstacle_considering_prob.objective.value
        tau_k = np.min([MU * tau_k, TAU_MAX])
        iteration_counter += 1

        if iteration_counter > ITERATIONS_MAX:
            print('Breaking because iteration limit reached')
            break

    return inputs_colwise.value[0, :], inputs_colwise.value[1, :], \
           states_colwise_prev[0, :], states_colwise_prev[1, :],


if __name__ == "__main__":
    plt.figure()
    plt.gca().set_aspect('equal')
    final_pose = [0.1, 6, 0]
    plt.scatter(final_pose[0], final_pose[1], 30, color='g', marker='o')
    theta = np.linspace(0, 2 * np.pi, 100)
    obstacle_list = [[0], [4], [1], [0], [0]]
    # obstacle = [np.array([0, 4]), np.array([0, 0]), 1]
    circle_x = obstacle_list[0] + obstacle_list[2] * np.cos(theta)
    circle_y = obstacle_list[1] + obstacle_list[2] * np.sin(theta)
    plt.plot(circle_x, circle_y)
    plt.xlim([-5, 5])
    plt.ylim([-1, 8])
    plt.draw()
    plt.pause(0.1)
    _, _, x_prev, y_prev = MPC_solver([0, 0, 0], final_pose,
        n_steps=60, gamma=0.2, obstacles=obstacle_list,
        cvxpy_args={'solver':'ECOS'})
    plt.plot(x_prev, y_prev, 'b+-')
    plt.show()
