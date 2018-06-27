#!/usr/bin/python
import quadprog
import numpy
from numpy import array
import numpy as np
from scipy.linalg import block_diag
import qp_matrix

def MPC_solver(actual=0., desired=0.,nsteps=10.,interval=0.1):
	big_H = np.eye(2*nsteps)			#G
	big_h = np.zeros(2*nsteps)		
	
	big_Aeq, big_beq = MPC_dynamic_constraints(actual, desired,nsteps,interval)
	dyn_A, dyn_b = MPC_terminal_constraints(nsteps, actual, desired, big_Aeq,big_beq)
	
	big_A = np.row_stack((big_Aeq, dyn_A))
	big_b = np.concatenate((big_beq, dyn_b))

	u_in = qp_matrix.quadprog_solve_qp(big_H, big_h, 0*big_A, 0*big_b, C=big_A, d=big_b)
	print(u_in[nsteps])
	return u_in[nsteps]

def MPC_dynamic_constraints(actual=0., desired=0.,nsteps=10.,interval=0.1):
	global big_A, big_b
	big_H = np.eye(2*nsteps)			#G
	big_h = np.zeros(2*nsteps)		
	# print(np.shape(big_H))
	# print(np.shape(big_h))
	# big_A = np.empty((21, 10), dtype=float)
	big_A = np.column_stack((np.eye(nsteps , dtype=float) + np.eye(nsteps, nsteps , 1, dtype=float) * -1, np.eye(nsteps, dtype=float) * interval)) 	#C
	big_b = np.zeros(10, dtype=float)
	#print(np.shape(big_A))
	#print(np.shape(big_b))
	#print(u_in[nsteps])
	# for i in range(0, 9, step=1):
	return big_A,big_b


def MPC_terminal_constraints(nsteps, actual, desired, A_eq, b_eq):
	global big_A, big_b

	dyn_A = np.zeros((1,2*nsteps), dtype=float)
	dyn_A[0][0] = 1
	#lhs_matrix[1][10] = 1
	# lhs_matrix = np.eye(2, dtype=float)
	dyn_b = np.array([actual-desired])
	
	# big_A = block_diag(big_A, lhs_matrix)

	return dyn_A, dyn_b
	

if __name__ == "__main__":
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=200, suppress=None, nanstr=None, infstr=None, formatter=None)
	MPC_solver(1,5,10,0.1)