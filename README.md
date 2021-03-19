# network_control_and_dopamine
This repository contains code associated with the paper Braun et al, 2021: Brain network dynamics during working memory are modulated by dopamine and diminished in schizophrenia 

README

System Requirements:
MATLAB 2015a or newer
Control Systems Toolbox

Installation Guide:
Add m-files to path

Description:

optim_fun.m:	 computes optimal control inputs and trajectories

Inputs: 	A     	(NxN) stable structural connectivity matrix, e.g. stabilised 					A = (A / (| λ(A)max | + 1)) - 1.
B     	(NxN) Input matrix: selects which nodes to inject input into. Define S so there is a 1 on the diagonal of elements you want to add input to, and 0 otherwise. 
S     	(NxN) Selects nodes whose distance from the final target state you wish to penalize in the cost functional. Define so that there is a 1 on the diagonal of elements you want to constrain, and a zero otherwise
T     	Time horizon: how long you allow for the state trajectory to reach the target state as a result of the input injected into the control nodes. Small values of T will make the system difficult to control, whereas large values of T will make the system easy to control. In theory, T is dimensionless if not coupled to external time domains; therefore T = 1 can be used to have a normalized time.
rho   	User-defined parameter to penalize the energy used by the optimal input relative to the trajectories. Small rho allows larger control energy.

compute_suboptimal_trajectories.m:	computes a number N of suboptimal trajectories that slightly deviate from the minimum-energy trajectory. This is done by adding a control action drawn from the kernel of the controllability matrix 
 to the optimal control input.

 Inputs:		x0 	initial state
 xf       final state
 A       stable structural connectivity matrix
 B       	control matrix
 T       	time horizon
 N       	number of suboptimal trajectories computed
 gamma 	parameter controlling suboptimality of trajectories. Must be greater than 1. The larger this parameter, the more energy is allowed for suboptimal trajectories
 C       	controllability matrix (optional)



Example:
Load data:
load('example_data.mat');

Stabilize structural connectivity matrix A:
A_star=A./(eigs(A,1)+1)-eye(size(A,1));

Set parameters for optimal control:
T=10;
rho=1;

Run control code
[X_opt, U_opt, n_err] = optim_fun(A, T, B, x0, xf, rho);

Compute control energy:
CE = trapz(U_opt.^2);

Convert A to discrete:
To compute the suboptimal trajectories, we first need to convert A from continuous to discrete dynamical system:

[sysd, sysd_tustin, Ts] = continuous_to_discrete (A, B);

Compute suboptimal trajectories
gamma = 1.01;
T = 10;
N = 100;
[X_subopt, X_star] = compute_suboptimal_trajectories(x0, xf, sysd.A, sysd.B, T, N, gamma);
[Relative_Deviation] = variability_among_first_components(T, N, X_star, X_subopt);

Note that the provided output example for Relative_Deviation should be different for each run, because of the added random component to the optimal trajectories.
