function [X_subopt, X_star] = compute_suboptimal_trajectories(x0, xf, A, B, T, N, gamma, C)

% This code is used to compute an arbitrary number N of suboptimal 
% trajectories from the minimum-energy trajectory. This is done by adding 
% a control action drawn from the kernel of the controllability matrix 
% (thus orthogonal to the minimum-energy control u_star) to the full
% control input - T. Menara, F. Pasqualetti 2019
%
% INPUTS:
% x0        initial state
% xf        final state
% A         stable structural connectivity matrix
% B         control matrix
% T         time horizon
% N         number of suboptimal trajectories computed
% gamma 	parameter controlling suboptimality of trajectories. Must be 
%           greater than 1. The larger this parameter, the
%           more energy is allowed for suboptimal trajectories
% C         controllability matrix
%
% OUTPUTS:
% X_subopt  suboptimal control trajectories
% X_star    optimal (mininum-energy) control trajectory

if nargin < 8
    C = B; % initialize controllability matrix at first time step
    for t = 1 : T-1
        C = [A*C B]; % controllability matrix
    end
end

if nargin < 7
    gamma = 1.01; % parameter > 1 used to scale the minimum energy in order to obtain the suboptimal trajectories
end

K = null(C); % kernel of the controllability matrix (it is orthogonal to u_star)

U_star = pinv(C)*(xf - A^T*x0); % minimum-energy control

% compute optimal trajectory
X_star = x0;
for t = 1:T
    X_star(:,t+1) = A*X_star(:,t) + B*U_star((t-1)*size(B,2)+1:t*size(B,2));
end

E_min = norm(U_star); % compute the minimum energy

for i = 1:N
    U_bar = K*randn(size(K,2),1); % initialize suboptimal control addition in the kernel of C
    U_bar = sqrt((gamma-1)*E_min)*U_bar/norm(U_bar);
    X = x0;
    for t = 1:T-1
        X(:,t+1) = A*X(:,t) + B*(U_star((t-1)*size(B,2)+1:t*size(B,2)) + U_bar((t-1)*size(B,2)+1:t*size(B,2))); % the suboptimal trajectory
    end
    X_subopt{i,1} = X; % save each suboptimal trajectory in a cell
end

