function [sysd, sysd_tustin, Ts] = continuous_to_discrete(A, B)

% This code is used to to discretize a multi-input multi-output continuous-
% time system. The sampling time is computed with a classical rule of thumb
% but can be further tuned by the user - T. Menara 2019
%
% INPUTS:
% A             structural connectivity matrix
% B             control matrix
% OUTPUTS:
% sysd          discretized dynamical system, i.e. sysd.A is the
%               discretized A matrix
% sysd_tustin   discretized and normalized dynamic system
% Ts            sampling time

N=size(A,1);

A = A-1.01*abs(max(eig(A)))*eye(N); % stabilize matrix A
% B = eye(N); % input matrix
C = eye(N); % output matrix
D = []; % no feedforward matrix

sys = ss(A,B,C,D); % state-space representation of the continuout-time system

S = stepinfo(sys); % compute step info (contains rise time) for all inputs and all outputs

for i = 1:size(S,1)
    for j = 1:size(S,2)
        RTime(i,j) = S(i,j).RiseTime;
    end
end

shortest_RTime = min(RTime(RTime~=0)); % choose fastest rise time (discard 0)

Ts = 1/10*shortest_RTime; % compute sampling time with classical rule of thumb

sysd = c2d(sys, Ts); % continuous to discrete
sysd_tustin = c2d(sys, Ts, 'tustin'); % discretization with Tustin method
