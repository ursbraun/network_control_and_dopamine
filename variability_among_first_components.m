function [Relative_Deviation] = variability_among_first_components(T, N, X_star, X_subopt)

% This code computes the variability between the first principal components 
% extracted via principal component analysis (PCA) of suboptimal trajectories.
% This is done in two steps. First, we compute the PCA of all suboptimal
% trajectories' points at each time step. Second, we compute the longest 
% distance of the suboptimal points projected onto the first two principal 
% components from the minimum-energy trajectory.
% This distance is used as a measure of variability of
% the suboptimal trajectories with respect to the
% minimum-energy trajectory at each time step. The output of the code is 
% the percentage of deviation from the optimal trajectory - T. Menara 2019
%
%
% INPUTS:
% X_Star                optimal (minimum-energy) trajectory, e.g. output of compute_suboptimal_trajectories function 
% X_subopt              suboptimal trajectory, e.g. output of compute_suboptimal_trajectories function              
% T                     time horizon
% N                     number of suboptimal trajectories computed
%
% OUTPUTS:
% Relative Deviation    Percentage of deviation from optimal trajectory


for t = 1 : T
    M = [];
    for r = 1 : N
        aux = X_subopt{r};
        aux = aux(:,t);
        M = [M aux]; % concatenate horizontally all suboptimal points at time t
    end
    
    warning off % remove Matlab warnings
    [coeff_t{t},~,~,~,explained] = pca(M'); % pca at time step t
    warning on
    v1 = coeff_t{t}(:,1); % vector from X_star(t) representing PC1
    v2 = coeff_t{t}(:,2); % vector from X_star(t) representing PC2
    for j = 1:N
        p1(:,j) = (v1*v1'/(v1'*v1))*(M(:,j)-X_star(:,t))+X_star(:,t); % projection onto v1 of the suboptimal point
        d1(j,1) = norm(p1(:,j)-X_star(:,t)); % compute distance from x_star of the projection along v1
        p2(:,j) = (v2*v2'/(v2'*v2))*(M(:,j)-X_star(:,t))+X_star(:,t); % projection onto v2 of the suboptimal point
        d2(j,1) = norm(p2(:,j)-X_star(:,t)); % compute distance from x_star of the projection along v2
    end
    [max_dist_PC1(t) idx1(t)] = max(d1); % maximum distance of the components along PC1 from x_star(t)
    [max_dist_PC2(t) idx2(t)] = max(d2);
    
    %Length of X_star
    Length_X_star = [];
    for r = 1 : size(X_star,2)
        Length_X_star(1,r) = norm(X_star(:,r)); % lenght of optimal trajectory
    end
    
    if t > 1 || norm(X_star(:,t))~=0
        Relative_Deviation(1,t) = max_dist_PC1(t)./norm(X_star(:,t))*100; % Percentage of deviation from optimal trajectory
    else
        Relative_Deviation(1,t) = 0;
    end

end
