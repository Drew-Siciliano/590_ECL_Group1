function [wr, t2, t3, disable] = formation_control(wr, t2, t3, disable)
%% Consensus formation controller. Computes Xdot Ydot for the three robots
% Inputs and Outputs:
%   wr - robot 1
%   t2 - robot 2
%   t3 - robot 3
%   disable - boolean for when to stop when formation reaches desired
%             formation

    %% Graph definition

    % Adjacency matrix: all 3 nodes connected to each other
    A = [0 1 1;
         1 0 1;
         1 1 0];

    % Degree matrix
    Dg = 2 * eye(3);

    % graph Laplacian
    Lg = Dg - A;

end