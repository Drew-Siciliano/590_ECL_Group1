function [wr, t2, t3, disable] = formation_control(wr, t2, t3, time, disable)
%% Consensus formation controller. Computes Xdot Ydot for the three robots
% Inputs and Outputs:
%   wr - robot 1
%   t2 - robot 2
%   t3 - robot 3
%   time - sim time step
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
    
    %% Formation Control Law
    % robot locations 3x2 X, Y
    x = [];

    % desired formation locations 3x2 X, Y
    xi = [];
    % define tau to be displacement of robot position from target location
    tau = x - xi;
    tauDot = - Lg * tau; % 3x2 matrix of Xdot, Ydot for 3 robots

    %% Convert Xdot Ydot to speed and headings
    speeds = [norm(tauDot(1, :)); norm(tauDot(2, :)); norm(tauDot(3, :))];
    headings = [atan2(tauDot(1, 2), tauDot(1, 1)); ...
                atan2(tauDot(2, 2), tauDot(2, 1)); ...
                atan2(tauDot(3, 2), tauDot(3, 1))];

    %% Paste speed and heading code here

    %% Disable
    if norm(tau) < 5
        disable = 1;
    else
        disable = 0;
    end
end