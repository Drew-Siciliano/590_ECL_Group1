function [c_inequality, c_equality] = collocationConstraints(Z, vehicle_params, opt_params, constraint_params)
    N_nodes = opt_params.N_nodes; % Number of Nodes
    
    nx = vehicle_params.nx; % Length of State
    nu = vehicle_params.nu; % Length of Control
    L = vehicle_params.L; % Length between two wheels

    X0 = constraint_params.X0(:);
    Xf = constraint_params.Xf(:);

    r_ball = constraint_params.r_ball; % Radius of Ball
    pos_ball = constraint_params.pos_ball(:); % Position of Ball

    % Extrac State Inputs
    Tf = Z(1);
    Z_state_control = Z(2:end);
    Z_state_control = reshape(Z_state_control, nx+nu, N_nodes);

    x = Z_state_control(1,:);
    y = Z_state_control(2,:);
    theta = Z_state_control(3,:);

    vr = Z_state_control(4,:);
    vl = Z_state_control(5,:);

    dt = Tf/(N_nodes-1);

    %% Dynamic Constraints
    c_dyn = []; 
    f = @(k) [(vr(k) + vl(k))/2 * cos(theta(k)); ...
             (vr(k) + vl(k))/2 * sin(theta(k)); ...
             (vr(k) - vl(k))/L ...
             ];
    for k = 1:N_nodes-1
        fk = f(k);
        fkp1 = f(k+1);
        
        Xk = [x(k);y(k);theta(k)];
        Xkp1 = [x(k+1);y(k+1);theta(k+1)];

        ck_dyn = (Xkp1 - Xk) - 0.5*dt*(fk + fkp1);
        c_dyn = [c_dyn; ck_dyn];
    end

    %% Boundary Conditions
    c_BC = [x(1);y(1);x(end);y(end)] - [X0;Xf];

    %% Ball Constraint
    c_ball = [];
    for k = 1:N_nodes
        Xk = [x(k);y(k)];
        dist2ball = Xk - pos_ball;

        c_ball = [c_ball; r_ball^2 - dist2ball.'*dist2ball]; % r_ball^2 < dist2ball^2 (removes norm gradient issue)
    end

    %% Outputs
    c_equality = [c_dyn;c_BC];
    c_inequality = c_ball;
end