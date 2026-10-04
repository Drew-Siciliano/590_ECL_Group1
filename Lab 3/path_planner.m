function [wr,WP,old_ref] = path_planner(wr,r_target,obs_pos,old_ref, fig_increment)
    %% Calc waypoint distances 
    heading_vec = wr.heading_vec;
    pos = wr.pos(:);
    theta = atan2(heading_vec(2), heading_vec(1)); % radians

    %% Distance to Waypoint
    pos_curWP = r_target(:);
    vec_wr2WP = pos_curWP - pos;
    dist_curWP = norm(vec_wr2WP,2);

    if dist_curWP <= wr.dist_mar
        %% Reached Waypoint
        fprintf(" \n \n \n \n \nWayPoint "+string(wr.curWP)+" reached \n \n \n \n \n")
        wr.curWP = wr.curWP + 1;

        ul = 0;
        ur = 0;
        WP = 0;
        old_ref = [];
    elseif obs_pos(3) > 200
        %% 
        fprintf(" \n \n \n \n \n Put down the obsticale reached \n \n \n \n \n")
        ul = 0;
        ur = 0;
        WP = 0;
        old_ref = [];
    else

        fprintf(" \n \n \n \n \n Running MPC \n \n \n \n \n")

        options = optimoptions('fmincon', ...
            'Algorithm',              'sqp', ...
            'Display',                'off', ...
            'MaxIterations',          1000, ...
            'MaxFunctionEvaluations', 2e5, ...
            'ConstraintTolerance',    1e-6, ...
            'OptimalityTolerance',    1e-6);

        %% Collocation Setup
        % State is Z = [Tf; reshape([x; y; theta; vr; vl], [], 1)];

        N_nodes = 10;
        %N_nodes = 20; % for data
        nx = 3;
        nu = 2;

        opt_params.N_nodes = N_nodes; % Number of Nodes
        vehicle_params.nx = nx; % Length of State
        vehicle_params.nu = nu; % Length of Control
        vehicle_params.L = 151.2; % Length between two wheels (mm)

        constraint_params.X0 = [pos(:);theta];   % [x0; y0; theta0]
        constraint_params.r_target = pos_curWP(:);   % [xf; yf]

        constraint_params.r_ball = 0.3 * 1000 + vehicle_params.L; % Radius of Ball (mm) with some margin for tank width
        constraint_params.pos_ball = reshape(obs_pos(1:2),[],1); % Position of Ball

        u_max = 150;

        Z0_guess = generateReference(constraint_params.X0,constraint_params.r_target,N_nodes,u_max);

        if 1 
            %% debug plot
            Z_state_control_opt = reshape(Z0_guess(2:end),nx+nu,N_nodes);
            
            fig = figure(1001);
            clf(fig)

            xk = Z_state_control_opt(1,:);
            yk = Z_state_control_opt(2,:);

            plot(xk,yk,'k--*') % Optimized Path
            hold on
            plot(pos(1),pos(2),'g*') % Start
            hold on
            plot(pos_curWP(1),pos_curWP(2),'rx') % End
            hold on

            % Ball constraint
            theta_plot = linspace(0,2*pi,1000);
            plot(obs_pos(1)+constraint_params.r_ball*cos(theta_plot),obs_pos(2)+constraint_params.r_ball*sin(theta_plot),'r-')

            axis equal
        end
        
        %% Constraints on Opt Variables
        Tf_min = Z0_guess(1)*0.9;
        Tf_max = inf;

        LB_nodes = -inf(nx+nu,N_nodes);
        UB_nodes =  inf(nx+nu,N_nodes);
        control_rows = nx + (1:nu);     % Rows 4 and 5

        % Control Constraints
        LB_nodes(control_rows,:) = -u_max;
        UB_nodes(control_rows,:) =  u_max;

        lb = [Tf_min;LB_nodes(:)];
        ub = [Tf_max;UB_nodes(:)];

        %% Minimum-Time Objective
        objective = @(Z) Z(1);

        %% Nonlinear Constraints
        nonlcon = @(Z) collocationConstraints(Z, vehicle_params, opt_params, constraint_params);

        %% Solve
        [Z_opt,~,exitflag,~] = fmincon( ...
                                objective, ...
                                Z0_guess, ...
                                [],[], ... % A, b
                                [],[], ... % Aeq, beq
                                lb,ub, ...
                                nonlcon, ...
                                options);

        if exitflag <= 0
            fprintf(" \n \n \n \n \nPlanner Failed \n \n \n \n \n")
        end
        old_ref = Z_opt;

        Z_state_control_opt = reshape(Z_opt(2:end),nx+nu,N_nodes);

        WP = Z_state_control_opt(1:2,:).';
        vr = Z_state_control_opt(4,:);
        vl = Z_state_control_opt(5,:);

        if 1 
            %% debug plot
            fig_num = 1000 + fig_increment;
            fig = figure(fig_num);
            clf(fig)

            xk = Z_state_control_opt(1,:);
            yk = Z_state_control_opt(2,:);

            plot(xk,yk,'k--*') % Optimized Path
            hold on
            plot(pos(1),pos(2),'g*') % Start
            hold on
            plot(pos_curWP(1),pos_curWP(2),'rx') % End
            hold on

            % Ball constraint
            theta_plot = linspace(0,2*pi,1000);
            plot(obs_pos(1)+constraint_params.r_ball*cos(theta_plot),obs_pos(2)+constraint_params.r_ball*sin(theta_plot),'r--')
            plot(obs_pos(1)+(constraint_params.r_ball-vehicle_params.L)*cos(theta_plot),obs_pos(2)+(constraint_params.r_ball-vehicle_params.L)*sin(theta_plot),'r-')

            axis equal
            title("Planned Path")
            legend("Trajectory/Waypoints", "Start", "End", "Obstacle + margin", "Obstacle")
            xlabel("x [mm]")
            ylabel("y [mm]")
        end

        % Get First control input
        ur = vr(1);
        ul = vl(1);
    end

    % setting the PWM limits and wheel directions
    wr.DIRL = sign(ul);
    wr.DIRR = sign(ur);
    
    PWML = abs(ul);
    PWMR = abs(ur);

    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end