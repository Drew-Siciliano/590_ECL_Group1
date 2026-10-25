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
    x = [wr.pos; t2.pos; t3.pos];

    % desired formation locations 3x2 X, Y
    xi = [wr.curWP; t2.curWP; t3.curWP];
    % define tau to be displacement of robot position from target location
    tau = x - xi;
    w = 1.5;
    tauDot = - w * Lg * tau; % 3x2 matrix of Xdot, Ydot for 3 robots

    %% Convert Xdot Ydot to speed and headings
    speeds = [norm(tauDot(1, :)); norm(tauDot(2, :)); norm(tauDot(3, :))];
    headings = rad2deg([atan2(tauDot(1, 2), tauDot(1, 1)); ...
                atan2(tauDot(2, 2), tauDot(2, 1)); ...
                atan2(tauDot(3, 2), tauDot(3, 1))]);
    wr.speed_error_old = 0;
    wr.cum_speed_error_int = 0;
    wr.dist_mar = 10;
    wr.fwd_deg = 5; % deg
    wr_target_heading = headings(1);
    wr.forward_spd = speeds(1);
    current_heading = atan2d(wr.heading_vec(2), wr.heading_vec(1));
    angle_curWP = wrapTo180(wr_target_heading - current_heading);
    if abs(angle_curWP) <= wr.fwd_deg
        fprintf("\nWR Speed\n")

        % Reached Heading, Going Forwards
        wr = wr_control_spd(wr, time);
    else
        % Control Heading
        fprintf("\nWR Heading\n")

        wr = wr_control_heading(wr, time, wr_target_heading);
    end

    t2.speed_error_old = 0;
    t2.cum_speed_error_int = 0;
    t2.dist_mar = 10;
    t2.fwd_deg = 15; % deg
    t2.forward_spd = speeds(2);
    t2_target_heading = headings(2);
    current_heading = atan2d(t2.heading_vec(2), t2.heading_vec(1));
    angle_curWP = wrapTo180(t2_target_heading - current_heading);
    if abs(angle_curWP) <= t2.fwd_deg
        fprintf("\nt2 Speed\n")

        % Reached Heading, Going Forwards
        t2 = wr_control_spd(t2, time);
    else
        % Control Heading
        fprintf("\nt2 Heading\n")

        t2 = wr_control_heading(t2, time, t2_target_heading);
    end

    t3.speed_error_old = 0;
    t3.cum_speed_error_int = 0;
    t3.dist_mar = 10;
    t3.fwd_deg = 15; % deg
    % t3.forward_spd = speeds(3);
    t3.forward_spd = 700;
    t3_target_heading = headings(3);
    current_heading = atan2d(t3.heading_vec(2), t3.heading_vec(1));
    angle_curWP = wrapTo180(t3_target_heading - current_heading);
    t3 = wr_control_spd(t3, time);
    % if abs(angle_curWP) <= t3.fwd_deg
    %     fprintf("\nt3 Speed\n")
    % 
    %     % Reached Heading, Going Forwards
    %     t3 = wr_control_spd(t3, time);
    % else
    %     % Control Heading
    %     fprintf("\nt3 Heading\n")
    % 
    %     t3 = wr_control_heading(t3, time, t3_target_heading);
    % end
    % 
    %% Mapping from Xdot Ydot to PWM
    % Compute V, velocity of robot
    % V = [atan2(tauDot(1, 2), tauDot(1, 1)); 
    %      atan2(tauDot(2, 2), tauDot(2, 1));
    %      atan2(tauDot(3, 2), tauDot(3, 1))];
    % % map V to PWM
    % PWM = 1.3027e-7 * V .^ 3 - 1.8165e-4 * V .^ 2 + 0.3162 * V + 2.4951;
    % 
    % % set robot PWMs
    % wr.PWML = PWM(1);
    % wr.PWMR = PWM(1);
    % t2.PWML = PWM(2);
    % t2.PWMR = PWM(2);
    % t3.PWML = PWM(3);
    % t3.PWMR = PWM(3);

    % %% WR speed toggle
    disp(wr.PWML)
    disp(wr.PWMR)
    % setting the PWM limits
    wr.PWML = min(150,  wr.PWML);
    wr.PWML = uint8(max(0,  wr.PWML));
    wr.PWMR = min(150,  wr.PWMR);
    wr.PWMR = uint8(max(0,  wr.PWMR));
    fprintf("asdaefwef")
    disp(wr.PWML)
    disp(wr.PWMR)

    t2.PWML = min(150,  t2.PWML);
    t2.PWML = uint8(max(0,  t2.PWML));
    t2.PWMR = min(150,  t2.PWMR);
    t2.PWMR = uint8(max(0,  t2.PWMR));
    
    t3.PWML = min(150,  t3.PWML);
    t3.PWML = uint8(max(0,  t3.PWML));
    t3.PWMR = min(150,  t3.PWMR);
    t3.PWMR = uint8(max(0,  t3.PWMR));

    %% Disable
    if norm(tau(1, :)) < 100 && norm(tau(2, :)) < 100 && norm(tau(3, :)) < 100
        disable = 1;
    elseif abs(norm(wr.pos - t2.pos) - 1e3) < 50 && abs(norm(wr.pos - t3.pos) - 1e3) < 50 && abs(norm(t3.pos - t2.pos) - 1e3) < 50
        disable = 1;
    else
        disable = 0;
    end
end