function x = control_heading(x, time, target_heading)
    x.PWML = uint8(0);
    x.PWMR = uint8(0);
    x.DIRL = 1;
    x.DIRR = 1;

    Kp = -3.52208291564815;
    Ki = 1.25956669370519;
    Kd = 11.7688369455175;
    % Ki = 0;
    % Kd = 0;

    dt = time.dt;
    %% Calculate the angle remaining to face the waypoint
    % pos_curWP = reshape(wr.WP(wr.curWP,:), 2, 1);
    % vec_wr2WP = pos_curWP - wr.pos(:);

    % Desired heading is angle of the waypoint vector from the +x axis.
    % target_heading = atan2d(vec_wr2WP(2), vec_wr2WP(1));
    %target_heading = 0;

    % Actual heading is angle of the rover's measured heading vector.
    x.current_heading = atan2d(x.heading_vec(2), x.heading_vec(1));

    % Error is desired heading minus actual heading.
    x.e_heading = wrapTo180(target_heading - x.current_heading);
    if abs(x.e_heading) >= x.fwd_deg
        e = x.e_heading;
        %% PID: convert heading error into a signed motor comman
        % Integral
        % I = wr.e_heading_cum + e * dt;
        % Derivative:
        x.e_heading_old = 0;
        D = wrapTo180(e - x.e_heading_old) / dt;
        % Proportional
        % u = Kp * e + Ki * I + Kd * D;
        u = Kp * e + Kd * D;
        % Clamp the actuator
        u = max(-150, min(150, u));
        %% Assign wheel directions
        % Positive u: left reverse, right forward.
        % Negative u: left forward, right reverse.
        x.DIRL = double(u <= 0);
        x.DIRR = double(u >= 0);
    
        PWML = abs(u);
        PWMR = abs(u);
    else
        PWML = 0;
        PWMR = 0;
    end

    % setting the PWM limits, keep the codes here
    PWML = min(150, PWML);
    x.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    x.PWMR = uint8(max(0, PWMR));
end