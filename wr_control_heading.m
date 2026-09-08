function wr = wr_control_heading(wr, time)
    wr.PWML = uint8(0);
    wr.PWMR = uint8(0);
    wr.DIRL = 1;
    wr.DIRR = 1;

    Kp = -3.52208291564815;
    Ki = 1.25956669370519;
    Kd = 11.7688369455175;

    dt = time.dt;
    %% Calculate the angle remaining to face the waypoint
    pos_curWP = reshape(wr.WP(wr.curWP,:), 2, 1);
    vec_wr2WP = pos_curWP - wr.pos(:);

    % Desired heading is angle of the waypoint vector from the +x axis.
    target_heading = atan2d(vec_wr2WP(2), vec_wr2WP(1));

    % Actual heading is angle of the rover's measured heading vector.
    current_heading = atan2d(wr.heading_vec(2), wr.heading_vec(1));

    % Error is desired heading minus actual heading.
    wr.e_heading = wrapTo180(target_heading - current_heading);
    e = wr.e_heading;
    %% PID: convert heading error into a signed motor comman
    % Integral
    I = wr.e_heading_cum + e * dt;
    % Derivative:
    D = wrapTo180(e - wr.e_heading_old) / dt;
    % Proportional
    u = Kp * e + Ki * I + Kd * D;
    % Clamp the actuator
    u = max(-150, min(150, u));
    %% Assign wheel directions
    % Positive u: left reverse, right forward.
    % Negative u: left forward, right reverse.
    wr.DIRL = double(u <= 0);
    wr.DIRR = double(u >= 0);

    PWML = abs(u);
    PWMR = abs(u);

    % setting the PWM limits, keep the codes here
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end