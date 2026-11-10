function [t2, time] = EvaderSpinOnDatThang(t2, time)
    t2.PWML = uint8(0);
    t2.PWMR = uint8(0);
    t2.DIRL = 1;
    t2.DIRR = 1;

    Kp = -3.52208291564815;
    Ki = 1.25956669370519;
    Kd = 11.7688369455175;

    % Desired heading is angle of the waypoint vector from the +x axis.
    %% idk spin on dat thang should prob use chaser heading somehow to mess with them
    target_heading = t2.heading + pi;

    % Error is desired heading minus actual heading.
    t2.e_heading = wrapTo180(target_heading - t2.heading);
    if abs(t2.e_heading) >= t2.fwd_deg
        e = t2.e_heading;
        %% PID: convert heading error into a signed motor comman
        % Integral
        I = t2.e_heading_cum + e * time.dt;
        % Derivative:
        D = wrapTo180(e - t2.e_heading_old) / time.dt;
        % Proportional
        u = Kp * e + Ki * I + Kd * D;
        % Clamp the actuator
        u = max(-150, min(150, u));
        %% Assign wheel directions
        % Positive u: left reverse, right forward.
        % Negative u: left forward, right reverse.
        t2.DIRL = double(u <= 0);
        t2.DIRR = double(u >= 0);
        t2.PWML = abs(u);
        t2.PWMR = abs(u);
    else
        t2.PWML = 0;
        t2.PWMR = 0;
    end
end