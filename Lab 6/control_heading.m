function x = control_heading(x, time, target_heading)
%
% Inputs:
%   x - robot struct
%   time - time struct
%   target_heading - Desired robot heading [radians]
% 
    x.PWML = uint8(0);
    x.PWMR = uint8(0);
    x.DIRL = 1;
    x.DIRR = 1;

    Kp = 3326.99627918386;

    dt = time.dt;

    x.e_heading = wrapToPi(target_heading - x.heading); % [rads]
    if abs(x.e_heading) >= deg2rad(x.fwd_deg)
        u = Kp * x.e_heading;
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