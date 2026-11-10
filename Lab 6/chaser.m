function wr = chaser(wr,t2,time)
    %% Assumptions
    % wr is chaser
    % t2 is evader
    %% Establish variables for all codes
    % Find headings [rads]
    % Chaser
    chaserHeadingVector = [ ...
        wr.front_marker(1) - wr.back_marker(1); ...
        wr.front_marker(2) - wr.back_marker(2)];
    theta_chaser = atan2( ...
        chaserHeadingVector(2), ...
        chaserHeadingVector(1));
    wr.heading = wrapToPi(theta_chaser);
    % Evader
    evaderHeadingVector = [t2.front_marker(1) - t2.back_marker(1); ...
        t2.front_marker(2) - t2.back_marker(2)];
    theta_evader = atan2( ...
        evaderHeadingVector(2), ...
        evaderHeadingVector(1));
    wr.evader_heading = wrapToPi(theta_evader);
    % Find some other thing thats being recaculated
    wr.u_max = 150;
    wr.vr = wr.old_vr;
    wr.vl = wr.old_vl;
    %% Select which chaser algo to use
    chaser = 2;

    if any(isnan(wr.pos))
        disp("Chaser is NaN")
    else
        if chaser == 1
            wr = chaser_Main(wr,t2,time);
        elseif chaser == 2
            wr = chaser_Dan(wr,t2,time);
        else
            error('Unsupported chaser algorithm: %d', chaser);
        end
    end
    %% Print statements here for debugging
    fprintf("Chaser heading is %d", wr.heading)
    %% Establish variables that are old now
    wr.old_vr = wr.vr;
    wr.old_vl = wr.vl;
    %% Setting the PWM limits
    wr.PWML = min(150,  wr.PWML);
    wr.PWML = uint8(max(0,  wr.PWML));
    wr.PWMR = min(150,  wr.PWMR);
    wr.PWMR = uint8(max(0,  wr.PWMR));
end