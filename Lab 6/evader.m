function t2 = evader(wr,t2,time)
    %% Assumptions
    % wr is chaser
    % t2 is evader
    %% Establish variables for all codes
    % Find headings [rads]
    % Chaser
    chaserHeadingVector = [wr.front_marker(1) - wr.back_marker(1); wr.front_marker(2) - wr.back_marker(2)];
    theta_chaser = acos(dot(chaserHeadingVector, [1; 0])/(norm(chaserHeadingVector)*norm([1; 0])));
    t2.chaser_heading = wrapToPi(theta_chaser);
    % Evader
    evaderHeadingVector = [t2.front_marker(1) - t2.back_marker(1); t2.front_marker(2) - t2.back_marker(2)];
    theta_evader = atan2(evaderHeadingVector(2), evaderHeadingVector(1));
    t2.heading = wrapToPi(theta_evader);
    % Find some other thing thats being recaculated
    t2.u_max = 150;
    t2.vr = t2.old_vr;
    t2.vl = wl.old_vl;
    %% Select which chaser algo to use
    evader = 1;

    if evader == 1
        t2 = evader_Main(wr,t2,time);
    elseif evader == 2
        t2 = EvaderSpinOnDatThang(t2,time);
    elseif evader == 3
        t2 = EvaderOutsideLoop(t2,time);
    else
        error('Unsupported evader algorithm: %d', evader);
    end
    %% Establish variables that are old now
    t2.old_vr = t2.vr;
    t2.old_vl = t2.vl;
    %% Setting the PWM limits
    t2.PWML = min(150,  t2.PWML);
    t2.PWML = uint8(max(0,  t2.PWML));
    t2.PWMR = min(150,  t2.PWMR);
    t2.PWMR = uint8(max(0,  t2.PWMR));
end
