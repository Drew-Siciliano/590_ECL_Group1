function x = control_wp(x, time)
   
    %% Calc waypoint distances
    pos_curWP = x.WP(x.curWP,:); 

    % Distance to Waypoint
    vec_wr2WP = pos_curWP - x.pos;
    dist_curWP = norm(vec_wr2WP,2);

    % Heading
    wr_target_heading = atan2d(vec_wr2WP(2), vec_wr2WP(1));
    current_heading = atan2d(x.heading_vec(2), x.heading_vec(1));

    angle_curWP = wrapTo180(wr_target_heading - current_heading);

    if dist_curWP <= x.dist_mar
        %% Reached Waypoint
        fprintf(" \n \n \n \n \nWayPoint "+string(x.curWP)+" reached \n \n \n \n \n")
        x.curWP = x.curWP + 1;

        x.current_heading = current_heading;

        ul = 0;
        ur = 0;
        x.DIRL = sign(ul);
        x.DIRR = sign(ur);

        PWML = abs(ul);
        PWMR = abs(ur);

        PWML = min(150, PWML);
        x.PWML = uint8(max(0, PWML));
        PWMR = min(150, PWMR);
        x.PWMR = uint8(max(0, PWMR));
    else    
        if abs(angle_curWP) <= x.fwd_deg
            fprintf("\nSpeed\n")

            %% Reached Heading, Going Forwards
            x = control_spd(x, time);
        else
            %% Control Heading
            fprintf("\nHeading\n")

            x = control_heading(x, time);
        end
    end
end