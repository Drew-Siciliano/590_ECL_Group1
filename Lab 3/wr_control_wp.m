function [wr] = wr_control_wp(wr, time)
   
    %% Calc waypoint distances
    pos_curWP = wr.WP(wr.curWP,:); 

    % Distance to Waypoint
    vec_wr2WP = pos_curWP - wr.pos;
    dist_curWP = norm(vec_wr2WP,2);

    % Heading
    wr_target_heading = atan2d(vec_wr2WP(2), vec_wr2WP(1));
    current_heading = atan2d(wr.heading_vec(2), wr.heading_vec(1));

    angle_curWP = wrapTo180(wr_target_heading - current_heading);

    if dist_curWP <= wr.dist_mar
        %% Reached Waypoint
        fprintf(" \n \n \n \n \nWayPoint "+string(wr.curWP)+" reached \n \n \n \n \n")
        wr.curWP = wr.curWP + 1;

        wr.current_heading = current_heading;

        ul = 0;
        ur = 0;
        wr.DIRL = sign(ul);
        wr.DIRR = sign(ur);

        PWML = abs(ul);
        PWMR = abs(ur);

        PWML = min(150, PWML);
        wr.PWML = uint8(max(0, PWML));
        PWMR = min(150, PWMR);
        wr.PWMR = uint8(max(0, PWMR));
    else    
        if abs(angle_curWP) <= wr.fwd_deg
            fprintf("\nSpeed\n")

            %% Reached Heading, Going Forwards
            wr = wr_control_spd(wr, time);
        else
            %% Control Heading
            fprintf("\nHeading\n")

            wr = wr_control_heading(wr, time);
        end
    end
end