function [wr] = wr_control_wp(wr, time)
   
    %% Calc waypoint distances
    pos_curWP = reshape(wr.WP(wr.curWP,:),2,1); 

    % Distance to Waypoint
    vec_wr2WP = pos_curWP - wr.pos;
    dist_curWP = norm(vec_wr2WP,2);

    % Heading
    wr_target_heading = acosd(vec_wr2WP(1)/dist_curWP);
    angle_curWP = wrapTo180(wr_target_heading - wr.heading_dir);

    if dist_curWP <= wr.dist_mar
        %% Reached Waypoint
        fprintf("WayPoint "+string(wr.curWP)+" reached \n")
        wr.curWP = wr.curWP + 1;
    else    
        if abs(angle_curWP) <= wr.fwd_deg
            %% Reached Heading, Going Forwards
            wr = wr_control_spd(wr, time);
        else
            %% Control Heading
            wr = wr_control_heading(wr, time);
        end
    end
end