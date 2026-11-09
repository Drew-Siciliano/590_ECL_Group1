function [wr, time] = EvaderOutsideLoop(wr, time)
    % This is literally just setting waypoints slightly inside the
    % boundaries of the loop and just following the waypoints
    % set waypoints in mm
    offset = 100; % mm offset from corner
    corner1 = [400 + offset, 2200 - offset]; 
    corner2 = [400 + offset, 400 + offset];
    corner3 = [3200 - offset, 400 + offset];
    corner4 = [4400 - offset, 1300];
    corner5 = [4400 - offset, 2200 - offset];
    dist2corner = [norm(wr.pos - corner1); norm(wr.pos - corner2); ...
        norm(wr.pos - corner3); norm(wr.pos - corner4); norm(wr.pos - corner5)];
    
    % sort corners into waypoints based on how close they are
    [~, idx] = sort(dist2corner, 'ascend');
    wr.WP = [corner1; corner2; corner3; corner4; corner5];
    wr.WP = wr.WP(idx, :);

    % control wp function
    wr = wr_control_wp(wr, time);
    
    % set last WP reached to 1 so we can reloop through this
    wr.lastWPreached = 1;
end