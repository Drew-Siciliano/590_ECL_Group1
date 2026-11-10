function [t2, time] = EvaderOutsideLoop(t2, time)
    % This is literally just setting waypoints slightly inside the
    % boundaries of the loop and just following the waypoints
    % set waypoints in mm
    offset = 100; % mm offset from corner
    corner1 = [400 + offset, 2200 - offset]; 
    corner2 = [400 + offset, 400 + offset];
    corner3 = [3200 - offset, 400 + offset];
    corner4 = [4400 - offset, 1300];
    corner5 = [4400 - offset, 2200 - offset];
    dist2corner = [norm(t2.pos - corner1); norm(t2.pos - corner2); ...
        norm(t2.pos - corner3); norm(t2.pos - corner4); norm(t2.pos - corner5)];
    
    % sort corners into waypoints based on how close they are
    [~, idx] = sort(dist2corner, 'ascend');
    t2.WP = [corner1; corner2; corner3; corner4; corner5];
    t2.WP = t2.WP(idx, :);

    % control wp function
    t2 = control_wp(t2, time);
    
    % set last WP reached to 1 so we can reloop through this
    t2.lastWPreached = 1;
end