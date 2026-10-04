clear;
close all;
data_prefix = "lab_data/";
static = false;
if static
    plotted_file = "obs_static.csv";
else
    plotted_file = "obs_moving.csv";
    end_move_time = 1487;
end
data = readmatrix(data_prefix + plotted_file);

% Initialize data
initial_pos = data(1, 2:3);
wr.pos = initial_pos;
initial_heading = data(1, 4);
wr.heading_vec(1:2) = [cosd(initial_heading); sind(initial_heading)];
assert(abs(atan2d(wr.heading_vec(2), wr.heading_vec(1)) - initial_heading) < 1e-12)
wr.dist_mar = 100;
wr.cur_WP = 1;
fig_increment = 0;

% Moving obstacle plotting
r_target = [3.5,1.3].'*1000;
initial_obs_pos = data(1, 5:end);

% Compute and plot planned path
[wr, WP, ~] = path_planner(wr, r_target, initial_obs_pos, [], fig_increment);

% Plot true path 
if static
    pos_data = data(:, 2:3);
else
    pos_data = data(1:end_move_time, 2:3);
end

figure(1000)
plot(pos_data(:, 1), pos_data(:, 2), "b-")
title("True and Planned Paths, Original Obstacle Location")
legend("Planned trajectory/WPs", "Start", "End", "Obstacle + margin", "Obstacle", "True trajectory")

if ~static
    % Recompute path after moving obstacle
    new_pos = data(end_move_time+1, 2:3);
    wr.pos = new_pos;
    new_heading = data(end_move_time+1, 4);
    wr.heading_vec(1:2) = [cosd(new_heading); sind(new_heading)];
    wr.cur_WP = 1;
    new_obs_pos = data(end_move_time+1, 5:end);
    fig_increment = 10;

    [wr, WP, ~] = path_planner(wr, r_target, new_obs_pos, [], fig_increment);
    
    % Plot true path after obs moved
    pos_data = data(end_move_time+1:end, 2:3);
    fig_num = 1000+fig_increment;
    figure(fig_num)
    plot(pos_data(:, 1), pos_data(:, 2), "b-")
    title("True and Planned Paths, New Obstacle Location")
    legend("Planned trajectory/WPs", "Start", "End", "Obstacle + margin", "Obstacle", "True trajectory")
end
