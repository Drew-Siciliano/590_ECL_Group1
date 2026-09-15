clear;
close all;
data_prefix = "lab02/lab_data/";

figure;
data = readmatrix(data_prefix + "data_20260908_1436_waypoint_70.csv");

x_pos = data(:,2);
y_pos = data(:,3);

waypoints = readmatrix("lab02/Rover_wp_lab2.xlsx");

plot(x_pos,y_pos,'.')
hold on
scatter(waypoints(:, 1), waypoints(:, 2), "rd")
viscircles(waypoints, 100*ones(length(waypoints), 1));
axis equal
grid minor
xlabel("x [mm]")
ylabel("y [mm]")
legend(["Trajectory", "Waypoints"],'Location', 'southwest')
title("Waypoint Tracking")

set(gca, 'FontSize', 16);

% Speed plot
data = readmatrix(data_prefix + "data_20260908_1430_speed_70.csv");
t = data(:, 1);
x_pos = data(:,2);
y_pos = data(:,3);
speed = [];
% Compute speed
for k = 1 : length(t)
    if k == 1
        continue
    end
    
    pos_diff = [x_pos(k) - x_pos(k-1), y_pos(k) - y_pos(k-1)];
    dist_traveled = norm(pos_diff);
    time_diff = t(k) - t(k-1);
    speed(k-1) = dist_traveled / time_diff;
end
speed = [0, speed];

figure
plot(t, speed)
hold on
yline(70, "r--")
legend("True speed", "Target speed")
grid on
title("Speed over Time")
set(gca, 'FontSize', 16);

% Heading plot
data = readmatrix(data_prefix + "data_20260908_1434_heading_70.csv");
t = data(:, 1);
theta = data(:, 4);
figure
plot(t, theta)
grid on
xlabel("Time [s]")
ylabel("Heading [deg]")

hold on
yline(-15, "r--")
yline(15, "r--")

legend(["Heading", "Dead zone"])
title("Heading over Time")
set(gca, 'FontSize', 16);
