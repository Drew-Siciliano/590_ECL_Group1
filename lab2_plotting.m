clear;
close all;
data_prefix = "lab02/lab_data/";

theta_plot = linspace(0,2*pi,1000);

%% Waypoint Plot
figure;
%data = readmatrix(data_prefix + "data_20260908_1436_waypoint_70.csv");
data = readmatrix(data_prefix + "data_20260908_1440_waypoint_200.csv");

x_pos = data(:,2);
y_pos = data(:,3);

waypoints = readmatrix("lab02/Rover_wp_lab2.xlsx");

plot(x_pos,y_pos,'.')
hold on
plot(x_pos(1),y_pos(2),'g*','MarkerSize',12)
hold on
plot(waypoints(:, 1), waypoints(:, 2), "rx",'LineStyle','none','MarkerSize',12)
r_deadzone = 100;
for i = 1:size(waypoints,1)
    hold on
    plot(waypoints(i,1)+r_deadzone*cos(theta_plot),waypoints(i,2)+r_deadzone*sin(theta_plot),'r-','LineWidth',2)
end

axis equal
grid minor
xlabel("x [mm]")
ylabel("y [mm]")
legend(["Trajectory", "Start Location","Waypoints"],'Location', 'northeast')
title("Waypoint Tracking")

set(gca, 'FontSize', 16);

%% Speed plot
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
