clear;
close all;
data_prefix = "lab_data/final_results/";
plotted_file = "heading_70_better.csv";
data = readmatrix(data_prefix + plotted_file);

% Initialize data
time = data(:, 1);
heading = data(:, 4);
n_rows = length(time);

ang_velo_fixedstep = [];
ang_velo = [];
time_diffs = [];
for k = 2 : n_rows
    ang_velo_k = (heading(k) - heading(k-1)) / (time(k) - time(k-1));
    time_diffs = [time_diffs; time(k) - time(k-1)];
    ang_velo_k_fixedstep = (heading(k) - heading(k-1)) / 0.01;
    ang_velo = [ang_velo; ang_velo_k];
    ang_velo_fixedstep = [ang_velo_fixedstep; ang_velo_k_fixedstep];
end

figure
subplot(2, 1, 1)
plot(time, heading)
ylabel("Heading [rad]")
grid on

subplot(2, 1, 2)
plot(time(2:end), ang_velo)
ylabel("Angular velocity [rad/s]")
xlabel("Time [s]")
grid on
sgtitle("Heading and Angular Velocity Over Time")

figure
subplot(2, 1, 1)
plot(time, heading)
ylabel("Heading [rad]")
grid on

subplot(2, 1, 2)
plot(time(2:end), ang_velo_fixedstep)
ylabel("Angular velocity [rad/s]")
xlabel("Time [s]")
grid on
sgtitle("Heading and Angular Velocity Over Time, Fixed Time Steps")

figure
plot_ks = 2:n_rows;
plot(plot_ks, time_diffs)
xlabel("Time step")
ylabel("$\Delta$ t [s]", "Interpreter", "latex")
title("Time Step Lengths Over Simulation")