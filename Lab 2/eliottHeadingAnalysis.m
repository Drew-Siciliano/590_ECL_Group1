clear;
close all;
clc;
%% PID Gains
Kp = -3.52208291564815;
Ki = 1.25956669370519;
Kd = 11.7688369455175;
N = 0.4;
%% Laplace variable
s = tf('s');
%% Speed transfer function
Plant = tf([0.4972 0.1716 0.3878 0.1714], [1 0.7091 1.551 0.4573 0.5846]);
%% PID controller
PID = Kp + Ki/s + Kd*(N*s)/(s + N);
%% Closed-loop transfer function
Ts = feedback(PID * Plant, 1);
%% Simulate unit step response
figure
step(Ts)
grid on
title('Closed-Loop Step Response')
%% Step-response characteristics
info = stepinfo(Ts);
fprintf('Step Response\n')
fprintf('Overshoot:          %.2f %%\n', info.Overshoot);
fprintf('Settling Time:      %.4f s\n', info.SettlingTime);
fprintf('Rise Time:          %.4f s\n', info.RiseTime);
fprintf('Peak Time:          %.4f s\n', info.PeakTime);
fprintf('Peak Value:         %.4f\n', info.Peak);
%% Steady-state response
yss = dcgain(Ts);
ess = 1 - yss;
fprintf('Steady-State Value: %.4f\n', yss);
fprintf('Steady-State Error: %.4e\n', ess);
%% Heading 180 analysis
heading = 180;
t = 0:0.001:20;

[y, t] = step(heading * Ts, t);

figure;
plot(t, y)
hold on
yline(heading, 'r--')
yline(heading + 15, 'g--')
yline(heading - 15, 'g--')
grid on
xlabel('Time [s]')
ylabel('Speed')
ylim([min(y), max(y) + 20])
legend('Simulated Response', 'Command')

heading180 = stepinfo(y, t, 180);
overshoot = heading180.Overshoot;
settlingTime = heading180.SettlingTime;
steadyStateError = 180 - y(end);

fprintf('Speed 120\n')
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Settling Time: %.4f s\n', settlingTime);
fprintf('Steady-State Error: %.4f\n', steadyStateError);
fprintf('Rise Time:          %.4f s\n', heading180.RiseTime);
fprintf('Peak Time:          %.4f s\n', heading180.PeakTime);
fprintf('Peak Value:         %.4f\n', heading180.Peak);