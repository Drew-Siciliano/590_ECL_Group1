%% Intializaion
clear; clc

vhc_param;
L = 151.2;

wr.time_since_gain = 10;
wr.old_vr = 0;
wr.old_vl = 0;
t2.time_since_gain = 10;
t2.old_vr = 0;
t2.old_vl = 0;

time.curr = 0;
time.dt = 0.01;

% WR settup
wr.pos = [1000, 1300]; % Initial Pos [mm]
wr.PWML = 0; wr.PWMR = 0;
wr.heading = pi/4;
wr.front_marker = [wr.pos(1) + L*cos(wr.heading), wr.pos(2) + L*sin(wr.heading)];
wr.back_marker = [wr.pos(1) - L*cos(wr.heading), wr.pos(2) - L*sin(wr.heading)];

% T2 settup
t2.pos = [2500, 1300]; % Initial Pos [mm]
t2.PWML = 0; t2.PWMR = 0;
t2.heading = 0;
t2.front_marker = [t2.pos(1) + L*cos(t2.heading), t2.pos(2) + L*sin(t2.heading)];
t2.back_marker = [t2.pos(1) - L*cos(t2.heading), t2.pos(2) - L*sin(t2.heading)];

% For recording updates
wr.pos_history = [];
t2.pos_history = [];
time.history = [];

% For iterating updates
devices.wr = wr;
devices.t2 = t2;
names = fieldnames(devices);
%% Simulation
disable = 0;
while (disable < 1)
    % Execute

    % Chasing Algorithm
    wr = chaser(wr, t2, time);
    % Evasion Algorithm
    t2 = evader(wr, t2, time);

    devices.wr = wr; devices.t2 = t2;
    
    % Update
    for i = 1:length(names)
        x = devices.(names{i});
        if x.PWMR == 0
            x.vr = 0;
        else
            pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(x.PWMR)];
            x.vr = roots(pwmr_cubic); 
            x.vr = x.vr(imag(x.vr) == 0);
            if x.DIRR == 0
                x.vr = -x.vr;
            end
        end
        if x.PWML == 0
            x.vl = 0;
        else
            pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(x.PWML)];
            x.vl = roots(pwml_cubic); 
            x.vl = x.vl(imag(x.vl) == 0);
            if x.DIRL == 0 
                x.vl = -x.vl;
            end
        end
    
        x.v = (x.vr + x.vl) / 2;
        x.pos = x.pos + x.v * [cos(x.heading), sin(x.heading)] * time.dt;
    
        x.w = (x.vr - x.vl) / L;
        x.heading = x.heading + x.w * time.dt;
    
        x.front_marker = [x.pos(1) + L*cos(x.heading), x.pos(2) + L*sin(x.heading)];
        x.back_marker = [x.pos(1) - L*cos(x.heading), x.pos(2) - L*sin(x.heading)];
    
        x.pos_history(:, end + 1) = x.pos;
        devices.(names{i}) = x;
    end

    wr = devices.wr;
    t2 = devices.t2;

    time.curr = time.curr + time.dt;
    time.history = [time.history, time.curr];

    % End simulation checks
    if wr.pos(1)<400 | wr.pos(1)>4400 | wr.pos(2)<400 | wr.pos(2)>2200
        disp('WR is out of the boundary');
        disable = disable + 1;
    end

    if wr.pos(1)>3200 & (wr.pos(2)-400)/(wr.pos(1)-3200) < 0.75
        disp('WR is out of the boundary');
        disable = disable + 1;
    end
    
    if t2.pos(1)<400 | t2.pos(1)>4400 | t2.pos(2)<400 | t2.pos(2)>2200
        disp('T2 is out of the boundary');
        disable = disable + 1;
    end

    if t2.pos(1)>3200 & (t2.pos(2)-400)/(t2.pos(1)-3200) < 0.75
        disp('T2 is out of the boundary');
        disable = disable + 1;
    end
    % Check if tag is achieved
    dis12 = sqrt((wr.pos(1)-t2.pos(1))^2+(wr.pos(2)-t2.pos(2))^2);
    if dis12<200
        disable = disable + 1;
        fprintf('\nChaser WINS\n');
    end
end
%% Visualization

figure
plot(time.history, wr.pos_history, 'LineWidth', 1.5)
hold on
plot(time.history, t2.pos_history, '--', 'LineWidth', 1.5)
title('Rover Positions vs. Time')
xlabel('Time [s]')
ylabel('Position [mm]')
legend('WR X', 'WR Y', 'T2 X', 'T2 Y', 'Location', 'best')
grid on

figure
hold on
c = time.history;
c = c(:)';

surface([wr.pos_history(1, :) ; wr.pos_history(1, :)], ...
        [wr.pos_history(2, :) ; wr.pos_history(2, :)], ...
        zeros(2, numel(c)), ...
        [c ; c], ...
        'FaceColor', 'none', ...
        'EdgeColor', 'interp', ...
        'LineWidth', 1.5)

surface([t2.pos_history(1, :) ; t2.pos_history(1, :)], ...
        [t2.pos_history(2, :) ; t2.pos_history(2, :)], ...
        zeros(2, numel(c)), ...
        [c ; c], ...
        'FaceColor', 'none', ...
        'EdgeColor', 'interp', ...
        'LineWidth', 1.5)

x = [400 3200 4400 4400 400 400];
y = [400 400 1300 2200 2200 400];
pgon = polyshape(x, y);
plot(pgon, 'FaceColor', 'none', 'EdgeColor', 'blue')

title('Rover X vs. Y over time')
xlabel('X [mm]')
ylabel('Y [mm]')
axis equal
xlim([0 4800])
ylim([0 2600])
grid on
colormap(turbo)
cb = colorbar;
cb.Label.String = 'Time [s]';
clim([min(time.history) max(time.history)])
