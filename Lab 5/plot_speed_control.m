clear;
close all;

set(0, 'DefaultLineLineWidth', 2);

data = readtable("lab_data\final_results\speed_70.csv");

time = data.time_s_;
position = [data.pb_x_mm_.';data.pb_y_mm_.'];
speed = data.speed_mm_s_;
heading = wrapTo360(rad2deg(data.yaw_rad_));


fig = figure('Position',[100,100,800,600]);
subplot(2,1,1)
plot(position(1,:),position(2,:))
hold on
plot(position(1,1),position(2,1),'g*')
grid on
axis equal
legend("Position Data","Start")
xlabel("x [mm]")
ylabel("y [mm]")

subplot(2,1,2)
plot(time,heading)
ylim([0,360])
xlabel("time [s]")
ylabel("heading [deg]")


% Lowpass filter on speed
dt = mean(diff(time));          
Fs = 1/dt;           
fc = 2;              
speed_filtered = lowpass(speed, fc, Fs);

fig2 = figure('Position',[100,100,800,600]);
plot(time,speed)
hold on
plot(time,speed_filtered)
hold on
yline(70,'r--','LineWidth',2)
legend("Raw Speed","Filtered Speed","Target Speed")
xlabel("time [s]")
ylabel("speed [mm/s]")
