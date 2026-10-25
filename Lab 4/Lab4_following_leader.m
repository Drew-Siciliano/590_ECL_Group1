clear;
close all;

set(0, 'DefaultLineLineWidth', 2);

%data = readtable("mission_data\dataw1p5Follower.csv");
%data = readtable("mission_data\data1p5Follower2.csv");
data = readtable("mission_data\data1p5Follower3_distMarg10.csv");

tank1 = [data.Var2.';data.Var3.'];
tank2 = [data.Var4.';data.Var5.'];
tank3 = [data.Var6.';data.Var7.'];

tank1_rel = tank1 - tank3;
tank2_rel = tank2 - tank3;
tank3_rel = tank3 - tank3;

tank_color = ["r","b","m"];

figure;
subplot(1,2,1)
plot(tank1(1,:),tank1(2,:),'-','LineWidth',2,"Color",tank_color(1))
hold on
plot(tank1(1,end),tank1(2,end),'LineStyle','none','Marker','x','MarkerSize',12,"Color",tank_color(1))
hold on
plot(tank2(1,:),tank2(2,:),'-','LineWidth',2,"Color",tank_color(2))
hold on
plot(tank2(1,end),tank2(2,end),'LineStyle','none','Marker','x','MarkerSize',12,"Color",tank_color(2))
hold on
plot(tank3(1,:),tank3(2,:),'-','LineWidth',2,"Color",tank_color(3))
hold on
plot(tank3(1,end),tank3(2,end),'LineStyle','none','Marker','x','MarkerSize',12,"Color",tank_color(3))
legend("Follower 1","Follower 1 @ t_f", ...
       "Follower 2","Follower 2 @ t_f", ...
       "Leader","Leader @ t_f", ...
       'Location','south')
grid on
axis equal
xlabel("x [mm]");
ylabel("y [mm]");
title("Relative to Origin")

subplot(1,2,2)
plot(tank1_rel(1,:),tank1_rel(2,:),'-','LineWidth',2,"Color",tank_color(1))
hold on
plot(tank1_rel(1,end),tank1_rel(2,end),'LineStyle','none','Marker','x','MarkerSize',12,"Color",tank_color(1))
hold on
plot(tank2_rel(1,:),tank2_rel(2,:),'-','LineWidth',2,"Color",tank_color(2))
hold on
plot(tank2_rel(1,end),tank2_rel(2,end),'LineStyle','none','Marker','x','MarkerSize',12,"Color",tank_color(2))
hold on
plot(0,0,'LineStyle','none','Marker','pentagram','MarkerSize',12,"Color",tank_color(3))
hold on

% Triangle
L = 1000;
h = sqrt(3)/2 * L;
hold on
plot([0, -L/2, L/2, 0], [0, -h, -h, 0], ...
     '--', 'LineWidth', 2, 'Color', 'k')

legend("Follower 1","Follower 1 @ t_f", ...
       "Follower 2","Follower 2 @ t_f", ...
       "Leader", ...
       'Location','south')
grid on
axis equal
xlabel("x [mm]");
ylabel("y [mm]");
title("Relative to Leader")