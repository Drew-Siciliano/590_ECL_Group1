function pb = pb_control_wp(pb, time, docking)
% Add your controller here..
% Set pb.done = true when all targets are met within your tolerances.
pb.VALF = 0;
pb.DIRF = 0;
pb.VALR = 0;
pb.DIRR = 0;

heading_dock_error = deg2rad(1);
distanc_dock_error = 5;

% if near dock, move slowly; else, move fast
if (docking == true)
    fwdspeed = 10;
    heading = pb.dock(3);
else
    fwdspeed = 80;
    heading = atan2(900 - pb.state(2), ...
        1000 - pb.state(1));
end

theta = pb.state(3);
thetap = heading;

% heading error
heading_error = thetap - theta;
if ((thetap - theta) > pi)
    heading_error = (thetap - theta) - 2 * pi;
elseif ((thetap - theta) < -pi)
    heading_error = (thetap - theta) + 2 * pi;
end
dock_distance = norm([pb.dock(2) - pb.state(2), ...
    pb.dock(1) - pb.state(1)]);

% if we are close to docking, slow down
if (heading_error < heading_dock_error && dock_distance < distanc_dock_error)
    pb.forward_spd = 0;
    pb = pb_control_spd(pb, time);
end

% calculate velocity at docking
vx = (pb.state(1) - pb.state_old(1)) / time.dt;
vy = (pb.state(2) - pb.state_old(2)) / time.dt;

speed = vx * cos(pb.state(3)) + vy * sin(pb.state(3));

% if docked, exit
if (abs(speed) < 2)
    pb.done = true;
    return;
end

% make sure we dont translate until we stopped rotating
theta = pb.state(3);
thetaOld = pb.state_old(3);
dtheta = theta - thetaOld;
if ((theta - thetaOld) > pi)
    dtheta = (theta - thetaOld) - 2 * pi;
elseif ((theta - thetaOld) < -pi)
    dtheta = (theta - thetaOld) + 2 * pi;
end
omega = dtheta / time.dt;

% correct heading, then translate
if (abs(heading_error) > heading_dock_error)
    pb.heading_rad = heading;
    pb = pb_control_heading(pb, time);
elseif (abs(omega) < pi / 4)
    pb.forward_spd = fwdspeed;
    pb = pb_control_spd(pb, time);
end
end
