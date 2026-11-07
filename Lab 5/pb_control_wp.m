function [pb, docking] = pb_control_wp(pb, time, docking)
% Add your controller here..
% Set pb.done = true when all targets are met within your tolerances.
pb.VALF = 0;
pb.DIRF = 0;
pb.VALR = 0;
pb.DIRR = 0;

heading_dock_error = deg2rad(10);
distanc_dock_error = 100;

heading = atan2(pb.dock(2) - pb.state(2), ...
          pb.dock(1) - pb.state(1));

theta = pb.state(3);
thetap = heading;

% heading error
heading_error = thetap - theta;
heading_error = wrapToPi(heading_error);
dock_distance = norm([pb.dock(2) - pb.state(2), pb.dock(1) - pb.state(1)]);

vx = (pb.state(1) - pb.state_old(1)) / time.dt;
vy = (pb.state(2) - pb.state_old(2)) / time.dt;

speed = vx * cos(pb.state(3)) + vy * sin(pb.state(3));

dock_head_error = wrapToPi(pb.dock(3) - pb.state(3));

% make sure we dont translate until we stopped rotating
thetaOld = pb.state_old(3);
dtheta = theta - thetaOld;
if ((theta - thetaOld) > pi)
    dtheta = (theta - thetaOld) - 2 * pi;
elseif ((theta - thetaOld) < -pi)
    dtheta = (theta - thetaOld) + 2 * pi;
end
omega = dtheta / time.dt;

if dock_distance < distanc_dock_error && abs(speed) < 20
    docking = true;
end
if ~docking
    if (abs(heading_error) > heading_dock_error)
        disp('WE TURNING \n')
        pb.heading_rad = heading;
        pb = pb_control_heading(pb, time);
    else 
         disp('WE MOVING \n')
        % disp(fwdspeed)
        pb = pb_control_spd(pb, time);
    end

else
    final_heading_dock_error = wrapToPi(theta - pb.dock(3));
    if (abs(final_heading_dock_error) > heading_dock_error)
        disp('WE turning at the end! \n')
        pb.heading_rad = pb.dock(3);
        pb = pb_control_heading(pb, time);
    else
        pb.forward_spd = 0;
        pb.done = true;
        return;
    end
end

end