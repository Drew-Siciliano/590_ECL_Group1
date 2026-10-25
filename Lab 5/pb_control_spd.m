function pb = pb_control_spd(pb, time)
    % Add your controller here.
    pb.VALF = 0;
    pb.DIRF = 0;
    pb.VALR = 0;
    pb.DIRR = 0;
    
    K = [0.577350459509541	0.000005773502692];

    % calculate current state and desired state
    vx = (pb.state(1) - pb.state_old(1)) / time.dt;
    vy = (pb.state(2) - pb.state_old(2)) / time.dt;

    speed = vx * cos(pb.state(3)) + vy * sin(pb.state(3));
    distance = norm([pb.state(1); pb.state(2)]);

    x_current = [speed; distance];
    x_desired = [pb.forward_spd; 0];

    controlInput = -K * (x_current - x_desired);

    % control input
    pb.VALF = min(abs(controlInput), 1);
    if (controlInput >= 0)
        pb.DIRF = 0;
    else
        pb.DIRF = 1;
    end
end
