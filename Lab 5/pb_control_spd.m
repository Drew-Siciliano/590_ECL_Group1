function pb = pb_control_spd(pb, time)
    % Add your controller here.
    pb.VALF = 0;
    pb.DIRF = 0;
    pb.VALR = 0;
    pb.DIRR = 0;
    if (pb.mode1)
        K = [0.5000    0.0000];
        x_desired = [pb.forward_spd; 0];
    else
        K = [3.5382    0.5000];
        x_desired = [0; 0];
    end

    % calculate current state and desired state
    vx = (pb.state(1) - pb.state_old(1)) / time.dt;
    vy = (pb.state(2) - pb.state_old(2)) / time.dt;
    
    speed = vx * cos(pb.state(3)) + vy * sin(pb.state(3));
    distance = norm([pb.state(1); pb.state(2)]);

    x_current = [speed; distance];


    controlInput = K * (x_current - x_desired);

    % control input
    pb.VALF = min(abs(controlInput), 1);
    if (controlInput >= 0)
        pb.DIRF = 0;
    else
        pb.DIRF = 1;
    end
end
