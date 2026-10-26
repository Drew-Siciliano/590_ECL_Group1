function pb = pb_control_heading(pb, time)
    % Add your controller here.
    pb.VALF = 0;
    pb.DIRF = 0;
    pb.VALR = 0;
    pb.DIRR = 0;

    K = [0.151452305080135	0.447213595499958];

    % change in theta
    theta = pb.state(3);
    thetaprime = pb.heading_rad;

    deltatheta = thetaprime - theta;
    if ((thetaprime - theta) > pi)
        deltatheta = (thetaprime - theta) - 2 * pi;
    elseif ((thetaprime - theta) < -pi)
        deltatheta = (thetaprime - theta) + 2 * pi;
    end

    % omega
    thetaOld = pb.state_old(3);

    dtheta = theta - thetaOld;
    if ((theta - thetaOld) > pi)
        dtheta = (theta - thetaOld) - 2 * pi;
    elseif ((theta - thetaOld) < -pi)
        dtheta = (theta - thetaOld) + 2 * pi;
    end
    omega = dtheta / time.dt;

    % control input
    controlInput = - K * ([omega; -deltatheta]);
    
    pb.VALR = min(abs(controlInput), 1);
    if (controlInput >= 0)
        pb.DIRR = 0;
    else
        pb.DIRR = 1;
    end
end
