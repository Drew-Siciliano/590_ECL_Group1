function pb = pb_control_heading(pb, time)
    % Add your controller here.
    pb.VALF = 0;
    pb.DIRF = 0;
    pb.VALR = 0;
    pb.DIRR = 0;

    K = [2.0016    1.0000] * 5;
    % K = [1.4165    1.0000];

    % change in theta
    theta = pb.state(3);
    thetaprime = pb.heading_rad;

    deltatheta = theta - thetaprime;
    deltatheta = wrapToPi(deltatheta);

    % omega
    thetaOld = pb.state_old(3);

    dtheta = theta - thetaOld;
    dtheta = wrapToPi(dtheta);
    omega = dtheta / time.dt;

    % control input
    controlInput = - K * ([omega; deltatheta]);
    
    pb.VALR = min(abs(controlInput), 1);
    if (controlInput >= 0)
        pb.DIRR = 0;
    else
        pb.DIRR = 1;
    end
end