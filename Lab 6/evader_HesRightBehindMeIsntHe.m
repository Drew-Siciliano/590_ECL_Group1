function t2 = evader_HesRightBehindMeIsntHe(wr,t2,time)
    L = 151.2;

    R = norm(wr.pos - t2.pos);

    dx = wr.pos(1) - t2.pos(1);
    dy = wr.pos(2) - t2.pos(2);

    chaser_angle = atan2(dy, dx);

    if (R > 650)
        target_heading = chaser_angle - pi/2;
        t2 = control_heading(t2, time, target_heading);
    else
        t2.PWMR = uint8(150); % vr_max = 579.1504
        vr_max = 579.1504;
        R_d = 500; % Radius around chaser
        vl_optimal = vr_max * (2*R/L - 1) / (2*R/L + 1); % Derived required vl

        if (R > R_d)
            t2.vl = vl_optimal * 0.5;
        elseif (R < R_d)
            t2.vl = vl_optimal * 1.5;
        else
            t2.vl = vl_optimal;
        end

        t2.PWML = polyval([1.3027e-7, -1.8165e-4, 0.3162, 2.4951], t2.vl);
        t2.DIRR = 1;
        t2.DIRL = 1;
    end