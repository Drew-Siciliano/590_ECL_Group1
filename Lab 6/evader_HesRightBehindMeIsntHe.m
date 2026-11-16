function t2 = evader_HesRightBehindMeIsntHe(wr,t2,time)
    L = 151.2;

    if (norm(wr.pos - t2.pos) > 500)
        target_heading = t2.chaser_heading + pi/2;
        t2 = control_heading(t2, time, target_heading);
    else
        t2.PWMR = uint8(150); % vr_max = 579.1504
        vr_max = 579.1504;
        R = 400; % Radius around chaser
        t2.vl = vr_max * (2*R/L - 1) / (2*R/L + 1); % Derived required vl

        t2.PWML = polyval([1.3027e-7, -1.8165e-4, 0.3162, 2.4951], t2.vl);
        t2.DIRR = 1;
        t2.DIRL = 1;
    end