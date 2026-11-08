function t2 = evader(wr,t2,time)
    heading_rad = atan2(t2.heading_vec(2), t2.heading_vec(1));
    L = 151.2;

    vr = t2.old_vr;
    vl = t2.old_vl;

    if (vl + vr) == 0
        vr = 50;
        vl = 50;
    else
        if t2.time_since_gain >= 10
            avg_wheel_speed = (vr+vl)/2;
            A = [0, 0, -avg_wheel_speed * sin(heading_rad);
                 0, 0, avg_wheel_speed * cos(heading_rad);
                 0, 0, 0];
            B = [0.5 * cos(heading_rad), 0.5 * cos(heading_rad);
                 0.5 * sin(heading_rad), 0.5 * sin(heading_rad);
                 1/L, -1/L];
        
            Q = diag([1, 1, 1e-3]);
            R = diag([1e-2, 1e-2]);
    
            % How slow is recomputing gains? :)
            tic
            K = lqr(A, B, Q, R);
            toc
            t2.old_K = K;
            t2.time_since_gain = 1;
        else
            K = t2.old_K;
            t2.time_since_gain = t2.time_since_gain + 1;
        end

        % Match pursuer heading so move away
        heading_chaser_rad = atan2(wr.heading_vec(2), wr.heading_vec(1));
        error_state = [wr.pos(1); wr.pos(2); heading_rad] - [t2.pos(1); t2.pos(2); heading_chaser_rad];
        error_state(3) = wrapToPi(heading_rad - heading_chaser_rad);

        u = -K * error_state;
        vr = u(1);
        vl = u(2);
    end

    t2.PWML = 1.3027 * 10^(-7) * vl^3 - 1.8165 * 10^(-4) * vl^2 + 0.3162 * vl + 2.4951;
    t2.PWMR = 1.3027 * 10^(-7) * vr^3 - 1.8165 * 10^(-4) * vr^2 + 0.3162 * vr + 2.4951;

    if t2.PWML < 0
        t2.DIRL = 0;
    else
        t2.DIRL = 1;
    end
    if t2.PWMR < 0
        t2.DIRR = 0;
    else
        t2.DIRR = 1;
    end
    t2.PWML = abs(t2.PWML);
    t2.PWMR = abs(t2.PWMR);
    %   setting the PWM limits
    t2.PWML = min(150,  t2.PWML);
    t2.PWML = uint8(max(0,  t2.PWML));
    t2.PWMR = min(150,  t2.PWMR);
    t2.PWMR = uint8(max(0,  t2.PWMR));

    % Turn if gonna go outside bounds
    bounds = [400, 400; 3200, 400; 4400, 1300; 4400, 2200; 400, 2200];

    if t2.PWMR == 0
        vr = 0;
    else
        pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(t2.PWMR)];
        vr = roots(pwmr_cubic); 
        vr = vr(imag(vr) == 0);
        if t2.DIRR == 0
            vr = -vr;
        end
    end
    if t2.PWML == 0
        vl = 0;
    else
        pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(t2.PWML)];
        vl = roots(pwml_cubic); 
        vl = vl(imag(vl) == 0);
        if t2.DIRL == 0 
            vl = -vl;
        end
    end
    
    future_state = [t2.pos(1); t2.pos(2)] + [((vr+vl)/2) * cos(heading_rad); ((vr+vl)/2) * sin(heading_rad)] * time.dt * 1.25; % 1.25 for margin
    if ~inpolygon(future_state(1), future_state(2), bounds(:, 1), bounds(:, 2))
        t2.PWML = uint8(50);
        t2.PWMR = uint8(50);
        t2.DIRL = 0;
        t2.DIRR = 1;
    end

    t2.PWMR = uint8(max(0,  t2.PWMR));

    pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(t2.PWMR)];
    vr = roots(pwmr_cubic); 
    vr = vr(imag(vr) == 0);
    if t2.DIRR == 0
        vr = -vr;
    end
    pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(t2.PWML)];
    vl = roots(pwml_cubic); 
    vl = vl(imag(vl) == 0);
    if t2.DIRL == 0 
        vl = -vl;
    end
    t2.old_vr = vr;
    t2.old_vl = vl;
end
