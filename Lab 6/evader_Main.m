function t2 = evader_Main(wr,t2,time)
    L = 151.2;

    if (t2.vl + t2.vr) == 0
        t2.vr = 50;
        t2.vl = 50;
    else
        if t2.time_since_gain >= 10
            avg_wheel_speed = (t2.vr+t2.vl)/2;
            A = [0, 0, -avg_wheel_speed * sin(t2.heading);
                 0, 0, avg_wheel_speed * cos(t2.heading);
                 0, 0, 0];
            B = [0.5 * cos(t2.heading), 0.5 * cos(t2.heading);
                 0.5 * sin(t2.heading), 0.5 * sin(t2.heading);
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
        
        error_state = [wr.pos(1); wr.pos(2); t2.heading] - [t2.pos(1); t2.pos(2); t2.chaser_heading];
        error_state(3) = wrapToPi(t2.heading - t2.chaser_heading);

        u = -K * error_state;
        t2.vr = u(1);
        t2.vl = u(2);
    end

    t2.PWML = 1.3027 * 10^(-7) * t2.vl^3 - 1.8165 * 10^(-4) * t2.vl^2 + 0.3162 * t2.vl + 2.4951;
    t2.PWMR = 1.3027 * 10^(-7) * t2.vr^3 - 1.8165 * 10^(-4) * t2.vr^2 + 0.3162 * t2.vr + 2.4951;

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
        t2.vr = 0;
    else
        pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(t2.PWMR)];
        t2.vr = roots(pwmr_cubic); 
        t2.vr = t2.vr(imag(t2.vr) == 0);
        if t2.DIRR == 0
            t2.vr = -t2.vr;
        end
    end
    if t2.PWML == 0
        t2.vl = 0;
    else
        pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(t2.PWML)];
        t2.vl = roots(pwml_cubic); 
        t2.vl = t2.vl(imag(t2.vl) == 0);
        if t2.DIRL == 0 
            t2.vl = -t2.vl;
        end
    end
    
    future_state = [t2.pos(1); t2.pos(2)] + [((t2.vr+t2.vl)/2) * cos(t2.heading); ((t2.vr+t2.vl)/2) * sin(t2.heading)] * time.dt * 100; % 1.25 for margin
    if ~inpolygon(future_state(1), future_state(2), bounds(:, 1), bounds(:, 2))
        t2.PWML = (150);
        t2.PWMR = (150);
        t2.DIRL = 0;
        t2.DIRR = 1;
        disp("GOING OUT")
    end

    % t2.PWMR = uint8(max(0,  t2.PWMR));

    pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(t2.PWMR)];
    t2.vr = roots(pwmr_cubic); 
    t2.vr = t2.vr(imag(t2.vr) == 0);
    if t2.DIRR == 0
        t2.vr = -t2.vr;
    end
    pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(t2.PWML)];
    t2.vl = roots(pwml_cubic); 
    t2.vl = t2.vl(imag(t2.vl) == 0);
    if t2.DIRL == 0 
        t2.vl = -t2.vl;
    end
    t2.old_t2.vr = t2.vr;
    t2.old_t2.vl = t2.vl;
end
