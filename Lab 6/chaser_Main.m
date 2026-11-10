function wr = chaser_Main(wr,t2,time)
    L = 151.2;

    if (wr.vl + wr.vr) == 0
        wr.vr = 50;
        wr.vl = 50;
    else
        if wr.time_since_gain >= 10
            % Linearized dynamics
            avg_wheel_speed = (wr.vr+wr.vl)/2;
            A = [0, 0, -avg_wheel_speed * sin(wr.heading);
                 0, 0, avg_wheel_speed * cos(wr.heading);
                 0, 0, 0];
            B = [0.5 * cos(wr.heading), 0.5 * cos(wr.heading);
                 0.5 * sin(wr.heading), 0.5 * sin(wr.heading);
                 1/L, -1/L];
        
            Q = diag([1, 1, 1e-3]);
            R = diag([1e-2, 1e-2]);
        
            % How slow is recomputing gains? :)
            tic
            K = lqr(A, B, Q, R);
            toc
            wr.old_K = K;
            wr.time_since_gain = 1;
        else
            K = wr.old_K;
            wr.time_since_gain = wr.time_since_gain + 1;
        end

        error_state = [wr.pos(1); wr.pos(2); wr.heading] - [t2.pos(1); t2.pos(2); wr.evader_heading];
        error_state(3) = wrapToPi(wr.heading - wr.evader_heading);
        u = -K * error_state;
        wr.vr = u(1);
        wr.vl = u(2);
    end

    wr.PWML = 1.3027 * 10^(-7) * wr.vl^3 - 1.8165 * 10^(-4) * wr.vl^2 + 0.3162 * wr.vl + 2.4951;
    wr.PWMR = 1.3027 * 10^(-7) * wr.vr^3 - 1.8165 * 10^(-4) * wr.vr^2 + 0.3162 * wr.vr + 2.4951;

    if wr.PWML < 0
        wr.DIRL = 0;
    else
        wr.DIRL = 1;
    end
    if wr.PWMR < 0
        wr.DIRR = 0;
    else
        wr.DIRR = 1;
    end
    
    % Slow down if gonna go outside bounds
    bounds = [400, 400; 3200, 400; 4400, 1300; 4400, 2200; 400, 2200];
    if wr.vl == 0 && wr.vr == 0
        valid_control = true; % Don't need to check if stationary
    else
        valid_control = false;
    end
    iterations = 0;
    while ~valid_control && iterations < 1000
        if wr.PWMR == 0
            wr.vr = 0;
        else
            pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(wr.PWMR)];
            wr.vr = roots(pwmr_cubic); 
            wr.vr = wr.vr(imag(wr.vr) == 0);
            if wr.DIRR == 0 
                wr.vr = -wr.vr;
            end
        end
        if wr.PWML == 0
            wr.vl = 0;
        else
            pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(wr.PWML)];
            wr.vl = roots(pwml_cubic); 
            wr.vl = wr.vl(imag(wr.vl) == 0);
            if wr.DIRL == 0 
                wr.vl = -wr.vl;
            end
        end

        future_state = [wr.pos(1); wr.pos(2)] + [((wr.vr+wr.vl)/2) * cos(wr.heading); ((wr.vr+wr.vl)/2) * sin(wr.heading)] * time.dt * 10; % 1.25 for margin
        if inpolygon(future_state(1), future_state(2), bounds(:, 1), bounds(:, 2))
            valid_control = true;
        else
            wr.PWMR = double(wr.PWMR)/2;
            wr.PWML = double(wr.PWML)/2;
            iterations = iterations + 1;
        end
    end
    if ~valid_control
        wr.PWMR = 0;
        wr.PWML = 0;
    end
end