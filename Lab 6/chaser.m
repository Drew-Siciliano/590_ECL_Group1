function wr = chaser(wr,t2,time)
    heading_rad = atan2(wr.heading_vec(2), wr.heading_vec(1));
    L = 151.2;

    vr = wr.old_vr;
    vl = wr.old_vl;

    if (vl + vr) == 0
        vr = 50;
        vl = 50;
    else
        if wr.time_since_gain >= 10
            % Linearized dynamics
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
            wr.old_K = K;
            wr.time_since_gain = 1;
        else
            K = wr.old_K;
            wr.time_since_gain = wr.time_since_gain + 1;
        end

        heading_evader_rad = atan2(t2.heading_vec(2), t2.heading_vec(1));
        error_state = [wr.pos(1); wr.pos(2); heading_rad] - [t2.pos(1); t2.pos(2); heading_evader_rad];
        error_state(3) = wrapToPi(heading_rad - heading_evader_rad);
        u = -K * error_state;
        vr = u(1);
        vl = u(2);
    end

    wr.PWML = 1.3027 * 10^(-7) * vl^3 - 1.8165 * 10^(-4) * vl^2 + 0.3162 * vl + 2.4951;
    wr.PWMR = 1.3027 * 10^(-7) * vr^3 - 1.8165 * 10^(-4) * vr^2 + 0.3162 * vr + 2.4951;

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
    wr.PWML = abs(wr.PWML);
    wr.PWMR = abs(wr.PWMR);
    %   setting the PWM limits
    wr.PWML = min(150,  wr.PWML);
    wr.PWML = uint8(max(0,  wr.PWML));
    wr.PWMR = min(150,  wr.PWMR);
    wr.PWMR = uint8(max(0,  wr.PWMR));
    
    % Slow down if gonna go outside bounds
    bounds = [400, 400; 3200, 400; 4400, 1300; 4400, 2200; 400, 2200];
    if vl == 0 && vr == 0
        valid_control = true; % Don't need to check if stationary
    else
        valid_control = false;
    end
    iterations = 0;
    while ~valid_control && iterations < 1000
        if wr.PWMR == 0
            vr = 0;
        else
            pwmr_cubic = [1.3027*10^(-7); -1.8165*10^(-4); 0.3162; 2.4951-double(wr.PWMR)];
            vr = roots(pwmr_cubic); 
            vr = vr(imag(vr) == 0);
            if wr.DIRR == 0 
                vr = -vr;
            end
        end
        if wr.PWML == 0
            vl = 0;
        else
            pwml_cubic = [1.3027*10^-7; -1.8165*10^-4; 0.3162; 2.4951-double(wr.PWML)];
            vl = roots(pwml_cubic); 
            vl = vl(imag(vl) == 0);
            if wr.DIRL == 0 
                vl = -vl;
            end
        end

        future_state = [wr.pos(1); wr.pos(2)] + [((vr+vl)/2) * cos(heading_rad); ((vr+vl)/2) * sin(heading_rad)] * time.dt * 1.25; % 1.25 for margin
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

    %   setting the PWM limits
    wr.PWML = min(150,  wr.PWML);
    wr.PWML = uint8(max(0,  wr.PWML));
    wr.PWMR = min(150,  wr.PWMR);
    wr.PWMR = uint8(max(0,  wr.PWMR));

    wr.old_vr = vr;
    wr.old_vl = vl;
end