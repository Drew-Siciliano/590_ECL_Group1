function x = control_spd(x, time)
   % default values
    x.DIRL = 1;
    x.DIRR = 1;

    Kp = 0.0496429912417118;
    % Ki = 0.724718220063641;
    Ki = 0;
    Kd = 0;

    speed_error = x.forward_spd - norm((x.pos - x.pos_old) / time.dt);
    cum_speed_error_int = x.cum_speed_error_int + speed_error * time.dt;
    speed_error_der = (speed_error - x.speed_error_old) / time.dt;

    u_t = Kp * speed_error + Ki * cum_speed_error_int + Kd * speed_error_der;
    x.PWML = u_t;
    x.PWMR = u_t;

    x.cum_speed_error_int = cum_speed_error_int;
    x.speed_error_old = speed_error; 
end