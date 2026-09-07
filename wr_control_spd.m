function [wr] = wr_control_spd(wr, time)
   % default values
    wr.DIRL = 1;
    wr.DIRR = 1;

    Kp = 0.0496429912417118;
    Ki = 0.724718220063641;
    Kd = 0;

    speed_error = wr.forward_spd - norm((wr.pos - wr.pos_old) / time.dt);
    cum_speed_error_int = wr.cum_speed_error_int + speed_error * time.dt;
    speed_error_der = (speed_error - wr.speed_error_old) / time.dt;

    u_t = Kp * speed_error + Ki * cum_speed_error_int + Kd * speed_error_der;
    PWML = u_t;
    PWMR = u_t;

    wr.cum_speed_error_int = cum_speed_error_int;
    wr.speed_error_old = speed_error; 

    % setting the PWM limits, keep the codes here
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end