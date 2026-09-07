function [wr] = wr_control_heading(wr, time)
   % defalut values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML =0;
    PWMR =0;

    plant = tf([0.4972 0.1716 0.3878 0.1714], [1 0.7091 1.551 0.4573 0.5846]);
    pid = pidTuner(plant, 'PID');

    % setting the PWM limits, keep the codes here
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end