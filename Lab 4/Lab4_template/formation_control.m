function [wr,t2,t3,disable] = formation_control(wr,t2,t3,time,disable)


%         setting the PWM limits
wr.PWML = min(150,  wr.PWML);
wr.PWML = uint8(max(-150,  wr.PWML));
wr.PWMR = min(150,  wr.PWMR);
wr.PWMR = uint8(max(-150,  wr.PWMR));

t2.PWML = min(150,  t2.PWML);
t2.PWML = uint8(max(-150,  t2.PWML));
t2.PWMR = min(150,  t2.PWMR);
t2.PWMR = uint8(max(-150,  t2.PWMR));

t3.PWML = min(150,  t3.PWML);
t3.PWML = uint8(max(-150,  t3.PWML));
t3.PWMR = min(150,  t3.PWMR);
t3.PWMR = uint8(max(-150,  t3.PWMR));