%% white rover
wr.ID = 1;
wr.mode = 0;
wr.update = 0;
wr.disable = 0;
wr.error_old =0;

% Mode
wr.mode = 0;
% forward angle [deg]
wr.fwd_deg = 10;
% distance marin [mm]
wr.dist_mar = 100;
wr.curWP = 1;
% PID gain
wr.spd = 0;
wr.espd = 0;
wr.dangle_cum = 0;
wr.dangle_old = 0;
wr.espd_old = 0;
wr.espd_cum = 0;
wr.pos = [0 0];
wr.pos_old = [0 0];

wr.pgain_h = 8;
wr.igain_h = 0.05;
wr.dgain_h = 2;
wr.pgain_v = 0.15;
wr.igain_v = 0.7;
wr.dgain_v = 0;

wr.forward_spd = 100;
wr.spd_avg = 0;
wr.filter_n = 10;
wr.v_buf = NaN(1, wr.filter_n);
wr.filter_init = 0;
%% tank_2
t2.ID = 2;
t2.mode = 0;
t2.update = 0;
t2.disable = 0;
% forward angle [deg]
t2.fwd_deg = 10;
% distance marin [mm]
t2.dist_mar = 100;
t2.curWP = 1;
% PID gain
t2.spd = 0;
t2.espd = 0;
t2.dangle_cum = 0;
t2.dangle_old = 0;
t2.espd_old = 0;
t2.espd_cum = 0;
t2.pos_old = [0 0 0];

t2.pgain_h = 10;
t2.igain_h = 0.1;
t2.dgain_h = 5;
t2.pgain_v = 3.5;
t2.igain_v = 0;
t2.dgain_v = 0.15;

t2.forward_spd = 100;

%% tank_3
t3.ID = 3;
t3.mode = 0;
t3.update = 0;
t3.disable = 0;
% forward angle [deg]
t3.fwd_deg = 10;
% distance marin [mm]
t3.dist_mar = 100;
t3.curWP = 1;
% PID gain
t3.spd = 0;
t3.espd = 0;
t3.dangle_cum = 0;
t3.dangle_old = 0;
t3.espd_old = 0;
t3.espd_cum = 0;
t3.pos_old = [0 0 0];

t3.pgain_h = 1;
t3.igain_h = 0.6;
t3.dgain_h = 0.2;
t3.pgain_v = 3.5;
t3.igain_v = 0;
t3.dgain_v = 0.15;

t3.forward_spd = 100;

%% blimp_1
b1.ID = 5;
b1.mode = 0;
b1.DIRL = 0;
b1.DIRR =0;
b1.PWMV = 0;
b1.PWML = 0;
b1.PWMR = 0;
b1.update = 0;
b1.disable = 0;
% forward angle [deg]
b1.fwd_deg = 45;
% distance marin [mm]
b1.dist_mar = 800;
b1.curWP = 1;
% PID gain
b1.spd = 0;
b1.espd = 0;
b1.vert_distance = 0;
b1.vert_distance_old = 0;
b1.vert_distance_cum = 0;
b1.dangle_cum = 0;
b1.dangle_old = 0;
b1.espd_old = 0;
b1.espd_cum = 0;
b1.pos_old = [0 0 0];

b1.pgain_h = 3.5;
b1.igain_h = 0.20;
b1.dgain_h = 9.0;
b1.pgain_v = 0.01;
b1.igain_v = 0.0060;
b1.dgain_v = 200.0;
b1.pgain_vert = 0.27;
b1.igain_vert = 0.035;
b1.dgain_vert = 0.38;
b1.spinDir = 0;

b1.forward_spd = 100;