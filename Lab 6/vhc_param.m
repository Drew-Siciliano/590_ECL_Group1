%% white rover
wr.ID = 12;
wr.mode = 0;
wr.update = 0;
wr.disable = 0;

% Mode
wr.mode = 0;
% forward angle [deg]
wr.fwd_deg = 15;
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
wr.pos = [-1 0];
wr.pos_old = [0 0];

wr.pgain_h = 0;
wr.igain_h = 0;
wr.dgain_h = 0;
wr.pgain_v = 0;
wr.igain_v = 0;
wr.dgain_v = 0;

wr.forward_spd = 100;
wr.spd_avg = 0;
wr.filter_n = 10;
wr.v_buf = NaN(1, wr.filter_n);
wr.filter_init = 0;
wr.heading_dir = [0 0];
%% tank_2
t2.ID = 11;
t2.mode = 0;
t2.update = 0;
t2.disable = 0;
% forward angle [deg]
t2.fwd_deg = 15;
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
t2.pos = [0 0];
t2.pos_old = [0 0];

t2.pgain_h = 0;
t2.igain_h = 0;
t2.dgain_h = 0;
t2.pgain_v = 0;
t2.igain_v = 0;
t2.dgain_v = 0;

t2.forward_spd = 100;
t2.spd_avg = 0;
t2.filter_n = 10;
t2.v_buf = NaN(1, t2.filter_n);
t2.filter_init = 0;
t2.heading_dir = [0 0];
