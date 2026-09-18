% main function
clear all
close all
Name = 'path planning problem';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
global nc_defect ndiffeq nnodes nlp_state ncv nlpv nc_ine x_0 xf2 xf1 x01 x02
% number of differential equations 
ndiffeq = 3; 
% number of control variables 
ncv = 1;
% number of discretization nodes 
nnodes = 20; 
% number of state nlp variables 
nlp_state = ndiffeq * nnodes; 
% number of control nlp variables 
nlp_control = ncv * nnodes; 
% total number of nlp variables 
nlpv = nlp_state + nlp_control; 
% number of state vector defect equality constraints 
nc_defect = nlp_state - ndiffeq; 
% number of auxilary equality constraints (boundary conditions) 
nc_aux = 4; 
% total number of equality constraints 
nc_total = nc_defect + nc_aux;
% total number of inequality constraints
nc_ine = nnodes-1;

% Set the starting and ending points
x01 = -1.2;  % starting point
x02 = 0;
xf1 = 1.8;  % ending point
xf2 = 0;
% set the initial guess/input of NLP variables
u_guess=rand*pi;
for i=1:1:nnodes
    x_0(3*i-2)=x01+(xf1-x01)/(nnodes-1)*(i-1);
    x_0(3*i-1)=x02+(xf2-x02)/(nnodes-1)*(i-1);
    x_0(3*i) = pi-4*rand;
    x_0(nlp_state+i)=rand;
    lb(3*i-2)=-2.1;
    lb(3*i-1)=-2;
    lb(3*i)=-pi;
    ub(3*i-2)=2.1;
    ub(3*i-1)=2;
    ub(3*i)=pi;
    lb(nlp_state+i)=-0.5;
    ub(nlp_state+i)=0.5;
end
x_0(nlpv+1)=50;
lb(nlpv+1)=5;
ub(nlpv+1)=100;
%opts = optimoptions('fmincon','Algorithm','sqp');
%opts = optimoptions(opts,'MaxIterations',4e4);
%opts = optimoptions('fmincon','MaxFunctionEvaluations',4e4);
%options.MaxFunctionEvaluations = 3.000000e+03
opts = optimoptions('fmincon','MaxFunctionEvaluations',1e7,'Algorithm','sqp', ...
    'MaxIterations',1e3,'FunctionTolerance',1e-7);
[X_OUT,FVAL,EXITFLAG,OUTPUT] = fmincon('trapm3_f',x_0,[],[],[],[],lb,ub,'trapm3_c',opts);
% For the inputs of the fmincon function, 'trapm3_f' is the objective
% function to be minimize and 'trapm3_c' include all constraints
for i=1:1:nnodes
    yff(i)=X_OUT(3*i-1);
    xff(i)=X_OUT(3*i-2);
    thetaff(i)=X_OUT(3*i);
    uff(i)=X_OUT(nlp_state+i);
end
xff=xff'*1000;
yff=yff'*1000;
figure (1)
plot(yff,xff)
t= 0:0.01:2*pi;
x_circle = sin(t)*0.3*1000;
y_circle = cos(t)*0.3*1000;
hold on
plot(x_circle, y_circle,'r')
axis equal