clear all
close all
% initial relative condition
x0 = [-1 0 1]';
y0 = [0 0 0]';

% final relative condition
xf = [-0.5 0 0.5]';
yf = [-0.5 sqrt(3)/2-0.5 -0.5]';

% update time interval
dt = 0.01;

% Laplacian Matrix
L = [2, -1, -1; ...
    -1, 2, -1; ...
    -1, -1, 2];

% Define the displacement vector \tau
tau = [(x0-xf);(y0-yf)]';
error(1) = 1;
error(2) = rand*0.1;
for i=1:300
    if abs(error(end)-error(end-1))>0.001 %Define the convergence criterion
        Xi(:,i)=tau+[xf;yf]';
        error(i)=std(tau(1:3))+std(tau(3:end));
        % run the Consensus Protocol 
        tau_dot = -[L,zeros(3,3);zeros(3,3),L]*tau';
        % update the states tau using RK44 integration
        ti(i) = dt*i;
        k1=dt*tau_dot;
        k2=dt*(-[L,zeros(3,3);zeros(3,3),L]*(tau'+k1/2));
        k3=dt*(-[L,zeros(3,3);zeros(3,3),L]*(tau'+k2/2));
        k4=dt*(-[L,zeros(3,3);zeros(3,3),L]*(tau'+k3));
        tau=tau+1/6*(k1'+2*k2'+2*k3'+k4');
    end
end
% Plot the trajectory
for i=1:3
    plot(Xi(i,1:end),Xi(i+3,1:end),'*');
    hold on
end
