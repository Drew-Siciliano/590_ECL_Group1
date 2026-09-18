function xdot=deriv(t,x,u)
% equations of motion
% input
% t = current time
% x = current state vector
% u = current control vector

% output
% xdot = derivative of x and y
global x02 

% evaluate equations of motion at current conditions
xdot(1) = 0.1*cos(x(3));
xdot(2) = 0.1*sin(x(3));
xdot(3) = u;
