function [A, B] = unicycleJacobianFcn(x,u)
% Jacobian of unicycle model.
% 3 States:
% x(1) x car position
% x(2) y car position
% x(3) theta, head direction
% 
% 2 Inputs: (u = [v omega])
% u(1) veloicty
% u(2) angle velocity
% 
% RETURN
% A = Jacobian of the state function with respect to x,
% B = Jacobian of the state function output with respect to u
% A = [1, 0, -Ts*u(1)*sin(x(3));...
%     0, 1, Ts*u(1)*cos(x(3));...
%     0, 0, 1];
% B = [Ts*cos(x(3)), 0;...
%     Ts*sin(x(3)), 0;...
%     0, Ts];


% dxdt = [u(1)*cos(x(3));...
%         u(1)*sin(x(3));...
%         u(2)];

A = [0, 0, -u(1)*sin(x(3));
     0, 0, u(1)*cos(x(3));
     0, 0, 0];

B = [cos(x(3)), 0;...
    sin(x(3)), 0;...
    0, 1];

end