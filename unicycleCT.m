function dxdt = unicycleCT(x, u)
%% Continuous-time nonlinear dynamic model of a unicycle model on a cart
%
% 3 states (x = [p_x p_y theta]): 
%   car position (p_x p_y)
%   car angle (theta)
% 
% 2 inputs: (u = [v omega])

dxdt = [u(1)*cos(x(3));...
        u(1)*sin(x(3));...
        u(2)];
end