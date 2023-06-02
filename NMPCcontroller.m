function nlobj = NMPCcontroller(params)
    nlobj = nlmpc(params.x_dim, params.y_dim,params.u_dim);
    
    
    % Specify prediction and control horizons
    nlobj.Ts=params.Ts;
    nlobj.PredictionHorizon = params.pHorizon;
    nlobj.ControlHorizon = params.cHorizon;
    
    % Define model update function
    nlobj.Model.StateFcn = @(x, u,Ts) unicycleDT(x, u,params.Ts);
    nlobj.Model.OutputFcn = @(x,u,Ts) [x(1); x(2); x(3)];
    nlobj.Model.IsContinuousTime = false;

    % Define Jacobian of the model 
%  If you do not specify a Jacobian for a given function, 
%  the nonlinear programming solver must numerically compute 
%  the Jacobian.
% nlobj.Jacobian.StateFcn = @(x, u) ...
%     [1, 0, -Ts*u(1)*sin(x(3)), Ts*cos(x(3)), 0;...
%     0, 1, Ts*u(1)*cos(x(3)), Ts*sin(x(3)), 0;...
%     0, 0, 1, 0, Ts];
% nlobj.Jacobian.StateFcn = @(x, u) ...
%     [1, 0, -Ts*u(1)*sin(x(3));...
%     0, 1, Ts*u(1)*cos(x(3));...
%     0, 0, 1];

    % Define weights
    nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1];
    nlobj.Weights.OutputVariables = [1 1 0.5];
    
    % Define constraints for inputs
    nlobj.ManipulatedVariables(1).Min = -1; % Min linear velocity
    nlobj.ManipulatedVariables(1).Max = 3;  % Max linear velocity
    nlobj.ManipulatedVariables(2).Min = -pi/4; % Min angular velocity
    nlobj.ManipulatedVariables(2).Max = pi/4; % Max angular velocity


end