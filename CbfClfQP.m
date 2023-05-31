function [u, slack, B, V, clf,cbf, comp_time] = CbfClfQP(cbfparams,position,params,p_e)
%% Implementation of vanilla CBF-CLF-QP
% Inputs:   x: state
%           u_ref: reference control input
%           with_slack: flag for relaxing the clf constraint(1: relax, 0: hard-constraint)
%           verbose: flag for logging (1: print log, 0: run silently)
% Outputs:  u: control input as a solution of the CBF-CLF-QP
%           slack: slack variable for relaxation. (empty list when with_slack=0)
%           B: Value of the CBF at current state.
%           V: Value of the CLF at current state.
%           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
%           when qp is infeasible, u is determined from quadprog.)
%           comp_time: computation time to run the solver.

tstart = tic;
n = size(cbfparams,1);
% CLF
% p_e = params.p_d_t'-position;
u_ref = params.pid_p*p_e;
V = params.pV*sum(p_e.^2);%+qV*(v_e(1)^2+ v_e(2)^2 + v_e(3)^2);
Vdot = -2*params.pV*p_e; LgV = Vdot;
B=0;LgB=0;
if size(cbfparams,1) ~= 0 % if obstacles are detected
    % CLF-CBF
    x_xo = position(1)*ones(n,1) - cbfparams(:,1);
    y_yo = position(2)*ones(n,1) - cbfparams(:,2);
    a = cbfparams(:,4) + (params.safety_distance)*ones(n,1);
    b = cbfparams(:,5) + (params.safety_distance)*ones(n,1);
    theta = cbfparams(:,9);
%     trans = eul2rotm(cbfparams(:,7:9),'XYZ');
%     if n == 1
%         R = [trans(1,:) trans(2,:) trans(3,:)];
%     else
%         R = zeros(n,9);
%         for i =1 :n
%             R(i,:)=[trans(1,:,i) trans(2,:,i) trans(3,:,i)];
%         end
%     end
    B = ((x_xo.*cos(-cbfparams(:,9))+y_yo.*sin(-cbfparams(:,9)))./a).^4+...
        ((-x_xo.*sin(-cbfparams(:,9))+y_yo.*cos(-cbfparams(:,9)))./b).^4 - ones(n,1);

    %         h = (((R(:,1).*(x_xo) + R(:,2).*(y_yo) + R(:,3).*(z_zo))./a).^4+...
    %             ((R(:,4).*(x_xo) + R(:,5).*(y_yo) + R(:,6).*(z_zo))./b).^4 - ones(n,1));
    %         LgB =  [(4*(position(1)*ones(n,1) - cbfparams(:,1)).^3)./cbfparams(:,4).^4,...
    %             (4*(position(2)*ones(n,1) - cbfparams(:,2)).^3)./cbfparams(:,4).^4];
    LgB = zeros(n,2);
    LgB(:,1) =(4.*cos(theta).*(cos(theta).*x_xo + sin(theta).*y_yo).^3)./(a.^4) ...
        -(4.*sin(theta).*(cos(theta).*y_yo - sin(theta).*x_xo).^3)./(b.^4);
    LgB(:,2) =(4.*sin(theta).*(cos(theta).*x_xo + sin(theta).*y_yo).^3)./(a.^4) ...
        +(4.*cos(theta).*(cos(theta).*y_yo - sin(theta).*x_xo).^3)./(b.^4);

    A = zeros(9+n,5);
    A(1,:) = [LgV -1 V 0];        % CLF
    A(2:3,1:2) = eye(2);        % u <= vmax
    A(4:5,1:2) = -1 .* eye(2);  % -u <= -vmin
    A(6,4) = 1;                 % clf_decay_rate <= clf_max
    A(7,4) = -1;                % - clf_decay_rate <= -clf_min
    A(8,5) = 1;                 % cbf_decay_rate <= cbf_max
    A(9,5) = -1;                % - cbf_decay_rate <= -cbf_min
    A(10:end,:) = [-LgB zeros(n,2) -B];   % CBF
    %         A(12:end,:) = [-LgH zeros(n_obstable,2) -H];   % CBF

    b = [0; params.u_max.*ones(params.u_dim,1); -params.u_min.*ones(params.u_dim,1);...
        params.clf.rate_max; -params.clf.rate_min;...
        params.cbf.rate_max;-params.cbf.rate_min;
        zeros(n,1)];

    H = diag([ones(1,params.u_dim) params.m 0 0]);
    f = [-u_ref 0 0 0]';

    x0=[zeros(1,params.u_dim) 0 1 1];
    Aeq = [];
    beq = [];

else
    % clf
    A = zeros(9,5);
    A(1,:) = [LgV -1 V 0];        % CLF
    A(2:3,1:2) = eye(2);        % u <= vmax
    A(4:5,1:2) = -1 .* eye(2);  % -u <= -vmin
    A(6,4) = 1;                 % clf_decay_rate <= clf_max
    A(7,4) = -1;                % - clf_decay_rate <= -clf_min
    A(8,5) = 1;                 % cbf_decay_rate <= cbf_max
    A(9,5) = -1;                % - cbf_decay_rate <= -cbf_min

    b = [0; params.u_max.*ones(params.u_dim,1); -params.u_min.*ones(params.u_dim,1);...
        params.clf.rate_max; -params.clf.rate_min;...
        params.cbf.rate_max;-params.cbf.rate_min];

    H = diag([ones(1,params.u_dim) params.m 0 0]);
    f = [-u_ref 0 0 0]';

    x0=[zeros(1,params.u_dim) 0 1 1];
    Aeq = [];
    beq = [];

end
% QP A * x <= b
% options = optimoptions('quadprog','Algorithm','active-set','Display','off');
options = optimoptions('quadprog','Algorithm','interior-point-convex');
% options =  optimset('Display','off');
[out,~,exitflag,~,~] = quadprog(H,f,A,b,Aeq,beq,[],[],x0,options);
% if exitflag < 0
%     exitflag
% end
exitflag
u = [out(1) out(2)]; slack=out(3);clf = out(4);cbf=out(5);
comp_time = toc(tstart);
end

