function xk1 = unicycleDT(xk, uk,Ts)
M=10;
delta = Ts/M;
xk1 = xk;
% Repeat application of Euler method sampled at Ts/M.
for ct=1:M
    xk1 = xk1 + delta*unicycleCT(xk1,uk);
end
xk1 = xk;

        % xk1 = xk1 + delta*[xk1(1) + Ts*uk(1)*cos(xk1(3));...
        %         xk1(2) + Ts*uk(1)*sin(xk1(3));...
        %         xk1(3) + Ts*uk(2)];
end