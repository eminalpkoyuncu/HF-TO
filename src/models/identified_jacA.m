function A = identified_jacA(xin,h,thetaHat)
% Jacobian of the discrete-time model in identified_step.m w.r.t. the state.
% thetaHat defaults to the identified Heron model (heron_identified_params.csv).
if nargin < 3
    thetaHat = identified_params;
end
th = thetaHat;

phi = xin(3);
u = xin(4);
v = xin(5);
r = xin(6);

A = zeros(6,6);

% kinematics, forward Euler
A(1,:) = [1, 0, -h*(u*sin(phi) + v*cos(phi)), h*cos(phi), -h*sin(phi), 0];
A(2,:) = [0, 1,  h*(u*cos(phi) - v*sin(phi)), h*sin(phi),  h*cos(phi), 0];
A(3,:) = [0, 0, 1, 0, 0, h];

% identified velocity dynamics, d(s*abs(s))/ds = 2*abs(s)
A(4,4:6) = [th(1) + 2*th(4)*abs(u), th(3)*r, 2*th(2)*r + th(3)*v];
A(5,4:6) = [th(8)*v + th(9)*r, th(6) + th(8)*u + 2*th(10)*abs(v), th(7) + th(9)*u + 2*th(11)*abs(r)];
A(6,4:6) = [th(15)*v + th(16)*r, th(13) + th(15)*u + 2*th(17)*abs(v), th(14) + th(16)*u + 2*th(18)*abs(r)];
end
