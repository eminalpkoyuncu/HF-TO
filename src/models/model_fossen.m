function model = model_fossen(h,scale)
% Known 3-DoF Fossen USV model used in the Monte Carlo experiments:
%   eta_dot = R(psi)*nu,  M*nu_dot + C(nu)*nu + D(nu)*nu = T*[F1;F2]
% Parameters of paper Table (fossen) (hydrodynamic derivatives are negative
% by convention, so M = M_RB - diag(derivatives) and D = -damping_matrix(...)).
% TO and MPC use the forward-Euler discretization with step h; the plant is
% the continuous model f, integrated with ode45. scale (optional, 14x1)
% multiplies [m Iz Xu Yv Yr Nv Nr Xudot Yvdot Yrdot Nrdot absX absY absN]
% (plant parameter mismatch, make_plant.m).
m = 10;
Iz = 0.6487;
Xu = -5.6128;
Yv = -3.5059;
Yr = -0.0001;
Nv = -0.0001;
Nr = 0;
Xudot = -1.0946;
Yvdot = -1.0536;
Yrdot = -1.4353;
Nrdot = -0.1039;
absX = -2.3136;
absY = -3.9310;
absN = -0.2605;
b = 0.26;
if nargin > 1
    p = num2cell([m Iz Xu Yv Yr Nv Nr Xudot Yvdot Yrdot Nrdot absX absY absN].*scale(:).');
    [m,Iz,Xu,Yv,Yr,Nv,Nr,Xudot,Yvdot,Yrdot,Nrdot,absX,absY,absN] = p{:};
end

inertiaVec = [m; Iz];
dotVec = [Xudot; Yvdot; Yrdot; Nrdot];
dampVec = [Xu; Yv; Yr; Nv; Nr];
absVec = [absX; absY; absN];

M = [m-Xudot 0 0; 0 m-Yvdot -Yrdot; 0 -Yrdot Iz-Nrdot];
T = [1 1; 0 0; b -b];
Minv = inv(M);

f = @(x,u) [kinematics(x(1:3),x(4:6)); ...
            Minv*(T*u(:) - coriolis_matrix(x(4:6),inertiaVec,dotVec)*x(4:6) + damping_matrix(x(4:6),dampVec,absVec)*x(4:6))];

model.name = 'fossen';
model.h = h;
model.b = b;
model.f = f;
model.step = @(x,u) x(:) + h*f(x(:),u);
% f is affine in the input, so A does not depend on u
model.jacA = @(x) eye(6) + h*numjac_x(@(z) f(z,[0;0]),x(:));
model.B = h*[zeros(3,2); Minv*T];
end

function J = numjac_x(fun,x)
d = 1e-6;
J = zeros(numel(x));
for i = 1:numel(x)
    e = zeros(numel(x),1);
    e(i) = d;
    J(:,i) = (fun(x+e)-fun(x-e))/(2*d);
end
end
