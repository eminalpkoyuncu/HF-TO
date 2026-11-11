function ctrl = with_current_estimator(ctrl,model,cfg)
% Adds a current estimate to a controller (ctrl_hfto or ctrl_mpcgraph):
% a Kalman filter on the constant current Vc (world frame, random walk)
% with the one-step prediction residual x_k - current_step(x_{k-1},u_{k-1},Vc)
% as measurement (Jacobian in Vc by finite differences). The estimate is
% passed to the inner controller as cs.Vc, which uses it in its prediction
% model. u_{k-1} is the command clipped to cfg.uLim (what the plant gets,
% up to the unknown thrust noise). For a model with a continuous form
% (Fossen), the residual uses an RK4 step of model.f instead of the Euler
% step of the MPC, so the discretization error is not read as a current.
% Settings: cfg.est.
E.model = model;
if ~isempty(model.f) && cfg.est.rk4
    E.model.step = @(x,u) rk4_step(model.f,x,u,model.h,4);
end
E.uLim = cfg.uLim;
E.Q = cfg.est.q*eye(2);
E.R = diag(cfg.est.r);
E.vMax = cfg.est.vMax;
inner = ctrl.step;
ctrl.name = [ctrl.name ' + current estimate'];
ctrl.cs.Vc = [0;0];
ctrl.cs.est = struct('P',cfg.est.p0*eye(2),'xprev',[],'uprev',[]);
ctrl.step = @(t,x,cs) est_step(t,x,cs,inner,E);
end

function [u,cs,info] = est_step(t,x,cs,inner,E)
es = cs.est;
if ~isempty(es.xprev)
    Vc = cs.Vc;
    P = es.P + E.Q;
    xp = current_step(E.model,es.xprev,es.uprev,Vc);
    d = 1e-4;
    J = zeros(6,2);
    for i = 1:2
        e = zeros(2,1);
        e(i) = d;
        J(:,i) = (current_step(E.model,es.xprev,es.uprev,Vc+e) - xp)/d;
    end
    y = x - xp;
    y(3) = atan2(sin(y(3)),cos(y(3)));
    S = J*P*J.' + E.R;
    K = P*J.'/S;
    Vc = Vc + K*y;
    if norm(Vc) > E.vMax
        Vc = Vc*E.vMax/norm(Vc);
    end
    cs.Vc = Vc;
    es.P = (eye(2) - K*J)*P;
end
[u,cs,info] = inner(t,x,cs);
es.xprev = x;
es.uprev = min(max(u,E.uLim(1)),E.uLim(2));
cs.est = es;
end

function x = rk4_step(f,x,u,h,n)
dt = h/n;
for i = 1:n
    k1 = f(x,u);
    k2 = f(x+dt/2*k1,u);
    k3 = f(x+dt/2*k2,u);
    k4 = f(x+dt*k3,u);
    x = x + dt/6*(k1+2*k2+2*k3+k4);
end
end
