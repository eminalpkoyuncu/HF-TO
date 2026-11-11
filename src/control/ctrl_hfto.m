function ctrl = ctrl_hfto(nom,model,cfg)
% Tracking controller of our method: LTV-MPC (solve_ltv_mpc) around the
% nominal trajectory nom (plan_trajectory), paper Section 2.4. Controller
% interface shared with ctrl_mpcgraph: [u,cs,info] = ctrl.step(t,x,cs).
% With cfg.tube.K (TV-LQR gains, tvlqr_gains.m) and cfg.tube.cbar the MPC
% output is clipped to u*_k + K_k (x - x*_k) +- cbar, paper Eq. (clip), so
% the tube certificate covers the MPC whatever it computes.
% With cfg.est.on, a current estimate (with_current_estimator.m) enters the
% MPC prediction as the affine term w_j = current_step(x*_j,u*_j,Vc) -
% step(x*_j,u*_j) along the nominal, and the MPC uses cfg.est.mpcQ.
h = model.h;
tend = nom.t(end);
xpp = spline(nom.t,nom.x);
upp = spline(nom.t,nom.u);
D.xppval = @(t) ppval(xpp,t);
D.uppval = @(t) ppval(upp,t);
D.Aboxval = timeseries(nom.Aboxdata,nom.t);
D.Bboxval = timeseries(nom.Bboxdata,nom.t);
Agrid = zeros(6,6,numel(nom.t));
for k = 1:numel(nom.t)
    Agrid(:,:,k) = model.jacA(nom.x(:,k));
end
D.At = @(t) Agrid(:,:,min(round(t/h)+1,size(Agrid,3)));
D.B = model.B;
D.h = h;
D.N = cfg.Nmpc;
D.tend = tend;
D.opts.ulim = cfg.uLim;
D.opts.Q = cfg.mpcQ;
if cfg.est.on
    D.opts.Q = cfg.est.mpcQ;
end
D.opts.R = cfg.mpcR;
D.model = model;
if isfield(cfg,'tube') && isfield(cfg.tube,'K')
    D.K = cfg.tube.K;
    D.cbar = cfg.tube.cbar(:);
    D.xs = nom.x;
    D.us = nom.u;
end

ctrl.name = 'HF-TO';
ctrl.x0 = nom.x(:,1);
ctrl.tEnd = tend + cfg.tExtra;
ctrl.cs = struct;
ctrl.xref = @(t) D.xppval(min(t,tend));
ctrl.step = @(t,x,cs) hfto_step(t,x,cs,D);
if cfg.est.on
    ctrl = with_current_estimator(ctrl,model,cfg);
end
end

function [u,cs,info] = hfto_step(t,x,cs,D)
tc = min(t,D.tend);
dx = x - D.xppval(tc);
opts = D.opts;
if isfield(cs,'Vc') && any(cs.Vc)
    tj = min(t + (0:D.N-1)*D.h,D.tend);
    xs = D.xppval(tj);
    us = D.uppval(tj);
    opts.w = zeros(6,D.N);
    for j = 1:D.N
        opts.w(:,j) = current_step(D.model,xs(:,j),us(:,j),cs.Vc) - D.model.step(xs(:,j),us(:,j));
    end
end
[ubar,flag] = solve_ltv_mpc(D.At,D.B,t,dx,D.h,D.N,D.uppval,D.Aboxval,D.Bboxval,D.xppval,D.tend,opts);
u = ubar + D.uppval(tc);
if isfield(D,'K')
    k = min(round(t/D.h)+1,size(D.xs,2));
    uanc = D.us(:,k) + D.K(:,:,k)*(x - D.xs(:,k));
    u = min(max(u,uanc - D.cbar),uanc + D.cbar);
end
info.flag = flag;
info.done = false;
end
