function ctrl = ctrl_mpcgraph(nodes,path,map,model,cfg)
% MPC-Graph (Atasoy et al., 2023) for the underactuated USV. In the current
% funnel, an MPC steers the vessel to the funnel outlet (the centroid of the
% overlap with the next funnel; the goal in the last funnel), with the
% predicted positions constrained to the current funnel (shrunk by
% cfg.posMargin, as in our TO), the predicted surge and yaw rates to cfg.V
% and the thrusts to cfg.uLim. The funnel switches when the vessel enters
% the next funnel. The original method has no heading; here the heading is
% weighted (cfg.mg.Q(3,3)) towards the line of sight to the outlet, so that
% the vessel does not drive in reverse. The double integrator
% of the original work is replaced by the USV model (model.step, jacA, B)
% linearized
%   'trajectory': along the rollout of the previous (shifted) input
%                 sequence, i.e. LTV prediction that sees the turns,
%   'current':    at the current state and last input (LTI), or
%   'outlet':     at rest at the outlet, heading towards it (LTI; the
%                 linearization described for MPC-Graph).
% The terminal cost is the DARE solution when it exists, Q otherwise.
% With cfg.est.on, the prediction model includes the current estimate cs.Vc
% (with_current_estimator.m, current_step.m).
% Controller interface shared with ctrl_hfto: [u,cs,info] = ctrl.step(t,x,cs).
np = numel(path);
D.refs = zeros(2,np);
D.boxes = cell(1,np);
D.Cpos = cell(1,np);
D.bpos = cell(1,np);
for k = 1:np
    node = nodes{path(k)};
    if k < np
        D.refs(:,k) = get_ref(node,nodes{path(k+1)},cfg.REF);
    else
        D.refs(:,k) = map.goal(1:2);
    end
    D.boxes{k} = node.box;
    [D.Cpos{k},D.bpos{k}] = region_halfspaces(node.box,D.refs(:,k));
    D.bpos{k} = D.bpos{k} - cfg.posMargin*vecnorm(D.Cpos{k},2,2);
end
D.np = np;
D.model = model;
D.mg = cfg.mg;
D.N = cfg.mg.N;
D.uLim = cfg.uLim;
D.V = cfg.V;
D.goal = map.goal(1:2);
D.goal = D.goal(:);
D.goalTol = cfg.goalTol;
D.qpoptions = optimoptions(@quadprog,'Display','off','MaxIterations',cfg.mg.qpMaxIter);

start = map.start(:);
ctrl.name = 'MPC-Graph';
ctrl.x0 = [start(1:2); start(3); 0; 0; 0];
if ~isempty(cfg.mg.tMax)
    ctrl.tEnd = cfg.mg.tMax;
else
    ctrl.tEnd = cfg.mg.tMaxDefault;
end
ctrl.cs = struct('k',1,'uprev',[0;0],'Upred',zeros(2,cfg.mg.N), ...
    'psiLin',atan2(D.refs(2,1)-start(2),D.refs(1,1)-start(1)));
ctrl.xref = [];
ctrl.step = @(t,x,cs) mg_step(x,cs,D);
if cfg.est.on
    ctrl = with_current_estimator(ctrl,model,cfg);
end
end

function [u,cs,info] = mg_step(x,cs,D)
k = cs.k;
if k < D.np && inpolygon(x(1),x(2),D.boxes{k+1}(1,:),D.boxes{k+1}(2,:))
    k = k+1;
    cs.k = k;
    cs.psiLin = atan2(D.refs(2,k)-x(2),D.refs(1,k)-x(1));
end
info.done = (k == D.np) && norm(x(1:2)-D.goal) <= D.goalTol && norm(x(4:5)) <= D.mg.vStop;
if info.done
    u = [0;0];
    info.flag = 1;
    return;
end

N = D.N;
nx = 6;
nu = 2;
model = D.model;
B = model.B;
if isfield(cs,'Vc') && any(cs.Vc)
    Vc = cs.Vc;
    model.step = @(x,u) current_step(D.model,x,u,Vc);
end
% prediction over j = 1..N as X = Xfree + G*dU, U = Ubar + dU
switch D.mg.lin
    case 'trajectory'
        % LTV along the rollout of the shifted previous input sequence
        Ubar = cs.Upred;
        [Xbar,Aseq] = rollout(model,x,Ubar);
        if ~all(isfinite(Xbar),'all') || ~all(isfinite(Aseq),'all')
            % diverged rollout: restart from zero thrust
            Ubar = zeros(nu,N);
            [Xbar,Aseq] = rollout(model,x,Ubar);
        end
        Xfree = reshape(Xbar(:,2:end),[],1);
        c = zeros(nx,1);
    case {'current','outlet'}
        % LTI, x+ = A x + B u + c
        if strcmp(D.mg.lin,'current')
            xl = x;
            ul = cs.uprev;
        else
            xl = [D.refs(:,k); cs.psiLin; 0; 0; 0];
            ul = [0;0];
        end
        A = model.jacA(xl);
        c = model.step(xl,ul) - A*xl - B*ul;
        Aseq = repmat(A,1,1,N);
        Ubar = zeros(nu,N);
        Xfree = [];
    otherwise
        error('Unknown linearization %s.',D.mg.lin);
end
G = zeros(nx*N,nu*N);
Gprev = zeros(nx,nu*N);
xprev = x;
xfree = zeros(nx*N,1);
for j = 1:N
    rows = nx*(j-1)+(1:nx);
    Gprev = Aseq(:,:,j)*Gprev;
    Gprev(:,nu*(j-1)+(1:nu)) = B;
    G(rows,:) = Gprev;
    xprev = Aseq(:,:,j)*xprev + c;
    xfree(rows) = xprev;
end
if isempty(Xfree)
    Xfree = xfree;
end
Ubar = Ubar(:);

% cost sum (x_j - xr)' Q (x_j - xr) + u_j' R u_j
Q = D.mg.Q;
R = D.mg.R;
Qf = Q;
if strcmp(D.mg.terminal,'dare')
    ws = warning('off','all');
    try
        P = dare(Aseq(:,:,N),B,Q,R);
        if all(isfinite(P),'all')
            Qf = (P+P.')/2;
        end
    catch
    end
    warning(ws);
end
% heading reference: line of sight to the outlet (cfg.mg.Q(3,3) weights it),
% the current heading once within cfg.mg.losMin of the outlet
d = D.refs(:,k) - x(1:2);
psiRef = x(3);
if norm(d) > D.mg.losMin
    psiRef = x(3) + wrapToPi(atan2(d(2),d(1)) - x(3));
end
xr = [D.refs(:,k); psiRef; 0; 0; 0];
Qbar = blkdiag(kron(eye(N-1),Q),Qf);
Rbar = kron(eye(N),R);
e0 = Xfree - repmat(xr,N,1);
Hqp = G.'*Qbar*G + Rbar;
Hqp = (Hqp+Hqp.')/2;
fqp = G.'*Qbar*e0 + Rbar*Ubar;

% funnel and velocity constraints on the predicted states
Cpos = D.Cpos{k};
Sp = kron(eye(N),[Cpos zeros(size(Cpos,1),4)]);
Sv = kron(eye(N),[0 0 0 1 0 0; 0 0 0 0 0 1]);
vmax = repmat([D.V.u(2); D.V.r(2)],N,1);
vmin = repmat([D.V.u(1); D.V.r(1)],N,1);
Aineq = [Sp*G; Sv*G; -Sv*G];
bineq = [repmat(D.bpos{k},N,1) - Sp*Xfree; vmax - Sv*Xfree; -vmin + Sv*Xfree];
lb = D.uLim(1) - Ubar;
ub = D.uLim(2) - Ubar;

[dU,~,flag] = quadprog(Hqp,fqp,Aineq,bineq,[],[],lb,ub,[],D.qpoptions);
if isempty(dU) || flag < 0
    % infeasible state constraints: fall back to the input-bounded problem
    dU = quadprog(Hqp,fqp,[],[],[],[],lb,ub,[],D.qpoptions);
    if isempty(dU)
        dU = zeros(nu*N,1);
    end
end
U = min(max(reshape(Ubar + dU,nu,N),D.uLim(1)),D.uLim(2));
u = U(:,1);
cs.uprev = u;
cs.Upred = [U(:,2:end) U(:,end)];
info.flag = flag;
end

function [Xbar,Aseq] = rollout(model,x,Ubar)
N = size(Ubar,2);
Xbar = zeros(numel(x),N+1);
Xbar(:,1) = x;
Aseq = zeros(numel(x),numel(x),N);
for j = 1:N
    Aseq(:,:,j) = model.jacA(Xbar(:,j));
    Xbar(:,j+1) = model.step(Xbar(:,j),Ubar(:,j));
end
end
