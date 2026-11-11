function res = simulate_run(ctrl,model,map,regions,cfg,seed)
% One closed-loop run of a controller (ctrl_hfto or ctrl_mpcgraph) on the
% plant, with multiplicative thrust disturbance F_real = F + level*|F|*n,
% n ~ U[-1,1], redrawn every control step, and the parameter mismatch and
% current of cfg.paramPert / cfg.current (make_plant.m). The run stops when the vessel
% leaves the union of the funnel regions (checked at every integrator
% output), when the controller reports arrival, or at ctrl.tEnd.
rng(seed);
h = model.h;
plant = make_plant(model,cfg,seed);

nSteps = round(ctrl.tEnd/h);
X = nan(6,nSteps+1);
U = nan(2,nSteps);
cpu = nan(1,nSteps);
flags = nan(1,nSteps);
sat = false(1,nSteps);
VC = nan(2,nSteps);
P = cell(1,nSteps+1);
TP = cell(1,nSteps+1);
x = ctrl.x0;
X(:,1) = x;
P{1} = x(1:2);
TP{1} = 0;
cs = ctrl.cs;
k = 0;
while k < nSteps
    t = k*h;
    tq = tic;
    [ucmd,cs,info] = ctrl.step(t,x,cs);
    tc = toc(tq);
    if info.done
        break;
    end
    k = k+1;
    cpu(k) = tc;
    flags(k) = info.flag;
    if isfield(cs,'Vc')
        VC(:,k) = cs.Vc;
    end
    usat = min(max(ucmd,cfg.uLim(1)),cfg.uLim(2));
    sat(k) = any(abs(usat-ucmd) > 1e-6);
    ureal = usat + cfg.noiseLevel*abs(usat).*(2*rand(2,1)-1);

    [x,pts,tp] = plant.advance(x,ureal,t);
    X(:,k+1) = x;
    U(:,k) = usat;
    P{k+1} = pts;
    TP{k+1} = tp;
    if cfg.stopOnExit && ~all(in_regions(pts,regions))
        break;
    end
end
log.T = (0:k)*h;
log.X = X(:,1:k+1);
log.U = U(:,1:k);
log.cpu = cpu(1:k);
log.flags = flags(1:k);
log.sat = sat(1:k);
log.P = [P{1:k+1}];
log.TP = [TP{1:k+1}];
res = run_metrics(log,map,regions,ctrl,cfg);
res.paramScale = plant.paramScale;
res.currentDir = plant.currentDir;
res.currentSpeed = plant.currentSpeed;
res.Vc = plant.Vc;
res.VcHat = single(VC(:,1:k));    % current estimate (NaN without estimator)
end
