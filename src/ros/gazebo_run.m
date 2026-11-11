function [res,log] = gazebo_run(ctrl,io,map,regions,cfg)
% Real-time run of a controller (ctrl_hfto or ctrl_mpcgraph) against the
% Gazebo Heron through io = ros_io(cfg), or against sim_io for a dry run.
% io.start(), [t,x] = io.read(), io.write(F), io.wait(), io.stop().
% Controller time excludes the blocking reads, so it is comparable to the
% Monte Carlo controller time; the full cycle time is logged separately.
% The same metrics as in the Monte Carlo runs are computed afterwards.
nMax = ceil(ctrl.tEnd*cfg.gz.rate) + 10;
T = nan(1,nMax+1);
X = nan(6,nMax+1);
U = nan(2,nMax);
cpu = nan(1,nMax);
cycle = nan(1,nMax);
flags = nan(1,nMax);
sat = false(1,nMax);
cs = ctrl.cs;
psiPrev = [];
k = 0;
io.start();
tCycle = tic;
while true
    [t,x] = io.read();
    if ~isempty(psiPrev)
        x(3) = psiPrev + mod(x(3)-psiPrev+pi,2*pi) - pi;
    end
    psiPrev = x(3);
    T(k+1) = t;
    X(:,k+1) = x;
    if t > ctrl.tEnd || k >= nMax
        break;
    end
    if cfg.stopOnExit && ~in_regions(x(1:2),regions)
        break;
    end
    tq = tic;
    [ucmd,cs,info] = ctrl.step(t,x,cs);
    tc = toc(tq);
    if info.done
        break;
    end
    usat = min(max(ucmd,cfg.uLim(1)),cfg.uLim(2));
    ureal = usat + cfg.noiseLevel*abs(usat).*(2*rand(2,1)-1);
    io.write(ureal);
    k = k+1;
    U(:,k) = usat;
    cpu(k) = tc;
    flags(k) = info.flag;
    sat(k) = any(abs(usat-ucmd) > 1e-6);
    cycle(k) = toc(tCycle);
    io.wait();
    tCycle = tic;
end
io.stop();

log.T = T(1:k+1);
log.X = X(:,1:k+1);
log.U = U(:,1:k);
log.cpu = cpu(1:k);
log.cycle = cycle(1:k);
log.flags = flags(1:k);
log.sat = sat(1:k);
log.P = log.X(1:2,:);
log.TP = log.T;
res = run_metrics(log,map,regions,ctrl,cfg);
res.cycleMean = mean(log.cycle);
res.overruns = sum(log.cpu > 1/cfg.gz.rate);
end
