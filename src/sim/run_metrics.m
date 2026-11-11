function res = run_metrics(log,map,regions,ctrl,cfg)
% Metrics of one run, the same for every controller and for simulation and
% Gazebo. log.T (1xK+1), log.X (6xK+1), log.U (2xK, held on [T(k),T(k+1)]),
% log.cpu, log.flags (QP exit flags), log.sat (1xK), and the positions used
% for the region and clearance checks, log.P (2xM) at times log.TP (1xM).
% A run fails if any position leaves the union of the funnel regions, or if
% the final position error exceeds cfg.goalTol.
inside = in_regions(log.P,regions);
iOut = find(~inside,1);
res.reason = "";
res.tFail = NaN;
if ~isempty(iOut)
    res.reason = "left_regions";
    res.tFail = log.TP(iOut);
end
goal = map.goal(1:2);
err = vecnorm(log.X(1:2,:) - goal(:));
res.finalErr = err(end);
if res.reason == "" && res.finalErr > cfg.goalTol
    res.reason = "goal_not_reached";
end
res.success = (res.reason == "");

% time from which the vessel stays within goalTol of the goal
lastOut = find(err > cfg.goalTol,1,'last');
if isempty(lastOut)
    res.tGoal = log.T(1);
elseif lastOut == numel(err)
    res.tGoal = NaN;
else
    res.tGoal = log.T(lastOut+1);
end
res.duration = log.T(end) - log.T(1);
res.pathLen = sum(vecnorm(diff(log.X(1:2,:),1,2)));

if isempty(ctrl.xref)
    res.rmsPosErr = NaN;
    res.maxPosErr = NaN;
else
    Xn = ctrl.xref(log.T - log.T(1));
    posErr = vecnorm(log.X(1:2,:) - Xn(1:2,:));
    res.rmsPosErr = sqrt(mean(posErr.^2));
    res.maxPosErr = max(posErr);
end

res.minClear = inf;
for j = 1:size(log.P,2)
    res.minClear = min(res.minClear,min_dist(log.P(:,j),map.limits,map.obstacle));
end

K = size(log.U,2);
dt = diff(log.T(1:K+1));
U = log.U;
X = log.X(:,1:K);
res.effort = sum(sum(U.^2,1).*dt);
res.energy = sum((abs((U(1,:)+U(2,:)).*X(4,:)) + abs(cfg.armB*(U(1,:)-U(2,:)).*X(6,:))).*dt);
res.cpuMean = mean(log.cpu);
res.cpuMax = max([log.cpu -inf]);
res.qpFails = sum(~ismember(log.flags,[0 1]));
res.satSteps = sum(log.sat);
res.nSteps = K;
res.xy = single(log.X(1:2,:));
end
