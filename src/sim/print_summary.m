function S = print_summary(results,cfg,label)
% Summary of the reviewer-requested metrics over a set of runs.
succ = [results.success];
reasons = [results.reason];
S.label = label;
S.nRuns = numel(results);
S.successRate = mean(succ);
S.leftRegions = sum(reasons == "left_regions");
S.goalNotReached = sum(reasons == "goal_not_reached");
S.minClear = min([results.minClear]);
S.minClearP5 = prctile([results.minClear],5);
S.minClearMedian = median([results.minClear]);
S.rmsPosErr = mean([results.rmsPosErr]);
S.finalErr = mean([results(succ).finalErr]);
S.finalErrMax = max([results(succ).finalErr]);
S.tGoal = mean([results(succ).tGoal]);
S.pathLen = mean([results(succ).pathLen]);
S.effort = mean([results.effort]);
S.energy = mean([results.energy]);
S.cpuMean = mean([results.cpuMean]);
S.cpuMax = max([results.cpuMax]);
S.qpFailSteps = sum([results.qpFails]);
S.qpFailRuns = sum([results.qpFails] > 0);
S.satPct = 100*sum([results.satSteps])/max(sum([results.nSteps]),1);

fprintf('\n%s: map %d, %s model, noise level %.2f, goal tolerance %.2f m\n', ...
    label, cfg.mapNo, cfg.model, cfg.noiseLevel, cfg.goalTol);
if isfield(cfg,'paramPert') && (cfg.paramPert > 0 || cfg.current > 0)
    fprintf('Plant mismatch: parameters +-%.0f %%, current %.2f +- %.2f (1 sigma) m/s\n', ...
        100*cfg.paramPert, cfg.current, cfg.currentStd);
end
fprintf('Success rate:          %.1f %% (%d/%d)\n', 100*S.successRate, sum(succ), S.nRuns);
fprintf('  left regions:        %d\n', S.leftRegions);
fprintf('  goal not reached:    %d\n', S.goalNotReached);
fprintf('Min obstacle clearance [m]: min %.3f, 5th pct %.3f, median %.3f\n', S.minClear, S.minClearP5, S.minClearMedian);
fprintf('Tracking error RMS [m]: %.3f\n', S.rmsPosErr);
fprintf('Final position error [m] (successful runs): mean %.3f, max %.3f\n', S.finalErr, S.finalErrMax);
fprintf('Time to goal [s] (successful runs): %.1f, path length [m]: %.1f\n', S.tGoal, S.pathLen);
fprintf('Control effort int(F1^2+F2^2) [N^2 s]: %.0f\n', S.effort);
fprintf('Mechanical energy [J]: %.1f\n', S.energy);
timingNote = "";
if cfg.useParallel && S.nRuns > 1
    timingNote = " (parallel runs: rerun serially for reportable timing)";
end
fprintf('Controller time [ms]: mean %.1f, max %.1f%s\n', 1e3*S.cpuMean, 1e3*S.cpuMax, timingNote);
fprintf('QP failures: %d steps in %d runs; saturated steps: %.2f %%\n', S.qpFailSteps, S.qpFailRuns, S.satPct);
end
