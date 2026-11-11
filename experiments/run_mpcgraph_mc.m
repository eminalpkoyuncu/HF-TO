% Monte Carlo experiment of MPC-Graph on the underactuated USV, on the same
% map, funnels, plant, thrust limits, disturbances (same seeds), failure
% criterion and metrics as run_hfto_mc.m. Settings: default_config.m
% (MPC-Graph ones in cfg.mg); override them by defining a struct cfgOverride
% before running this script. Results: results/mg_mce_*.mat.
clearvars -except cfgOverride
close all
setup;
if ~exist('cfgOverride','var')
    cfgOverride = struct;
end
[cfg,map,nodes,path,graph_params,regions,model] = mpcgraph_setup(cfgOverride,'MPC-Graph');
if ~isfield(cfg,'outFile')
    cfg.outFile = fullfile(cfg.resultsDir,sprintf('mg_mce_map%d_%s.mat',cfg.mapNo,cfg.model));
end
ctrl = ctrl_mpcgraph(nodes,path,map,model,cfg);

results = cell(1,cfg.nRuns);
tRuns = tic;
if cfg.useParallel && license('test','Distrib_Computing_Toolbox')
    parfor k = 1:cfg.nRuns
        results{k} = simulate_run(ctrl,model,map,regions,cfg,cfg.seed+k);
    end
else
    for k = 1:cfg.nRuns
        results{k} = simulate_run(ctrl,model,map,regions,cfg,cfg.seed+k);
    end
end
results = [results{:}];
fprintf('Simulated %d runs in %.1f s\n', cfg.nRuns, toc(tRuns));

summary = print_summary(results,cfg,'MPC-Graph');
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end
save(cfg.outFile,'cfg','results','summary','map','nodes','path');
fprintf('Saved %s\n', cfg.outFile);

plot_runs(results,map,nodes,path,[],'MPC-Graph');
