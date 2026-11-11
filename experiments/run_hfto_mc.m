% Monte Carlo experiment of our method (HF-TO): plan once with
% the funnel-constrained TO, then track the plan with the LTV-MPC in
% cfg.nRuns runs under random thrust disturbances (paper Section 4.1).
% Settings are in default_config.m; to change them without editing, define
% a struct cfgOverride (e.g. cfgOverride.mapNo = 3) before running this
% script. The MPC-Graph counterpart is run_mpcgraph_mc.m, and
% summarize_results.m tabulates both. Results: results/hfto_mce_*.mat.
clearvars -except cfgOverride
close all
setup;
if ~exist('cfgOverride','var')
    cfgOverride = struct;
end
cfg = default_config(cfgOverride);
if ~isfield(cfg,'outFile')
    cfg.outFile = fullfile(cfg.resultsDir,sprintf('hfto_mce_map%d_%s.mat',cfg.mapNo,cfg.model));
end

% ---------------------------------------------------------------
% funnels, model and plan
[map,nodes,path,graph_params,regions] = build_funnels(cfg);
cfg.REF = graph_params.REF;
model = make_model(cfg);
[nom,planStats] = get_plan(map,nodes,path,model,cfg);
ctrl = ctrl_hfto(nom,model,cfg);

% ---------------------------------------------------------------
% Monte Carlo runs
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

summary = print_summary(results,cfg,'HF-TO');
fprintf('Nominal duration %.1f s, planning total %.1f s\n', nom.t(end), planStats.totalTime);
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end
save(cfg.outFile,'cfg','planStats','nom','results','summary','map','nodes','path');
fprintf('Saved %s\n', cfg.outFile);

plot_runs(results,map,nodes,path,nom.x(1:2,:),'HF-TO');
