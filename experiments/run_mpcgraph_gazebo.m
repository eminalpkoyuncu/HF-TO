% MPC-Graph on the Heron in Gazebo, with the same ROS loop, funnels, limits
% and metrics as run_hfto_gazebo.m. Uses the 'gazebo'
% preset of default_config; extra settings go in cfgOverride. Set
% cfgOverride.dryRun = true to run against the model instead of Gazebo.
clearvars -except cfgOverride
close all
setup;
if ~exist('cfgOverride','var')
    cfgOverride = struct;
end
if ~isfield(cfgOverride,'preset')
    cfgOverride.preset = 'gazebo';
end
dryRun = isfield(cfgOverride,'dryRun') && cfgOverride.dryRun;
[cfg,map,nodes,path,graph_params,regions,model] = mpcgraph_setup( ...
    rmfield(cfgOverride,intersect(fieldnames(cfgOverride),{'dryRun'})),'MPC-Graph');
if ~isfield(cfg,'outFile')
    cfg.outFile = fullfile(cfg.resultsDir,sprintf('mg_gazebo_map%d_%s_%s.mat', ...
        cfg.mapNo,cfg.model,string(datetime('now','Format','yyyyMMdd_HHmmss'))));
end
ctrl = ctrl_mpcgraph(nodes,path,map,model,cfg);

if dryRun
    io = sim_io(model,ctrl.x0,cfg.h);
else
    io = ros_io(cfg,map);
end
[res,log] = gazebo_run(ctrl,io,map,regions,cfg);
summary = print_summary(res,cfg,'MPC-Graph (Gazebo)');
fprintf('Cycle time [ms]: mean %.1f; controller overruns: %d\n', 1e3*res.cycleMean, res.overruns);
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end
save(cfg.outFile,'cfg','res','log','summary','map','nodes','path');
fprintf('Saved %s\n', cfg.outFile);
plot_runs(res,map,nodes,path,[],'MPC-Graph (Gazebo)');
