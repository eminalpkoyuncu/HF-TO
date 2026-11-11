% Our method on the Heron in Gazebo (paper Section 4.5): plan (or load the
% plan), then track it with the LTV-MPC at cfg.gz.rate through ROS. Start
% the simulator first (gazebo/gz.sh up). Uses the 'gazebo' preset of
% default_config (map 14, identified model, Heron thrust limits); extra
% settings go in cfgOverride. Set cfgOverride.dryRun = true to run against
% the model instead of Gazebo. MPC-Graph counterpart: run_mpcgraph_gazebo.m.
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
cfg = default_config(rmfield(cfgOverride,intersect(fieldnames(cfgOverride),{'dryRun'})));
if ~isfield(cfg,'outFile')
    cfg.outFile = fullfile(cfg.resultsDir,sprintf('hfto_gazebo_map%d_%s_%s.mat', ...
        cfg.mapNo,cfg.model,string(datetime('now','Format','yyyyMMdd_HHmmss'))));
end

[map,nodes,path,graph_params,regions] = build_funnels(cfg);
cfg.REF = graph_params.REF;
model = make_model(cfg);
[nom,planStats] = get_plan(map,nodes,path,model,cfg);
ctrl = ctrl_hfto(nom,model,cfg);

if dryRun
    io = sim_io(model,ctrl.x0,cfg.h);
else
    io = ros_io(cfg,map);
end
[res,log] = gazebo_run(ctrl,io,map,regions,cfg);
summary = print_summary(res,cfg,'HF-TO (Gazebo)');
fprintf('Cycle time [ms]: mean %.1f; controller overruns: %d\n', 1e3*res.cycleMean, res.overruns);
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end
save(cfg.outFile,'cfg','planStats','nom','res','log','summary','map','nodes','path');
fprintf('Saved %s\n', cfg.outFile);
plot_runs(res,map,nodes,path,nom.x(1:2,:),'HF-TO (Gazebo)');
