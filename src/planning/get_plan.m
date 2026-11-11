function [nom,planStats] = get_plan(map,nodes,path,model,cfg)
% Funnel-constrained TO plan, loaded from cfg.planFile when it was made with
% the same settings, otherwise computed and saved there.
keys = {'mapNo','seed','model','h','uTO','duMax','vnom','vthr','vpark','tSlack','V','posMargin'};
if cfg.reusePlan && isfile(cfg.planFile)
    S = load(cfg.planFile,'nom','planStats','planCfg');
    same = true;
    for i = 1:numel(keys)
        same = same && isfield(S.planCfg,keys{i}) && isequal(S.planCfg.(keys{i}),cfg.(keys{i}));
    end
    if same
        nom = S.nom;
        planStats = S.planStats;
        fprintf('Loaded plan from %s\n',cfg.planFile);
        return;
    end
    fprintf('Plan in %s was made with other settings; replanning.\n',cfg.planFile);
end
[nom,planStats] = plan_trajectory(map,nodes,path,model,cfg);
fprintf('Planning: %d regions, ok = %d, total %.1f s (%d failed attempts)\n', ...
    numel(path), planStats.ok, planStats.totalTime, planStats.fails);
if ~planStats.ok
    error('Trajectory optimization failed at region %d of %d.',planStats.failSeg,numel(path));
end
fprintf('  TO time per region [s]: %s\n', mat2str(planStats.segTime,3));
planCfg = struct;
for i = 1:numel(keys)
    planCfg.(keys{i}) = cfg.(keys{i});
end
if ~isfolder(fileparts(cfg.planFile))
    mkdir(fileparts(cfg.planFile));
end
save(cfg.planFile,'nom','planStats','planCfg');
end
