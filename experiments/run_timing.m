% Computation times of paper Table (timecomplex): planning time of our
% method (the TO over the whole funnel sequence, replanned from scratch) and
% the controller time per step of both methods, measured in this MATLAB
% process without parallel workers, over runs 2..nRuns of nRuns (the first
% run contains MATLAB's first-call overhead). Run it on an otherwise idle
% machine. Results: results/timing.mat.
clearvars
setup;
nRuns = 20;
maps = [3 13];
models = {'fossen','identified'};
T = table();
for mapNo = maps
    for mo = models
        o = struct('mapNo',mapNo,'model',mo{1},'nRuns',nRuns,'useParallel',false);
        cfg = default_config(o);
        [map,nodes,path,gp,regions] = build_funnels(cfg);
        cfg.REF = gp.REF;
        model = make_model(cfg);
        [~,planStats] = plan_trajectory(map,nodes,path,model,cfg);
        nom = get_plan(map,nodes,path,model,cfg);
        ctrls = {ctrl_hfto(nom,model,cfg)};
        o.posMargin = 0;
        [cfgm,mapm,nodesm,pathm,~,regionsm,modelm] = mpcgraph_setup(o,'MPC-Graph');
        ctrls{2} = ctrl_mpcgraph(nodesm,pathm,mapm,modelm,cfgm);
        cpu = zeros(2,2);
        for m = 1:2
            if m == 1
                R = arrayfun(@(k) simulate_run(ctrls{1},model,map,regions,cfg,cfg.seed+k),1:nRuns);
            else
                R = arrayfun(@(k) simulate_run(ctrls{2},modelm,mapm,regionsm,cfgm,cfgm.seed+k),1:nRuns);
            end
            R = R(2:end);
            cpu(m,:) = 1e3*[mean([R.cpuMean]) max([R.cpuMax])];
        end
        row = table(mapNo,string(mo{1}),planStats.totalTime,planStats.fails, ...
            cpu(1,1),cpu(1,2),cpu(2,1),cpu(2,2),'VariableNames', ...
            {'map','model','plan_s','plan_fails','ours_mean_ms','ours_max_ms','mg_mean_ms','mg_max_ms'});
        disp(row);
        T = [T; row]; %#ok<AGROW>
    end
end
disp(T);
save(fullfile(cfg.resultsDir,'timing.mat'),'T');
