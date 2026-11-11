% All Monte Carlo experiments of the paper (Section 4.1-4.2): our method and
% MPC-Graph, 500 runs each with the same seeds, on maps 3 (Map 1) and 13
% (Map 2), with the Fossen and the identified model, in the cases
%   'baseline': thrust disturbance F + |F|*n, n ~ U[-1,1] (noise level 1),
%   'param':    + plant parameters +-20 % (make_plant.m), controllers nominal,
%   'current':  + constant current, speed N(0.3, 0.1^2) m/s, direction
%               uniform, drawn per run,
% each without and with the current estimate (with_current_estimator.m).
% MPC-Graph runs at both margins (0.15 m and 0); the paper reports the better.
% Results: results/<hfto|mg>_mce_map<N>_<model>[_<case>][_est][_margin0].mat
% (no case suffix for the baseline without estimate). The plans are made
% once per map and model and reused. Define a struct batch before running
% to restrict the experiments, e.g.
%   batch.cases = {'baseline'}; batch.maps = 13; batch.est = false;
%   batch.methods = {'hfto'}; batch.nRuns = 50;
% batch.skipExisting = true leaves result files that already exist.
% Runtime: about 30 min per set of 500 runs on 8 workers.
clearvars -except batch
setup;
B.cases = {'baseline','param','current'};
B.maps = [13 3];
B.models = {'fossen','identified'};
B.est = [false true];
B.nRuns = 500;
B.methods = {'hfto','mg'};
B.skipExisting = false;
if exist('batch','var')
    for fn = fieldnames(batch).'
        B.(fn{1}) = batch.(fn{1});
    end
end
caseCfg.baseline = struct;
caseCfg.param = struct('paramPert',0.2);
caseCfg.current = struct('current',0.3,'currentStd',0.1);
resultsDir = default_config().resultsDir;

for est = B.est
    for c = B.cases
        for mapNo = B.maps
            for mo = B.models
                base = caseCfg.(c{1});
                base.mapNo = mapNo;
                base.model = mo{1};
                base.nRuns = B.nRuns;
                base.est.on = est;
                tag = sprintf('map%d_%s',mapNo,mo{1});
                if ~strcmp(c{1},'baseline') || est
                    tag = [tag '_' c{1}]; %#ok<AGROW>
                end
                if est
                    tag = [tag '_est']; %#ok<AGROW>
                end

                o = base;
                o.outFile = fullfile(resultsDir,['hfto_mce_' tag '.mat']);
                if ismember('hfto',B.methods) && ~(B.skipExisting && isfile(o.outFile))
                    run_script('run_hfto_mc.m',o);
                end

                for margin = [0.15 0]
                    if ~ismember('mg',B.methods)
                        break
                    end
                    o = base;
                    o.posMargin = margin;
                    suffix = '';
                    if margin == 0
                        suffix = '_margin0';
                    end
                    o.outFile = fullfile(resultsDir,['mg_mce_' tag suffix '.mat']);
                    if B.skipExisting && isfile(o.outFile)
                        continue
                    end
                    run_script('run_mpcgraph_mc.m',o);
                end
            end
        end
    end
end

function run_script(file,cfgOverride) %#ok<INUSD>
% the experiment scripts clear their workspace; run them in this one
run(file);
close all
end
