function [cfg,map,nodes,path,graph_params,regions,model] = mpcgraph_setup(override,label)
% Settings, funnels and model for the MPC-Graph experiments, from the same
% code as for our method, so both methods solve exactly the same problem.
% The run time limit is cfg.mg.tMaxFactor times the duration of our nominal
% trajectory when its plan file exists (run run_hfto_mc.m first).
cfg = default_config(override);
[map,nodes,path,graph_params,regions] = build_funnels(cfg);
cfg.REF = graph_params.REF;
model = make_model(cfg);

if isempty(cfg.mg.tMax)
    if isfile(cfg.planFile)
        S = load(cfg.planFile,'nom','planCfg');
        if S.planCfg.mapNo == cfg.mapNo && S.planCfg.seed == cfg.seed
            cfg.mg.tMax = cfg.mg.tMaxFactor*S.nom.t(end);
            fprintf('%s: run time limit %.1f s (%g x HF-TO nominal)\n', label, cfg.mg.tMax, cfg.mg.tMaxFactor);
        end
    end
    if isempty(cfg.mg.tMax)
        cfg.mg.tMax = cfg.mg.tMaxDefault;
        fprintf('%s: no HF-TO plan in %s, run time limit %.1f s\n', label, cfg.planFile, cfg.mg.tMax);
    end
end
end
