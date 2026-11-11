% Tuning of the MPC-Graph heading weight cfg.mg.Q(3,3) (line-of-sight
% heading reference, ctrl_mpcgraph.m) on the tuning seeds 10001-10000+nSeeds,
% which the Monte Carlo experiment does not use (it uses seeds 2-501). Thrust
% noise only, both maps and models, both margins. Prints the success rates.
clearvars
setup;
qpsi = [0 1 5 20 50];
nSeeds = 40;
maps = [13 3];
models = {'fossen','identified'};
if isempty(gcp('nocreate'))
    parpool(8);
end
rate = nan(numel(maps),numel(models),2,numel(qpsi));
for im = 1:numel(maps)
    for io = 1:numel(models)
        for ig = 1:2
            margin = [0.15 0];
            o = struct;
            o.mapNo = maps(im);
            o.model = models{io};
            o.posMargin = margin(ig);
            [cfg,map,nodes,path,~,regions,model] = mpcgraph_setup(o,'MPC-Graph');
            for iq = 1:numel(qpsi)
                c = cfg;
                c.mg.Q(3,3) = qpsi(iq);
                ctrl = ctrl_mpcgraph(nodes,path,map,model,c);
                succ = false(1,nSeeds);
                parfor k = 1:nSeeds
                    r = simulate_run(ctrl,model,map,regions,c,10000+k);
                    succ(k) = r.success;
                end
                rate(im,io,ig,iq) = mean(succ);
                fprintf('map %2d %-10s margin %.2f Qpsi %4g: %5.1f %%\n', ...
                    maps(im),models{io},margin(ig),qpsi(iq),100*mean(succ));
            end
        end
    end
end
save(fullfile(default_config().resultsDir,'tune_mpcgraph_heading.mat'),'rate','qpsi','maps','models','nSeeds');
