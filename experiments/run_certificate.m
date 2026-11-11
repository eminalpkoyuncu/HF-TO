% Safety certificate experiments (paper Sections 2.5 and 4.3): the LTV-MPC
% output is clipped to u*_k + K_k e_k +- cbar, paper Eq. (clip), so the
% zonotope certificate (tube_zonotope.m, certify_plan.m) covers the MPC.
% For each case (maps 13 and 3, Fossen and identified model):
%   1. TV-LQR gains along the plan and the certified noise level
%      level*(cbar) for a sweep of correction budgets cbar,
%   2. cbarSel = largest cbar with level*(cbar) >= keep*level*(0),
%   3. Monte Carlo with the clipped MPC: nRunsCert runs at level*(cbarSel)
%      (all must stay in the tube) and nRunsMC runs at the MCE level 1
%      (success rate and metrics, against the unclipped HF-TO batch).
% Fossen: the plant is the continuous model (ode45) and the tube the Euler
% model, so the one-step difference is bounded by sampling over the tube
% (opt.modelErr, as the Lipschitz constant). Results:
% results/optionA_map<N>_<model>.mat. Set optA.* before running to restrict it.
clearvars -except optA
setup;
resultsDir = default_config().resultsDir;

O.maps = [13 3];
O.models = {'identified','fossen'};
O.cbar = [0 0.05 0.1 0.2 0.3 0.5];   % [N] per thruster
O.keep = 0.5;
O.Qx = diag([100 100 10 10 10 10]);
O.Ru = 1e-3*eye(2);
O.rv = 0;
O.nRunsCert = 200;
O.nRunsMC = 500;
O.nSamp = 12;                  % samples per step for the Fossen model error
if exist('optA','var')
    for fn = fieldnames(optA).'
        O.(fn{1}) = optA.(fn{1});
    end
end

th = linspace(0,2*pi,33); th(end) = [];
Cdir = [cos(th); sin(th)].';

for mapNo = O.maps
    for mo = O.models
        o = struct;
        o.mapNo = mapNo;
        o.model = mo{1};
        cfg = default_config(o);
        [map,nodes,path,gp,regions] = build_funnels(cfg);
        cfg.REF = gp.REF;
        model = make_model(cfg);
        nom = get_plan(map,nodes,path,model,cfg);
        T = size(nom.x,2);
        K = tvlqr_gains(nom,model,O.Qx,O.Ru,O.Qx);
        fprintf('\n=== map %d, %s: %d samples ===\n',mapNo,mo{1},T);

        topt = struct;
        if ~isempty(model.f)
            topt.modelErr = @(k,beta,du) euler_err(model,nom,k,beta,du,O.nSamp);
        end
        certAt = @(lev,cb) certify_plan(nom,tube_zonotope(nom,model,K, ...
            setfield(setfield(topt,'level',lev),'cmax',cb*[1;1])),cfg,O.rv);

        % 1. level*(cbar)
        levStar = zeros(size(O.cbar));
        for j = 1:numel(O.cbar)
            lo = 0; hi = 1;
            if ~certAt(0,O.cbar(j)).ok
                levStar(j) = NaN;      % not even certified without noise
                fprintf('cbar %4.2f N: not certified at level 0\n',O.cbar(j));
                continue;
            end
            for it = 1:12
                mid = (lo+hi)/2;
                if certAt(mid,O.cbar(j)).ok
                    lo = mid;
                else
                    hi = mid;
                end
            end
            levStar(j) = lo;
            fprintf('cbar %4.2f N: level* = %.4f\n',O.cbar(j),lo);
        end

        % 2. budget
        ok = find(levStar >= O.keep*levStar(1));
        if isempty(ok)
            fprintf('not certifiable for any cbar; skipping the Monte Carlo runs\n');
            save(fullfile(resultsDir,sprintf('optionA_map%d_%s.mat',mapNo,mo{1})),'O','K','levStar','nom');
            continue;
        end
        jSel = ok(end);
        cbarSel = O.cbar(jSel);
        levSel = levStar(jSel);
        tubeSel = tube_zonotope(nom,model,K,setfield(setfield(topt,'level',levSel),'cmax',cbarSel*[1;1])); %#ok<SFLD>
        certSel = certify_plan(nom,tubeSel,cfg,O.rv);
        fprintf('selected cbar = %.2f N, level* = %.4f (position slack %.3f m, input slack %.2f N)\n', ...
            cbarSel,levSel,certSel.minPos,certSel.minU);

        % 3. Monte Carlo with the clipped MPC
        c = cfg;
        c.tube.K = K;
        c.tube.cbar = cbarSel*[1;1];
        ctrl = ctrl_hfto(nom,model,c);
        mc = struct;
        levs = [levSel 1];
        nr = [O.nRunsCert O.nRunsMC];
        for j = 1:2
            cj = c;
            cj.noiseLevel = levs(j);
            if j == 1
                cj.goalTol = inf;          % only the regions matter for the certificate
            end
            R = cell(1,nr(j));
            parfor r = 1:nr(j)
                R{r} = simulate_run(ctrl,model,map,regions,cj,cj.seed+r);
            end
            R = [R{:}];
            mc(j).level = levs(j);
            mc(j).results = R;
            mc(j).success = mean([R.success]);
            if j == 1
                ratio = 0;
                for r = 1:numel(R)
                    xy = double(R(r).xy);
                    for k = 2:min(size(xy,2),T)
                        sup = tubeSel.posSup(k,Cdir);
                        ce = Cdir*(xy(:,k) - nom.x(1:2,k));
                        okk = sup > 1e-6;
                        ratio = max([ratio; ce(okk)./sup(okk)]);
                    end
                end
                mc(j).maxRatio = ratio;
                fprintf('level %.4f, %d runs: in regions %.1f %%, max tube support ratio %.3f\n', ...
                    levs(j),nr(j),100*mc(j).success,ratio);
            else
                print_summary(R,cj,sprintf('HF-TO, clipped MPC (cbar %.2f N)',cbarSel));
            end
        end
        save(fullfile(resultsDir,sprintf('optionA_map%d_%s.mat',mapNo,mo{1})), ...
            'O','K','levStar','cbarSel','levSel','certSel','mc','nom');
    end
end

function [eps0,err] = euler_err(model,nom,k,beta,du,m)
% eps0: plant step - Euler step at the nominal (x*_k,u*_k); err: sampled
% bound of how much that difference varies over x*_k + box(beta), |F - u*_k| <= du
persistent opts
if isempty(opts)
    opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
end
h = model.h;
err = 0;
eps0 = plant_step(model,nom.x(:,k),nom.u(:,k),h,opts) - model.step(nom.x(:,k),nom.u(:,k));
if ~all(isfinite(beta)) || norm(beta(4:6)) > 5 || any(~isfinite(du))
    err = inf;      % diverged tube: not certifiable anyway, and ode45 would crawl
    return;
end
s = RandStream('mt19937ar','Seed',k);
for i = 1:m
    sx = sign(rand(s,6,1)-0.5).*rand(s,6,1).^0.2;   % mostly near the corners
    su = sign(rand(s,2,1)-0.5);
    x = nom.x(:,k) + beta.*sx;
    F = nom.u(:,k) + du.*su;
    err = max(err,norm(plant_step(model,x,F,h,opts) - model.step(x,F) - eps0));
end
end

function y = plant_step(model,x,F,h,opts)
[~,xs] = ode45(@(t,z) model.f(z,F),[0 h/2 h],x,opts);   % endpoints only
y = xs(end,:).';
end
