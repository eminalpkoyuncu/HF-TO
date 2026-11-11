% Figures of the paper, written to results/figures as PDF:
%   fig_mc_<model>.pdf     Monte Carlo trajectories, maps 1 and 2, both
%                          methods (MPC-Graph at its better margin),
%                          successful and failed runs separately
%   fig_run_map<N>.pdf     one representative run of both methods (same
%                          seed): path, u, v, r and thrusts against time
%   fig_tube_map<N>.pdf    certified tube (clipped MPC) and the runs at level*
%   fig_gazebo.pdf         our method in Gazebo (latest result file)
% Set figs.which (cell of 'mc','run','tube','gazebo') before running to
% restrict it.
clearvars -except figs
setup;
resDir = default_config().resultsDir;
outDir = fullfile(resDir,'figures');
if ~isfolder(outDir), mkdir(outDir); end
F.which = {'mc','run','tube','gazebo'};
if exist('figs','var')
    for fn = fieldnames(figs).', F.(fn{1}) = figs.(fn{1}); end
end
set(groot,'defaultTextInterpreter','latex','defaultAxesTickLabelInterpreter','latex', ...
    'defaultLegendInterpreter','latex','defaultAxesFontSize',8,'defaultTextFontSize',8);
C.hfto = [0 0.447 0.741];
C.mg = [0.85 0.325 0.098];
C.fail = [0.8 0.1 0.1];
C.nom = [0 0 0];
mapName = containers.Map({3,13},{'Map 1','Map 2'});
identCfg = struct('uLim',[-19.88 33.6],'uTO',[-15 28],'duMax',45);

%% Monte Carlo trajectories
if ismember('mc',F.which)
    for mo = {'fossen','identified'}
        f = figure('Units','centimeters','Position',[2 2 18 10.5],'Color','w');
        tl = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
        maps = [3 13];
        for i = 1:2
            S = load(fullfile(resDir,sprintf('hfto_mce_map%d_%s.mat',maps(i),mo{1})));
            M = best_mg(resDir,maps(i),mo{1});
            sets = {S.results, M.results};
            names = {'HF-TO', sprintf('MPC-Graph ($m=%g$ m)',M.cfg.posMargin)};
            cols = {C.hfto, C.mg};
            for j = 1:2
                R = sets{j};
                s = [R.success];
                ax = nexttile(tl);
                hold(ax,'on');
                draw_map(ax,S.map,S.nodes,S.path);
                for k = find(s)
                    plot(ax,R(k).xy(1,:),R(k).xy(2,:),'Color',[cols{j} 0.2],'LineWidth',0.4);
                end
                for k = find(~s)
                    plot(ax,R(k).xy(1,:),R(k).xy(2,:),'Color',[C.fail 0.12],'LineWidth',0.4);
                    plot(ax,R(k).xy(1,end),R(k).xy(2,end),'x','Color',C.fail,'MarkerSize',3);
                end
                plot(ax,S.nom.x(1,:),S.nom.x(2,:),'--','Color',C.nom,'LineWidth',0.7);
                title(ax,sprintf('(%s) %s, %s: %d/%d reached the goal','a'+2*(i-1)+j-1, ...
                    mapName(maps(i)),names{j},sum(s),numel(s)));
                axis(ax,'equal'); axis(ax,S.map.limits); box(ax,'on');
                xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
            end
        end
        save_fig(f,fullfile(outDir,sprintf('fig_mc_%s.pdf',mo{1})),'image');
    end
end

%% one representative run of both methods
if ismember('run',F.which)
    for mapNo = [3 13]
        o = struct('mapNo',mapNo,'model','fossen');
        cfg = default_config(o);
        [map,nodes,path,gp,regions] = build_funnels(cfg);
        cfg.REF = gp.REF;
        model = make_model(cfg);
        nom = get_plan(map,nodes,path,model,cfg);
        cs = ctrl_hfto(nom,model,cfg);
        M = best_mg(resDir,mapNo,'fossen');
        om = o; om.posMargin = M.cfg.posMargin;
        [cfgm,mapm,nodesm,pathm,~,regionsm,modelm] = mpcgraph_setup(om,'MPC-Graph');
        cm = ctrl_mpcgraph(nodesm,pathm,mapm,modelm,cfgm);
        % first seed on which both methods reach the goal
        S = load(fullfile(resDir,sprintf('hfto_mce_map%d_fossen.mat',mapNo)),'results');
        both = find([S.results.success] & [M.results.success],1);
        seed = cfg.seed + both;
        % on pool workers: their default random generator is the one of the
        % Monte Carlo batches, so the runs are the ones in the result files
        cc = {cs,cm}; mm = {model,modelm}; rr = {regions,regionsm}; gg = {cfg,cfgm};
        LL = cell(1,2);
        parfor i = 1:2
            LL{i} = run_logged(cc{i},mm{i},rr{i},gg{i},seed);
        end
        [La,Lb] = LL{:};

        f = figure('Units','centimeters','Position',[2 2 18 21],'Color','w');
        tl = tiledlayout(f,5,2,'TileSpacing','compact','Padding','compact');
        ax = nexttile(tl,[2 2]);
        hold(ax,'on');
        draw_map(ax,map,nodes,path);
        hn = plot(ax,nom.x(1,:),nom.x(2,:),'--','Color',C.nom,'LineWidth',0.8);
        ha = plot(ax,La.X(1,:),La.X(2,:),'Color',C.hfto,'LineWidth',1);
        hb = plot(ax,Lb.X(1,:),Lb.X(2,:),'Color',C.mg,'LineWidth',1);
        axis(ax,'equal'); axis(ax,map.limits); box(ax,'on');
        xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
        legend(ax,[hn ha hb],{'nominal (HF-TO)','HF-TO','MPC-Graph'},'Location','best');
        title(ax,sprintf('(a) %s, Fossen model, $\\ell = 1$, run %d',mapName(mapNo),both));
        labs = {'$u$ [m/s]','$v$ [m/s]','$r$ [rad/s]','$F_1$ [N]','$F_2$ [N]','Clearance [m]'};
        tags = 'bcdefg';
        for q = 1:6
            ax = nexttile(tl);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');
            if q <= 3
                plot(ax,nom.t,nom.x(q+3,:),'--','Color',C.nom,'LineWidth',0.8);
                plot(ax,La.T,La.X(q+3,:),'Color',C.hfto);
                plot(ax,Lb.T,Lb.X(q+3,:),'Color',C.mg);
            elseif q <= 5
                plot(ax,La.T(1:end-1),La.U(q-3,:),'Color',C.hfto);
                plot(ax,Lb.T(1:end-1),Lb.U(q-3,:),'Color',C.mg);
                plot(ax,nom.t,nom.u(q-3,:),'--','Color',C.nom,'LineWidth',0.8);
                yline(ax,cfg.uLim,':','Color',[0.4 0.4 0.4]);
            else
                plot(ax,La.T,clearance(La.X,map),'Color',C.hfto);
                plot(ax,Lb.T,clearance(Lb.X,map),'Color',C.mg);
                ylim(ax,[0 inf]);
            end
            ylabel(ax,labs{q}); xlabel(ax,'Time [s]');
            title(ax,['(' tags(q) ')']);
        end
        save_fig(f,fullfile(outDir,sprintf('fig_run_map%d.pdf',mapNo)));
        fprintf('run figure map %d: seed %d, HF-TO %s (%.1f s), MPC-Graph %s (%.1f s)\n',mapNo,seed, ...
            La.reason,La.T(end),Lb.reason,Lb.T(end));
        fprintf('  surge mean/min [m/s]: HF-TO %.2f/%.2f, MPC-Graph %.2f/%.2f; max |v|: %.2f, %.2f\n', ...
            mean(La.X(4,:)),min(La.X(4,:)),mean(Lb.X(4,:)),min(Lb.X(4,:)), ...
            max(abs(La.X(5,:))),max(abs(Lb.X(5,:))));
    end
end

%% certified tube and the runs at level*
if ismember('tube',F.which)
    for mapNo = [13 3]
        A = load(fullfile(resDir,sprintf('optionA_map%d_identified.mat',mapNo)));
        o = identCfg; o.mapNo = mapNo; o.model = 'identified';
        cfg = default_config(o);
        [map,nodes,path] = build_funnels(cfg);
        model = make_model(cfg);
        tube = tube_zonotope(A.nom,model,A.K,struct('level',A.levSel,'cmax',A.cbarSel*[1;1]));
        th = linspace(0,2*pi,65); th(end) = [];
        D = [cos(th); sin(th)].';
        T = size(A.nom.x,2);
        f = figure('Units','centimeters','Position',[2 2 18 16],'Color','w');
        tl = tiledlayout(f,4,2,'TileSpacing','compact','Padding','compact');
        ax = nexttile(tl,[2 2]);
        hold(ax,'on');
        draw_map(ax,map,nodes,path);
        for k = 1:2:T
            P = A.nom.x(1:2,k) + support_polygon(tube.posSup(k,D),D);   % posSup: error coordinates
            patch(ax,P(1,:),P(2,:),[0.3 0.6 0.9],'EdgeColor','none','FaceAlpha',0.25);
        end
        R = A.mc(1).results;
        for r = 1:numel(R)
            plot(ax,R(r).xy(1,:),R(r).xy(2,:),'Color',[C.mg 0.15],'LineWidth',0.3);
        end
        plot(ax,A.nom.x(1,:),A.nom.x(2,:),'k','LineWidth',0.8);
        axis(ax,'equal'); axis(ax,map.limits); box(ax,'on');
        xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
        title(ax,sprintf('(a) %s, identified model: tube (blue) at $\\ell^\\star = %.3f$, $\\bar c = %.1f$ N, and %d runs', ...
            mapName(mapNo),A.levSel,A.cbarSel,numel(R)));
        % (b) zoom on the stretch where the tube is widest
        ax0 = ax;
        ax = nexttile(tl,[2 1]);
        copyobj(allchild(ax0),ax);
        hold(ax,'on');
        rad0 = zeros(1,T);
        for k = 1:T, rad0(k) = max(tube.posSup(k,D)); end
        [~,kz] = max(rad0);
        w = max(8*rad0(kz),0.15);
        axis(ax,'equal'); axis(ax,[A.nom.x(1,kz)+[-w w] A.nom.x(2,kz)+[-w w]]); box(ax,'on');
        grid(ax,'on'); ax.Layer = 'top';
        xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
        title(ax,sprintf('(b) zoom at $t = %.1f$ s',A.nom.t(kz)));
        rectangle(ax0,'Position',[A.nom.x(1,kz)-w A.nom.x(2,kz)-w 2*w 2*w],'EdgeColor','k','LineWidth',0.8);
        % (c) tube extent from the nominal and the deviations of the runs
        rad = zeros(1,T);
        dev = zeros(1,T);
        for k = 1:T
            rad(k) = max(tube.posSup(k,D));
        end
        for r = 1:numel(R)
            xy = double(R(r).xy);
            n = min(size(xy,2),T);
            dev(1:n) = max(dev(1:n),vecnorm(xy(:,1:n) - A.nom.x(1:2,1:n)));
        end
        ax = nexttile(tl,[2 1]);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        plot(ax,A.nom.t,rad,'Color',[0.3 0.6 0.9],'LineWidth',1);
        plot(ax,A.nom.t,dev,'Color',C.mg);
        legend(ax,{'tube extent','largest run deviation'},'Location','northwest');
        xlabel(ax,'Time [s]'); ylabel(ax,'Distance from nominal [m]');
        title(ax,sprintf('(c) max. support ratio over all runs and directions: %.2f',A.mc(1).maxRatio));
        save_fig(f,fullfile(outDir,sprintf('fig_tube_map%d.pdf',mapNo)));
    end
end

%% Gazebo
if ismember('gazebo',F.which)
    files = dir(fullfile(resDir,'hfto_gazebo_map14_identified_*.mat'));
    [~,j] = max([files.datenum]);
    G = load(fullfile(files(j).folder,files(j).name));
    L = G.log;
    f = figure('Units','centimeters','Position',[2 2 18 20],'Color','w');
    tl = tiledlayout(f,5,2,'TileSpacing','compact','Padding','compact');
    ax = nexttile(tl,[2 2]);
    hold(ax,'on');
    draw_map(ax,G.map,G.nodes,G.path);
    plot(ax,G.nom.x(1,:),G.nom.x(2,:),'--','Color',C.nom,'LineWidth',0.8);
    plot(ax,L.X(1,:),L.X(2,:),'Color',C.hfto,'LineWidth',1);
    axis(ax,'equal'); axis(ax,G.map.limits); box(ax,'on');
    xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
    legend(ax,{'nominal','Gazebo'},'Location','best');
    title(ax,'(a)');
    labs = {'$u$ [m/s]','$v$ [m/s]','$r$ [rad/s]','$F_1$ [N]','$F_2$ [N]'};
    tags = 'bcdefg';
    for q = 1:5
        ax = nexttile(tl);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        if q <= 3
            plot(ax,G.nom.t,G.nom.x(q+3,:),'--','Color',C.nom,'LineWidth',0.8);
            plot(ax,L.T,L.X(q+3,:),'Color',C.hfto);
        else
            plot(ax,G.nom.t,G.nom.u(q-3,:),'--','Color',C.nom,'LineWidth',0.8);
            plot(ax,L.T(1:size(L.U,2)),L.U(q-3,:),'Color',C.hfto);
            yline(ax,G.cfg.uLim,':','Color',[0.5 0.5 0.5]);
        end
        ylabel(ax,labs{q}); xlabel(ax,'Time [s]');
        title(ax,['(' tags(q) ')']);
    end
    ax = nexttile(tl);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    plot(ax,L.T(1:numel(L.cpu)),1e3*L.cpu,'Color',C.hfto);
    plot(ax,L.T(1:numel(L.cycle)),1e3*L.cycle,'Color',C.mg);
    yline(ax,1e3/G.cfg.gz.rate,':','Color',[0.5 0.5 0.5]);
    set(ax,'YScale','log');
    legend(ax,{'MPC','control cycle'},'Location','best');
    ylabel(ax,'Time per step [ms]'); xlabel(ax,'Time [s]');
    title(ax,'(g)');
    save_fig(f,fullfile(outDir,'fig_gazebo.pdf'));
end

% ---------------------------------------------------------------
function M = best_mg(resDir,mapNo,model)
% MPC-Graph result at its better margin
A = load(fullfile(resDir,sprintf('mg_mce_map%d_%s.mat',mapNo,model)),'results','cfg');
B = load(fullfile(resDir,sprintf('mg_mce_map%d_%s_margin0.mat',mapNo,model)),'results','cfg');
M = A;
if mean([B.results.success]) > mean([A.results.success])
    M = B;
end
end

function draw_map(ax,map,nodes,path)
for j = 1:numel(path)
    b = nodes{path(j)}.box;
    patch(ax,b(1,:),b(2,:),[0.6 0.6 0.6],'FaceAlpha',0.18,'EdgeColor',[0.6 0.6 0.6],'LineWidth',0.3);
end
for i = 1:numel(map.obstacle)
    o = map.obstacle{i};
    patch(ax,o(1,:),o(2,:),'k','EdgeColor','none');
end
% metric grid on top of the map: labelled major lines, 1 m (10 m) minor lines
span = max(diff(map.limits(1:2)),diff(map.limits(3:4)));
if span <= 20
    major = 2; minor = 1;
elseif span <= 40
    major = 5; minor = 1;
else
    major = 20; minor = 10;
end
ax.XTick = map.limits(1):major:map.limits(2);
ax.YTick = map.limits(3):major:map.limits(4);
ax.XAxis.MinorTickValues = map.limits(1):minor:map.limits(2);
ax.YAxis.MinorTickValues = map.limits(3):minor:map.limits(4);
grid(ax,'on'); grid(ax,'minor');
ax.Layer = 'top';
ax.GridAlpha = 0.35; ax.MinorGridAlpha = 0.2;
ax.GridColor = [0.45 0.45 0.45]; ax.MinorGridColor = [0.6 0.6 0.6];
end

function P = support_polygon(s,D)
% polygon of the set {p : D p <= s} (D: unit directions, sorted by angle)
n = numel(s);
P = zeros(2,n);
for i = 1:n
    j = mod(i,n)+1;
    P(:,i) = [D(i,:); D(j,:)] \ [s(i); s(j)];
end
end

function L = run_logged(ctrl,model,regions,cfg,seed)
% simulate_run.m with the full state and thrust history kept
rng(seed);
h = model.h;
plant = make_plant(model,cfg,seed);
nSteps = round(ctrl.tEnd/h);
X = nan(6,nSteps+1);
U = nan(2,nSteps);
x = ctrl.x0;
X(:,1) = x;
cs = ctrl.cs;
k = 0;
L.reason = "time";
while k < nSteps
    t = k*h;
    [ucmd,cs,info] = ctrl.step(t,x,cs);
    if info.done
        L.reason = "arrived";
        break;
    end
    k = k+1;
    usat = min(max(ucmd,cfg.uLim(1)),cfg.uLim(2));
    ureal = usat + cfg.noiseLevel*abs(usat).*(2*rand(2,1)-1);
    [x,pts] = plant.advance(x,ureal,t);
    X(:,k+1) = x;
    U(:,k) = usat;
    if cfg.stopOnExit && ~all(in_regions(pts,regions))
        L.reason = "left_regions";
        break;
    end
end
L.T = (0:k)*h;
L.X = X(:,1:k+1);
L.U = U(:,1:k);
end

function d = clearance(X,map)
d = zeros(1,size(X,2));
for k = 1:size(X,2)
    d(k) = min_dist(X(1:2,k),map.limits,map.obstacle);
end
end

function save_fig(f,file,type)
if nargin < 3, type = 'vector'; end
if strcmp(type,'image')
    exportgraphics(f,file,'ContentType','image','Resolution',300);
else
    exportgraphics(f,file,'ContentType','vector');
end
fprintf('Saved %s\n',file);
end
