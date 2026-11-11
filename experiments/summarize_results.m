function T = summarize_results(mapNo,mode,model)
% Side-by-side table of HF-TO and MPC-Graph results from cfg.resultsDir,
% e.g. summarize_results(13) or summarize_results(14,'gazebo','identified').
% For Gazebo, the latest result file of each method is used.
if nargin < 2, mode = 'mce'; end
if nargin < 3
    model = 'fossen';
    if strcmp(mode,'gazebo'), model = 'identified'; end
end
dir0 = default_config().resultsDir;
methods = {'hfto','mg'};
labels = {'HF-TO','MPC-Graph'};
rows = {};
for i = 1:2
    files = dir(fullfile(dir0,sprintf('%s_%s_map%d_%s*.mat',methods{i},mode,mapNo,model)));
    if isempty(files)
        warning('No %s results for map %d (%s).',labels{i},mapNo,mode);
        continue;
    end
    [~,j] = max([files.datenum]);
    S = load(fullfile(files(j).folder,files(j).name));
    if isfield(S,'results'), r = S.results; else, r = S.res; end
    s = summarize(r,S.cfg);
    s.method = string(labels{i});
    rows{end+1} = s; %#ok<AGROW>
end
T = struct2table([rows{:}]);
T = movevars(T,'method','Before',1);
disp(T);
end

function s = summarize(r,cfg)
succ = [r.success];
s.runs = numel(r);
s.success_pct = 100*mean(succ);
s.left_regions = sum([r.reason] == "left_regions");
s.goal_not_reached = sum([r.reason] == "goal_not_reached");
s.min_clear_m = min([r.minClear]);
s.median_clear_m = median([r.minClear]);
s.time_to_goal_s = mean([r(succ).tGoal]);
s.path_len_m = mean([r(succ).pathLen]);
s.final_err_m = mean([r(succ).finalErr]);
s.effort_N2s = mean([r.effort]);
s.energy_J = mean([r.energy]);
s.cpu_mean_ms = 1e3*mean([r.cpuMean]);
s.cpu_max_ms = 1e3*max([r.cpuMax]);
s.qp_fail_steps = sum([r.qpFails]);
s.noise = cfg.noiseLevel;
end
