function cfg = default_config(override)
% Settings shared by the HF-TO and MPC-Graph experiments. Both methods read
% the same struct, so the map, funnels, plant, thrust limits, disturbance
% and success criterion are identical. Fields of override replace the
% defaults; override.preset = 'gazebo' selects the Heron/Gazebo settings.
if nargin < 1 || isempty(override)
    override = struct;
end

% ---------------------------------------------------------------
% Monte Carlo defaults (paper Section 4)
cfg.preset = 'mce';
cfg.mapNo = 13;             % paper Map 1 = 3, Map 2 = 13, Gazebo = 14
cfg.model = 'fossen';       % 'fossen' (known model) or 'identified'
cfg.nRuns = 500;
cfg.seed = 1;               % funnel generation; run k uses seed+k
cfg.noiseLevel = 1;         % F_real = F + level*|F|*n, n ~ U[-1,1]
cfg.paramPert = 0;          % plant parameters scaled by 1 + paramPert*U[-1,1] (make_plant.m)
cfg.current = 0;            % [m/s] mean speed of a constant current, per run:
cfg.currentStd = 0;         %   speed N(current,currentStd^2), direction U[0,2pi)
cfg.est.on = false;         % current estimate in both MPCs (with_current_estimator.m)
cfg.est.q = 1e-7;           % random-walk variance of Vc per step [(m/s)^2]
cfg.est.r = [];             % residual variances [x y psi u v r]; [] = per model
                            % (closed-loop residual std at noise level 1)
cfg.est.p0 = 0.01;          % initial variance of Vc [(m/s)^2] (prior std 0.1 m/s)
cfg.est.rk4 = true;         % residual with RK4 of model.f when available
cfg.est.vMax = 1.5;         % [m/s] estimate clipped to this speed
cfg.est.mpcQ = diag([50 50 15 0.1 0.1 0.1]);  % HF-TO MPC weight with the estimate
                            % (lower heading weight: the vessel may crab)
cfg.goalTol = 0.5;         % [m] success if final position error <= goalTol
cfg.stopOnExit = true;      % stop a run when it leaves the funnel regions
cfg.h = 0.1;                % TO and MPC step [s]
cfg.uLim = [-10 20];        % thrust limits of plant and both controllers [N]
                            % (Fossen model; identified model: see below)
cfg.posMargin = 0.15;       % [m] funnels shrunk by this in the TO and MPC-Graph
cfg.armB = 0.26;            % thruster arm [m], for the energy metric
cfg.useParallel = true;     % parfor over runs (controller timing less reliable)
cfg.resultsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');

% HF-TO: TO and tracking MPC
cfg.Nmpc = 40;              % MPC horizon (steps)
cfg.mpcQ = diag([50 50 150 0.1 0.1 0.1]);   % MPC state weight [x y psi u v r]
cfg.mpcR = 0.1*eye(2);      % MPC input weight
cfg.tExtra = 0;             % [s] run time beyond the end of the nominal
cfg.duMax = 10;             % TO thrust rate limit per step [N]
cfg.vnom = 1;               % planning speeds [m/s]: middle, first, last segment
cfg.vthr = 0.5;
cfg.vpark = 0.3;
cfg.tSlack = 1.1;           % segment duration = tSlack*length/speed
cfg.maxPlanFails = 3;
cfg.reusePlan = true;       % load the nominal from planFile if it matches
cfg.toMaxIter = 10000;      % fmincon iteration cap
cfg.toCheckGradients = false;
cfg.toHessian = 'exact';    % Hessian of the Lagrangian: 'exact' or 'lbfgs'
cfg.toConTol = 1e-4;        % constraint tolerance
cfg.toJoinRate = true;      % thrust rate limit also between consecutive TO problems

% MPC-Graph on the USV
cfg.mg.N = 40;              % horizon (steps), same as HF-TO by default
cfg.mg.Q = diag([1 1 1 5 5 5]);  % [x y psi u v r]; heading weight on the line of
                            % sight (ctrl_mpcgraph.m) chosen on seeds 10001-10040
                            % (tune_mpcgraph_heading.m; not used in the MCE)
cfg.mg.R = 1e-2*eye(2);
cfg.mg.losMin = 0.5;        % [m] no heading reference closer to the outlet
cfg.mg.lin = 'trajectory';  % 'trajectory' (LTV), 'current' or 'outlet' (LTI)
cfg.mg.terminal = 'dare';   % terminal cost from the DARE ('dare') or Q ('none')
cfg.mg.qpMaxIter = 200;     % quadprog iteration cap (avoids rare stalls)
cfg.mg.vStop = 0.1;         % [m/s] arrived when within goalTol and slower
cfg.mg.tMax = [];           % [s] run time; [] = tMaxFactor*duration of our nominal
cfg.mg.tMaxFactor = 3;
cfg.mg.tMaxDefault = 120;   % used if no HF-TO plan exists

% Gazebo
cfg.gz.rate = 10;           % control rate [Hz]
cfg.gz.masterUri = "http://localhost:11311";   % gazebo/ container (host network)
cfg.gz.ip = "127.0.0.1";
cfg.gz.origin = [];         % map pose [x;y;psi] of the Gazebo origin; [] = [start;0]

% ---------------------------------------------------------------
preset = 'mce';
if isfield(override,'preset')
    preset = override.preset;
end
if strcmp(preset,'gazebo')
    % Heron in Gazebo
    cfg.preset = 'gazebo';
    cfg.mapNo = 14;
    cfg.model = 'identified';
    cfg.nRuns = 1;
    cfg.noiseLevel = 0;
    cfg.stopOnExit = false;
    cfg.vnom = 2.2;             % mean speed of the nominal about 1.7 m/s
    cfg.vthr = 1.2;
    cfg.vpark = 0.8;
end

for fn = fieldnames(override).'
    if isstruct(override.(fn{1})) && isfield(cfg,fn{1}) && isstruct(cfg.(fn{1}))
        for sub = fieldnames(override.(fn{1})).'
            cfg.(fn{1}).(sub{1}) = override.(fn{1}).(sub{1});
        end
    else
        cfg.(fn{1}) = override.(fn{1});
    end
end

% derived settings, unless given explicitly
if strcmp(cfg.model,'identified')
    % Heron thrusters (the Fossen model is a smaller, scaled vessel)
    if ~isfield(override,'uLim'), cfg.uLim = [-19.88 33.6]; end
    if ~isfield(override,'uTO'), cfg.uTO = [-15 28]; end
    if ~isfield(override,'duMax'), cfg.duMax = 45; end
end
if ~isfield(cfg,'uTO')
    cfg.uTO = 0.85*cfg.uLim;    % tightened TO thrust limits, margin for feedback
end
if ~isfield(override,'V')
    switch cfg.mapNo
        case 3
            cfg.V = struct('u',[-0.5 4],'r',[-3 3]);
        case 13
            cfg.V = struct('u',[-0.6 3],'r',[-2 2]);
        otherwise
            cfg.V = struct('u',[-0.8 3],'r',[-1.5 1.5]);
    end
end
if isempty(cfg.est.r)
    switch cfg.model
        case 'fossen'
            cfg.est.r = [2.5e-3 2.5e-3 1.2e-2 3e-2 2.5e-2 1.5e-1].^2;
        otherwise
            cfg.est.r = [1e-3 1e-3 1e-3 3.2e-2 1e-3 3.3e-2].^2;
    end
end
if ~isfield(cfg,'planFile')
    cfg.planFile = fullfile(cfg.resultsDir,sprintf('plan_map%d_%s.mat',cfg.mapNo,cfg.model));
end
end
