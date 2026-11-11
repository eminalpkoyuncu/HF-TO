function plant = make_plant(model,cfg,seed)
% Plant of one Monte Carlo run. The controllers keep using model; the plant
% can differ from it by
%   cfg.paramPert: every model parameter scaled by 1 + paramPert*n,
%       n ~ U[-1,1] per parameter and run (identified model: the diagonal
%       coefficients th(1), th(6), th(14) are perturbed through th-1, the
%       continuous-time damping times h, so the plant stays stable),
%   cfg.current, cfg.currentStd: constant current with speed
%       N(current, currentStd^2) [m/s] (clipped at 0) and direction uniform,
%       drawn per run. The plant integrates the velocity relative to the water,
%       nu_r = nu - R(psi)'*Vc, and drifts with Vc; the state it returns has
%       the velocity over ground (what GPS/odometry would measure).
% The random draws use their own streams, so the thrust noise sequence of
% run seed is the same in every case. plant.advance(x,u,t) returns the
% next state and the points (and times) at which the regions are checked.
plant.paramScale = [];
plant.currentDir = NaN;
plant.currentSpeed = 0;
plant.Vc = [0;0];
pm = model;
if cfg.paramPert > 0
    rs = RandStream('mt19937ar','Seed',seed + 100000);
    switch model.name
        case 'fossen'
            s = 1 + cfg.paramPert*(2*rand(rs,14,1)-1);
            pm = model_fossen(model.h,s);
        case 'identified'
            th = identified_params;
            d = th;
            dia = [1 6 14];
            d(dia) = th(dia) - 1;
            s = 1 + cfg.paramPert*(2*rand(rs,numel(th),1)-1);
            d = d.*s;
            th = d;
            th(dia) = 1 + d(dia);
            pm = model_identified(model.h,th);
    end
    plant.paramScale = s;
end
if cfg.current > 0
    rs = RandStream('mt19937ar','Seed',seed + 200000);
    plant.currentDir = 2*pi*rand(rs);
    plant.currentSpeed = max(cfg.current + cfg.currentStd*randn(rs),0);
    plant.Vc = plant.currentSpeed*[cos(plant.currentDir); sin(plant.currentDir)];
end
Vc = plant.Vc;
h = model.h;
odeopts = odeset('RelTol',1e-6,'AbsTol',1e-8);
plant.advance = @advance;

    function [x,pts,tp] = advance(x,u,t)
        z = x;
        z(4:5) = x(4:5) - Rt(x(3))*Vc;
        if isempty(pm.f)
            z = pm.step(z,u) + [h*Vc; 0; 0; 0; 0];
            pts = z(1:2);
            tp = t+h;
        else
            [ts,zs] = ode45(@(tt,zz) pm.f(zz,u) + [Vc; 0; 0; 0; 0],[t t+h],z,odeopts);
            z = zs(end,:).';
            pts = zs(2:end,1:2).';
            tp = ts(2:end).';
        end
        x = z;
        x(4:5) = z(4:5) + Rt(z(3))*Vc;
    end
end

function R = Rt(psi)
% world -> body rotation (planar)
R = [cos(psi) sin(psi); -sin(psi) cos(psi)];
end
