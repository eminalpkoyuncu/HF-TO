function cert = certify_plan(nom,tube,cfg,rv)
% Safety certificate of a nominal plan, paper Proposition (cert), with the
% error tube of tube_zonotope (tube.posSup(k,C), tube.velRad(k)).
% Position: for every step k there must be a region R (the region of sample
% k or of sample k+1) such that both tube cross-sections x*_k + E(Q_k) and
% x*_{k+1} + Z_{k+1}, grown by the vessel radius rv and the inter-sample
% bulge d_k = h^2 a_k / 8, lie in R. For R = {p : C p <= b} this is, row
% by row, with the support function s_Z(c) = max_{e in Z} c'e_pos,
%   c_i' p* + s_Z(c_i) + (rv + d_k) |c_i| <= b_i.
% Input: u*_k +- (|K_k e_k|max + cmax) must lie within cfg.uLim.
% cert.ok is true when all slacks are >= 0.
T = size(nom.x,2);
h = nom.t(2) - nom.t(1);
cmax = tube.opt.cmax;

% inter-sample bulge from an acceleration bound over the tube
rvel = zeros(1,T);
for k = 1:T
    rvel(k) = tube.velRad(k);
end
dint = zeros(1,T-1);
for k = 1:T-1
    nu = nom.x(4:6,k:k+1);
    sp = max(vecnorm(nu(1:2,:))) + max(rvel(k:k+1));   % |(u,v)| bound
    yr = max(abs(nu(3,:))) + max(rvel(k:k+1));         % |r| bound
    dnu = norm(nu(:,2)-nu(:,1)) + rvel(k) + rvel(k+1); % |nu_{k+1}-nu_k| bound
    a = dnu/h + yr*sp;                                 % |p''| <= |nu'| + |r||nu|
    dint(k) = h^2*a/8;
end

slackPos = zeros(1,T-1);
for k = 1:T-1
    best = -inf;
    for kr = [k k+1]
        C = nom.Aboxdata(:,:,kr);
        b = nom.Bboxdata(:,:,kr);
        s = inf;
        for kk = [k k+1]
            p = nom.x(1:2,kk);
            sup = tube.posSup(kk,C);
            s = min(s, min((b - C*p - sup - (rv + dint(k))*vecnorm(C,2,2))./vecnorm(C,2,2)));
        end
        best = max(best,s);
    end
    slackPos(k) = best;
end

up = nom.u + tube.uHalf + cmax;
lo = nom.u - tube.uHalf - cmax;
slackU = min(cfg.uLim(2) - up, lo - cfg.uLim(1));   % 2xT

cert.slackPos = slackPos;       % [m], distance margin left at each step
cert.slackU = slackU;           % [N]
cert.dint = dint;
cert.minPos = min(slackPos);
cert.minU = min(slackU,[],'all');
cert.ok = cert.minPos >= 0 && cert.minU >= 0;
end
