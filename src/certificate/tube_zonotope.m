function tube = tube_zonotope(nom,model,K,opt)
% Zonotopic outer bound of the tracking error e_k = x_k - x*_k, paper
% Eq. (errdyn), under the ancillary law u_k = u*_k + K_k e_k + c_k,
% |c_k| <= opt.cmax (the MPC correction), and the thrust disturbance
% F_real = u + w, |w_i| <= opt.level*|u_i|:
%   e_{k+1} = (A_k + B K_k) e_k + B (c_k + w_k) + delta_k,
%   |delta_k| <= (L_k/2) |e_k|^2   (Taylor remainder; dynamics affine in u),
% with L_k a Lipschitz constant of model.jacA, estimated by sampling
% (opt.nL samples). opt.G0 (optional) generates the initial error set.
% The error set is the zonotope
%   Z_k = {G_k xi : |xi|_inf <= 1},
% propagated without approximation for the linear part:
%   G_{k+1} = [ (A_k + B K_k) G_k ,  B diag(a_k) ,  rho_k I ],
% where a_k bounds the input disturbance and rho_k the Taylor remainder
% (a ball of radius rho_k lies in the box rho_k*[-1,1]^n). Supports are
% exact: max_{e in Z} c'e = c'z_k + |G' c|_1. No wrapping effect. The center
% z_k follows the plan defect d_k = f(x*_k,u*_k) - x*_{k+1}:
%   z_{k+1} = (A_k + B K_k) z_k + d_k.
% opt.modelErr (optional) = @(k,beta_k,du_k) returns [eps0, rad]: the
% one-step difference eps0 between the plant and model.step at the nominal
% (x*_k, u*_k), which shifts the center like the plan defect, and a bound
% rad on how much that difference varies over x*_k + box(beta_k) and thrust
% deviations |F - u*_k| <= du_k, which is added to the remainder ball
% (Fossen: ode45 plant vs Euler model).
T = size(nom.x,2);
n = size(nom.x,1);
B = model.B;
if ~isfield(opt,'cmax'), opt.cmax = [0;0]; end
if ~isfield(opt,'G0'), opt.G0 = zeros(n,0); end
if ~isfield(opt,'nL'), opt.nL = 30; end

G = opt.G0;
z = zeros(n,T);
Gpos = cell(1,T);
beta = zeros(n,T);      % |e_i| <= beta_i (box hull)
r = zeros(1,T);
L = zeros(1,T);
uHalf = zeros(2,T);
wHalf = zeros(2,T);
rng(0);
for k = 1:T
    Gpos{k} = G(1:2,:);
    beta(:,k) = abs(z(:,k)) + sum(abs(G),2);
    r(k) = norm(beta(:,k));
    uHalf(:,k) = abs(K(:,:,k)*z(:,k)) + sum(abs(K(:,:,k)*G),2);
    if k == T
        break;
    end
    A = model.jacA(nom.x(:,k));
    Acl = A + B*K(:,:,k);
    wHalf(:,k) = opt.level*(abs(nom.u(:,k)) + uHalf(:,k) + opt.cmax);
    a = wHalf(:,k) + opt.cmax;
    L(k) = lipschitz_jac(model,nom.x(:,k),max(r(k),1e-3),opt.nL);
    rho = 0.5*L(k)*r(k)^2;
    eps0 = zeros(n,1);
    if isfield(opt,'modelErr')
        [eps0,rad] = opt.modelErr(k,beta(:,k),uHalf(:,k) + a);
        rho = rho + rad;
    end
    if ~isfinite(rho) || ~all(isfinite(a))
        % diverged: mark the rest of the tube huge so the certificate fails
        % (finite, because 0*inf = NaN would be skipped by min in certify_plan)
        big = 1e9;
        for kk = k+1:T
            Gpos{kk} = big*eye(2);
        end
        z(:,k+1:T) = 0;
        beta(:,k+1:T) = big;
        r(k+1:T) = big;
        uHalf(:,k+1:T) = big;
        break;
    end
    G = [Acl*G, B*diag(a), rho*eye(n)];
    z(:,k+1) = Acl*z(:,k) + (model.step(nom.x(:,k),nom.u(:,k)) - nom.x(:,k+1)) + eps0;
    G(:,all(G == 0,1)) = [];
end
tube.type = 'zonotope';
tube.Gpos = Gpos;
tube.c = z;
tube.beta = beta;
tube.r = r;
tube.L = L;
tube.uHalf = uHalf;
tube.wHalf = wHalf;
tube.opt = opt;
tube.posSup = @(k,C) C*z(1:2,k) + sum(abs(C*Gpos{k}),2);
tube.velRad = @(k) norm(beta(4:6,k));
end

function L = lipschitz_jac(model,x,rad,m)
% sampled estimate of max |jacA(x+d) - jacA(x)|_2 / |d| over |d| <= rad
A0 = model.jacA(x);
L = 0;
for i = 1:m
    d = randn(size(x));
    d = d/norm(d)*rad*(0.5+0.5*rand);
    L = max(L, norm(model.jacA(x+d)-A0)/norm(d));
end
end
