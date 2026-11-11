function [u,exitflag] = solve_ltv_mpc(A,B,t0,x0,h,N,unom,Aboxval,Bboxval,xppval,tend,opts)
% One step of the tracking LTV-MPC, paper Eq. (mpccontrol). In the error
% coordinates x = chi - chi_nom, u = F - F_nom,
%   min  sum_{j=0}^{N} x_j' Q x_j + sum_{j=0}^{N-1} u_j' R u_j
%   s.t. x_{j+1} = A_j x_j + B u_j + w_j,  x_0 = x0,
%        C_j (p_nom,j + p_j) <= b_j          (region of step j),
%        u_j + F_nom,j in [ulim(1), ulim(2)],
% solved as a QP in u. A(t) returns A_j at time t, unom(t) and xppval(t)
% the nominal thrust and state, Aboxval/Bboxval (timeseries) the region of
% each nominal sample. Beyond tend the nominal is held at its final value.
% opts: Q, R, ulim = [Fmin Fmax]; w (6xN, optional) is the affine term of
% the current estimate (ctrl_hfto.m). Returns the correction u_0 (zero if
% the QP fails, i.e. the nominal thrust is applied) and the quadprog flag.
qpoptions = optimoptions(@quadprog,'Display','off','MaxIterations',10000,'ConstraintTolerance',7e-2,'OptimalityTolerance',1e-4);
tpoints = t0:h:(t0+(N)*h);

[G,H] = prediction_matrices(t0,h,N,A,B,tend);
% free response of the affine term, [x_0; ...; x_N] += c
nx = size(B,1);
c = zeros(nx*(N+1),1);
if isfield(opts,'w') && ~isempty(opts.w)
    for j = 1:N
        Aj = A(min(t0+(j-1)*h,tend));
        c(nx*j+(1:nx)) = Aj*c(nx*(j-1)+(1:nx)) + opts.w(:,j);
    end
end
Q = opts.Q;
R = opts.R;
ulim = opts.ulim;

Qbar = kron(eye(N+1),Q);
Rbar = kron(eye(N),R);
M = G.'*Qbar*G + Rbar;
M = (M+M.')/2;
alphaT = (H*x0 + c).'*Qbar*G;

% thrust bounds on the total thrust
uactive = unom(min(tpoints(1:end-1),tend));
uactivetemp = zeros(2*N,1);
uactivetemp(1:2:end)  = uactive(1,:);
uactivetemp(2:2:end)  = uactive(2,:);
lb = ulim(1)-uactivetemp;
ub = ulim(2)-uactivetemp;

% the region of the predicted state x_j is the one active at time t0+j*h
tbox = min(tpoints(2:end),tend);
kbox = sum(Aboxval.Time(:) <= tbox + 1e-9,1);
Aboxvaltemp = Aboxval.Data(:,:,kbox);
Bboxvaltemp = Bboxval.Data(:,:,kbox);
nbox = size(Aboxvaltemp,1);
Aboxtemp = cat(2,Aboxvaltemp,zeros(nbox,4,N));
bbox = [0;reshape(Bboxvaltemp,[],1)];
Aboxtemp = mat2cell(Aboxtemp,nbox,6,ones(1,size(Aboxtemp,3)));
Abox = blkdiag(zeros(1,6),Aboxtemp{:});
xactive = reshape(xppval(min(tpoints,tend)),[],1);
Aineq = Abox*G;
b = bbox - Abox*(H*x0 + c) - Abox*xactive;

[sol,~,exitflag] = quadprog(M,alphaT,Aineq,b,[],[],lb,ub,[],qpoptions);
if ((exitflag ~= 0) && (exitflag ~= 1))
    u = [0;0];
else
    u = sol(1:2);
end
end
