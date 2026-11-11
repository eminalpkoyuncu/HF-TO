function [nom,stats] = plan_trajectory(map,nodes,path,model,cfg)
% Receding-horizon trajectory optimization inside the funnel sequence,
% paper Section 2.3, Eq. (optproblemplan). For region i, a trajectory through
% regions i and i+1 (look-ahead of two regions) is optimized by direct
% transcription, x = [x(1:n) y psi u v r F1 F2], with
%   cost         h*sum(F1^2 + F2^2),
%   dynamics     x_{j+1} = model.step(x_j,u_j)  (exact sparse Jacobian),
%   regions      each step constrained to its region, tightened by
%                cfg.posMargin (tighten below),
%   limits       velocities cfg.V, thrusts cfg.uTO, rate cfg.duMax, also
%                across the joins when cfg.toJoinRate,
%   waypoints    outlet of region i and of region i+1 (the goal state in
%                the last region),
% and only the part in region i is kept. Segment durations follow from the
% distances and the planning speeds cfg.vthr, cfg.vnom, cfg.vpark (times
% cfg.tSlack). fmincon interior point with the exact Hessian of the
% Lagrangian (cfg.toHessian = 'exact'; the dynamics are affine in the
% thrusts) or L-BFGS ('lbfgs'); a feasible solution at the iteration cap is
% accepted. Each problem is warm-started with the discarded second part of
% the previous solution. A failed problem is retried with the speeds scaled
% by 0.75, up to cfg.maxPlanFails failures in total.
% nom: x (6xT), u (2xT), t, and Aboxdata/Bboxdata, the region of every
% sample (C p <= b). stats: solve times, iterations and failed attempts.
h = model.h;
options = optimoptions(@fmincon,'Algorithm','interior-point','MaxIterations',cfg.toMaxIter, ...
    'MaxFunctionEvaluations',inf,'ConstraintTolerance',cfg.toConTol,'StepTolerance',1e-6, ...
    'OptimalityTolerance',1e-3,'SpecifyObjectiveGradient',true, ...
    'SpecifyConstraintGradient',true, ...
    'CheckGradients',cfg.toCheckGradients,'FiniteDifferenceType','central', ...
    'Display','off');   % central differences only matter for CheckGradients

start = [map.start(:); zeros(3-numel(map.start),1)];
goal = [map.goal(:); zeros(3-numel(map.goal),1)];
xgoal = [goal; 0; 0; 0];
xinit = [start; 0; 0; 0; 0; 0];

current_node = nodes{2};
next_node = nodes{current_node.path};
predicted_node = nodes{next_node.path};

xVec = {};
Aboxdata = [];
Bboxdata = [];
stats.segTime = [];
stats.segRetries = [];
stats.attemptTime = [];
stats.segIter = [];
stats.attempt = zeros(0,5);   % [region exitflag iterations violation speed scale]
stats.ok = true;
fails = 0;
retries = 0;
scale = 1;
Xtail = [];     % second part of the last accepted solution (8 x n)
uPrev = [];     % last kept thrust before the join (rate limit across joins)
i = 1;
while i <= length(path)
    vnom = scale*cfg.vnom;
    vthr = scale*cfg.vthr;
    vpark = scale*cfg.vpark;
    if current_node.depth ~= 0
        xbetween = get_ref(current_node,next_node,cfg.REF);
        if next_node.depth ~= 0
            xfinal = get_ref(next_node,predicted_node,cfg.REF);
        else
            xfinal = xgoal;
        end
        [cnstrnt_pos1, bnd_pos1] = region_halfspaces(current_node.box,xinit(1:2));
        [cnstrnt_pos2, bnd_pos2] = region_halfspaces(next_node.box,xbetween(1:2));
        bnd_pos2 = tighten(cnstrnt_pos2,bnd_pos2,cfg.posMargin,xbetween,xfinal);

        v1 = vnom;
        if i == 1
            v1 = vthr;
        end
        v2 = vnom;
        if next_node.depth == 0
            v2 = vpark;
        end
        N1 = round(cfg.tSlack*norm(xinit(1:2)-xbetween(1:2))/v1/h);
        N2 = round(cfg.tSlack*norm(xfinal(1:2)-xbetween(1:2))/v2/h);

        Abox = [blkdiag(kron(speye(N1),sparse(cnstrnt_pos1(:,1))),kron(speye(N2),sparse(cnstrnt_pos2(:,1)))) ...
                blkdiag(kron(speye(N1),sparse(cnstrnt_pos1(:,2))),kron(speye(N2),sparse(cnstrnt_pos2(:,2)))) ...
                sparse(size(cnstrnt_pos1,1)*(N1+N2),(N1+N2)*6)];
        Bbox = [repmat(tighten(cnstrnt_pos1,bnd_pos1,cfg.posMargin,xinit,xbetween),N1,1); repmat(bnd_pos2,N2,1)];
        [Aeq,beq] = boundary_constraints(xinit,xbetween,xfinal,"BETWEEN",N1,N2);

        uguess = xinit(7:8);
        if i == 1
            uguess = min(15,cfg.uTO(2))*[1; 1];
        end
        G2 = leg_guess(xbetween,xfinal,N2,uguess);
        if ~isempty(Xtail) && retries == 0
            % first part = what the previous segment planned to do here
            G1 = resample_cols(Xtail,N1);
            G1(:,1) = xinit;
        else
            G1 = leg_guess(xinit,xbetween,N1,uguess);
            G1(3:6,1) = xinit(3:6);
        end
        G2(3,:) = unwrap_near(G2(3,:),G1(3,end));
        G = [G1 G2];
        nbox = N1-1;
    else
        % goal heading taken modulo 2*pi, nearest to the arrival heading
        % (the TO heading is not wrapped, so it can be off by full turns)
        xfinal = xgoal;
        xfinal(3) = xgoal(3) + 2*pi*round((xinit(3)-xgoal(3))/(2*pi));
        [cnstrnt_pos1, bnd_pos1] = region_halfspaces(current_node.box,xinit(1:2));
        N1 = round(cfg.tSlack*norm(xinit(1:2)-xfinal(1:2))/vpark/h);
        N2 = 0;

        Abox = [kron(speye(N1),sparse(cnstrnt_pos1(:,1))) kron(speye(N1),sparse(cnstrnt_pos1(:,2))) ...
                sparse(size(cnstrnt_pos1,1)*N1,N1*6)];
        Bbox = repmat(tighten(cnstrnt_pos1,bnd_pos1,cfg.posMargin,xinit,xfinal),N1,1);
        [Aeq,beq] = boundary_constraints(xinit,0,xfinal,"FINAL",N1,0);

        % straight interpolation of all states
        G = [interp_cols(xinit(1:6),xfinal(1:6),N1); repmat(xinit(7:8),1,N1)];
        nbox = N1;
    end
    Ntraj = N1 + N2;
    x0 = reshape(G.',1,[]);     % layout [x(1:N) y psi u v r F1 F2]

    % input rate limit |F(k+1)-F(k)| <= duMax
    D1 = spdiags([ones(Ntraj,1) -ones(Ntraj,1)],0:1,Ntraj-1,Ntraj);
    A1 = kron(speye(2),D1);
    A = [sparse(2*(Ntraj-1),6*Ntraj) A1; sparse(2*(Ntraj-1),6*Ntraj) -A1; Abox];
    b = [cfg.duMax*ones(4*(Ntraj-1),1); Bbox];

    lb = [-inf(3*Ntraj,1); cfg.V.u(1)*ones(Ntraj,1); -inf(Ntraj,1); cfg.V.r(1)*ones(Ntraj,1); cfg.uTO(1)*ones(2*Ntraj,1)];
    ub = [ inf(3*Ntraj,1); cfg.V.u(2)*ones(Ntraj,1);  inf(Ntraj,1); cfg.V.r(2)*ones(Ntraj,1); cfg.uTO(2)*ones(2*Ntraj,1)];
    if cfg.toJoinRate && ~isempty(uPrev)
        % |F(1) - uPrev| <= duMax: the thrust rate limit also holds across the
        % join, where F(1) replaces the input of the previous problem
        iF = [6 7]*Ntraj + 1;
        lb(iF) = max(lb(iF),uPrev - cfg.duMax);
        ub(iF) = min(ub(iF),uPrev + cfg.duMax);
    end
    x0 = min(max(x0,lb.'),ub.');

    fun = @(x) objective(x,Ntraj,h);
    nonlcon = @(x) dyncon(x,Ntraj,model);
    if strcmp(cfg.toHessian,'exact')
        opts = optimoptions(options,'HessianFcn',@(x,lambda) lagrangian_hessian(x,lambda,Ntraj,h,model));
    else
        opts = optimoptions(options,'HessianApproximation','lbfgs');
    end

    tic
    [x,~,exitflag,output] = fmincon(fun,x0,A,b,sparse(Aeq),beq,lb,ub,nonlcon,opts);
    tsolve = toc;
    stats.attemptTime(end+1) = tsolve;
    stats.attempt(end+1,:) = [i exitflag output.iterations output.constrviolation scale];

    % a feasible solution at the iteration cap is accepted: the nominal
    % must satisfy the constraints, optimality to 1e-3 is not needed
    if exitflag > 0 || (exitflag == 0 && output.constrviolation <= options.ConstraintTolerance)
        Aboxdata = cat(3,Aboxdata,repmat(cnstrnt_pos1,1,1,nbox));
        Bboxdata = cat(3,Bboxdata,repmat(bnd_pos1,1,1,nbox));
        X = reshape(x(1:8*Ntraj),Ntraj,8).';
        xVec{end+1} = X(:,1:N1); %#ok<AGROW>
        xinit = X(:,N1);
        uPrev = X(7:8,N1-1);
        Xtail = X(:,N1:end);
        stats.segTime(end+1) = tsolve;
        stats.segRetries(end+1) = retries;
        stats.segIter(end+1) = output.iterations;
        retries = 0;
        scale = 1;
        current_node = next_node;
        next_node = nodes{current_node.path};
        predicted_node = nodes{next_node.path};
        i = i+1;
    else
        fails = fails+1;
        retries = retries+1;
        scale = 0.75*scale;
        if fails > cfg.maxPlanFails
            stats.ok = false;
            stats.failSeg = i;
            stats.failXinit = xinit;
            stats.failXfinal = xfinal;
            break;
        end
    end
end
stats.fails = fails;
stats.totalTime = sum(stats.attemptTime);

nom = struct;
if ~stats.ok
    return;
end
X = xVec{1};
for j = 2:numel(xVec)
    % the join sample is the first sample of segment j: take its input (the
    % previous segment's input there belongs to its discarded second part)
    X(7:8,end) = xVec{j}(7:8,1);
    X = [X xVec{j}(:,2:end)]; %#ok<AGROW>
end
nom.x = X(1:6,:);
nom.u = X(7:8,:);
nom.t = (0:size(X,2)-1)*h;
nom.Aboxdata = Aboxdata;
nom.Bboxdata = Bboxdata;
if size(Aboxdata,3) ~= numel(nom.t)
    error('Region data (%d) and nominal trajectory (%d) are misaligned.',size(Aboxdata,3),numel(nom.t));
end
end

function [f,g] = objective(x,N,h)
% h*sum(F1^2 + F2^2)
x = x(:);
iu = 6*N+1:8*N;
f = h*(x(iu).'*x(iu));
g = zeros(8*N,1);
g(iu) = 2*h*x(iu);
end

function [c,ceq,gc,gceq] = dyncon(x,N,model)
% x_{i+1} = step(x_i,u_i), with the sparse Jacobian
X = reshape(x(1:6*N),N,6).';
U = reshape(x(6*N+1:8*N),N,2).';
ceq = zeros(6*(N-1),1);
nnzr = 6*(N-1)*(1+6+2);
I = zeros(nnzr,1);
J = zeros(nnzr,1);
V = zeros(nnzr,1);
B = model.B;
p = 0;
for i = 1:N-1
    rows = 6*(i-1)+(1:6);
    ceq(rows) = X(:,i+1) - model.step(X(:,i),U(:,i));
    A = model.jacA(X(:,i));
    % d/dX(:,i+1) = I, d/dX(:,i) = -A, d/dU(:,i) = -B; column of state s at
    % time j is (s-1)*N+j, of input c at time j is 6N+(c-1)*N+j
    [r,s] = ndgrid(rows,1:6);
    k = p+(1:6);
    I(k) = rows; J(k) = (0:5)*N+i+1; V(k) = 1;
    p = p+6;
    k = p+(1:36);
    I(k) = r(:); J(k) = (s(:)-1)*N+i; V(k) = -A(:);
    p = p+36;
    [r,c] = ndgrid(rows,1:2);
    k = p+(1:12);
    I(k) = r(:); J(k) = 6*N+(c(:)-1)*N+i; V(k) = -B(:);
    p = p+12;
end
c = [];
gc = [];
gceq = sparse(J,I,V,8*N,6*(N-1));   % fmincon wants the transpose
end

function H = lagrangian_hessian(x,lambda,N,h,model)
% Hessian of h*sum(F^2) + lambda'*ceq. ceq_i = x_{i+1} - step(x_i,u_i) is
% affine in u, so its second derivative is the state block
% -d/dx_i (jacA(x_i)'*lambda_i), taken by central differences of jacA.
X = reshape(x(1:6*N),N,6).';
lam = reshape(lambda.eqnonlin,6,N-1);
d = 1e-5;
I = zeros(36*(N-1),1);
J = I;
V = I;
for i = 1:N-1
    Hi = zeros(6);
    for j = 1:6
        e = zeros(6,1);
        e(j) = d;
        Hi(:,j) = -(model.jacA(X(:,i)+e).'*lam(:,i) - model.jacA(X(:,i)-e).'*lam(:,i))/(2*d);
    end
    Hi = (Hi+Hi.')/2;
    [r,c] = ndgrid(1:6,1:6);
    k = 36*(i-1)+(1:36);
    I(k) = (r(:)-1)*N+i;
    J(k) = (c(:)-1)*N+i;
    V(k) = Hi(:);
end
iu = (6*N+1:8*N).';
H = sparse([I; iu],[J; iu],[V; 2*h*ones(2*N,1)],8*N,8*N);
end

function G = leg_guess(xa,xb,n,u)
% straight leg from xa to xb, heading along it, at rest velocities
G = zeros(8,n);
G(1:2,:) = interp_cols(xa(1:2),xb(1:2),n);
G(3,:) = atan2(xb(2)-xa(2),xb(1)-xa(1));
G(7:8,:) = repmat(u(:),1,n);
end

function Y = interp_cols(a,b,n)
Y = a(:) + (b(:)-a(:))*linspace(0,1,n);
end

function Y = resample_cols(X,n)
% resample the columns of X to n columns over the same span
t0 = linspace(0,1,size(X,2));
Y = interp1(t0,X.',linspace(0,1,n),'linear').';
if n == 1
    Y = Y(:);
end
end

function psi = unwrap_near(psi,ref)
psi = psi + 2*pi*round((ref-psi)/(2*pi));
end

function b = tighten(C,b,margin,varargin)
% shrink the polygon C*p <= b by margin (C rows need not be normalized),
% but never past the points (varargin) the segment is pinned to, so a
% narrow funnel overlap gets a smaller margin instead of an infeasible TO
P = cell2mat(cellfun(@(p) reshape(p(1:2),2,1),varargin,'UniformOutput',false));
b = max(b - margin*vecnorm(C,2,2), max(C*P,[],2));
end
