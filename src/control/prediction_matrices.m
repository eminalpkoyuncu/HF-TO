function [G,H] = prediction_matrices(t0,h,N,At,B,tend)
% Prediction matrices of x_{j+1} = A_j x_j + B u_j over N steps:
% [x_0; ...; x_N] = H*x_0 + G*[u_0; ...; u_{N-1}], A_j = At(t0 + j*h).
% Beyond tend the system is linearized at the end of the nominal trajectory.
nx = size(B,1);
nu = size(B,2);
H = zeros(nx*(N+1),nx);
G = zeros(nx*(N+1),nu*N);
H(1:nx,:) = eye(nx);
for j = 1:N
    Aj = At(min(t0+(j-1)*h,tend));
    prev = nx*(j-1)+(1:nx);
    rows = nx*j+(1:nx);
    H(rows,:) = Aj*H(prev,:);
    G(rows,:) = Aj*G(prev,:);
    G(rows,nu*(j-1)+(1:nu)) = B;
end
end
