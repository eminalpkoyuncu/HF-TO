function [K,P] = tvlqr_gains(nom,model,Qx,Ru,Pf)
% Time-varying LQR gains along the nominal (x*_k, u*_k), k = 1..T, for the
% error dynamics e_{k+1} = A_k e_k + B v_k, A_k = jacA(x*_k):
%   P_T = Pf,  K_k = -(Ru + B'P_{k+1}B)^{-1} B'P_{k+1}A_k,
%   P_k = Qx + A_k'P_{k+1}(A_k + B K_k).
% K is 2x6xT (the ancillary feedback u = u*_k + K_k (x - x*_k)).
T = size(nom.x,2);
B = model.B;
K = zeros(size(B,2),size(B,1),T);
P = Pf;
for k = T:-1:1
    A = model.jacA(nom.x(:,k));
    K(:,:,k) = -(Ru + B.'*P*B) \ (B.'*P*A);
    P = Qx + A.'*P*(A + B*K(:,:,k));
    P = (P+P.')/2;
end
end
