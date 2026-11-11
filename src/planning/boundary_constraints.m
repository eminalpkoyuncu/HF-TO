function [Aeq,beq] = boundary_constraints(xinit,xbetween,xfinal,status,N1,N2)
% Equality constraints of a TO problem on the decision vector
% [x(1:N) y psi u v r F1 F2], N = N1+N2: the initial state xinit (6 states),
% and either the waypoint xbetween at step N1 and the position xfinal at
% step N ("BETWEEN", two regions), or the full state xfinal at step N
% ("FINAL", last region).
A = [1 zeros(1,((N1+N2)-1))];
Aprime = [zeros(1,((N1+N2)-1)) 1];
Abetween = [zeros(1,(N1-1)) 1 zeros(1,N2)];

if status == "FINAL"
    Aeq = [blkdiag(A,A,A,A,A,A) zeros(6,2*N1);blkdiag(Aprime,Aprime,Aprime,Aprime,Aprime,Aprime) zeros(6,2*N1)];
    beq = [xinit(1:6);xfinal(1:6)];
elseif status == "BETWEEN"
    Aeq = [blkdiag(A,A,A,A,A,A) zeros(6,2*(N1+N2));blkdiag(Aprime,Aprime) zeros(2,6*(N1+N2));blkdiag(Abetween,Abetween),zeros(2,6*(N1+N2))];
    beq = [xinit(1:6);xfinal(1:2);xbetween(1:2)];
end
end
