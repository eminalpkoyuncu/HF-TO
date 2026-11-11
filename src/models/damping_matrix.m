function D = damping_matrix(vVec,dampVec,absVec)
% Linear plus quadratic damping of the Fossen model, with the sign convention
% of the hydrodynamic derivatives (model_fossen.m uses -D); dampVec =
% [Xu; Yv; Yr; Nv; Nr], absVec = [X|u|u; Y|v|v; N|r|r].
Dl = [dampVec(1)         0               0;...
         0           dampVec(2)      dampVec(3);...
         0           dampVec(4)      dampVec(5)];

Dn = [absVec(1)*abs(vVec(1))         0                       0;...
                0           absVec(2)*abs(vVec(2))           0;...
                0                    0              absVec(3)*abs(vVec(3))];

D = Dl + Dn;
end
