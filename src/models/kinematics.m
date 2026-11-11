function etadotVec = kinematics(etaVec,vVec)
% Pose rate eta_dot = R(psi)*nu of the 3-DoF model, paper Eq. (stateanddynamics).
J = [cos(etaVec(3)) -sin(etaVec(3)) 0;...
     sin(etaVec(3)) cos(etaVec(3))  0;...
     0               0          1];
etadotVec = J*vVec;
end

