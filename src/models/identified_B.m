function B = identified_B(thetaHat)
% Jacobian of the discrete-time model in identified_step.m w.r.t. the input.
% thetaHat defaults to the identified Heron model (heron_identified_params.csv).
if nargin < 1
    thetaHat = identified_params;
end
th = thetaHat;

B = [0,      0
     0,      0
     0,      0
     th(5),  th(5)
     th(12), -th(12)
     th(19), -th(19)];
end
