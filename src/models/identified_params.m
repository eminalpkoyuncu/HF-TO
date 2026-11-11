function thetaHat = identified_params
% Identified Heron model parameters (heron_identified_params.csv), read once
% and cached.
persistent cached
if isempty(cached)
    cached = readmatrix(fullfile(fileparts(mfilename('fullpath')),'heron_identified_params.csv'));
end
thetaHat = cached;
end
