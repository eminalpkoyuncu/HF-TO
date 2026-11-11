function root = setup()
% Adds the code of this repository to the MATLAB path and returns its root
% folder. Run it once per session from the repository root; the experiment
% scripts call it themselves.
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root,'config'), fullfile(root,'experiments'), ...
    genpath(fullfile(root,'src')), fullfile(root,'baselines','mpc_graph'));
end
