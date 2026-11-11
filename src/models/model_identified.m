function model = model_identified(h,th)
% Identified discrete-time Heron model (identified_step.m, heron_identified_params.csv).
% It was identified at h = 0.1 s and is only valid for that step. The plant
% is the same discrete map (no continuous model available). th (optional)
% replaces the identified parameters (plant mismatch, make_plant.m).
if abs(h-0.1) > 1e-12
    error('The identified model is only valid for h = 0.1 s.');
end
if nargin < 2
    th = identified_params;
end

model.name = 'identified';
model.h = h;
model.b = [];
model.f = [];
model.step = @(x,u) identified_step(x(:),u(1),u(2),th,h);
model.jacA = @(x) identified_jacA(x,h,th);
model.B = identified_B(th);
end
