function xn = current_step(model,x,u,Vc)
% One step of the controller model in a constant current Vc (world frame,
% [m/s]), the same structure as the plant of make_plant.m: model.step acts
% on the velocity relative to the water, the vessel drifts with Vc, and x
% (in and out) holds the velocity over ground.
z = x(:);
z(4:5) = z(4:5) - Rt(z(3))*Vc;
z = model.step(z,u) + [model.h*Vc; 0; 0; 0; 0];
xn = z;
xn(4:5) = z(4:5) + Rt(z(3))*Vc;
end

function R = Rt(psi)
% world -> body rotation (planar)
R = [cos(psi) sin(psi); -sin(psi) cos(psi)];
end
