function vdotVec = identified_step(vVec,u1,u2,thetaHat,h)
% One step (h = 0.1 s) of the model identified from Gazebo data, paper
% Eq. (identified): forward-Euler kinematics and the NARX velocity update
% with the 19 parameters thetaHat. vVec is the state [x y psi u v r].
    vdotVec = zeros(6,1);
    x = vVec(1);
    y = vVec(2);
    phi = vVec(3);
    u = vVec(4);
    v = vVec(5);
    r = vVec(6);
    
    vdotVec(1) = x + h*(u*cos(phi) - v*sin(phi));
    vdotVec(2) = y + h*(u*sin(phi) + v*cos(phi));
    vdotVec(3) = phi + h*r;
    vdotVec(4) = thetaHat(1)*u + thetaHat(2)*r^2 + thetaHat(3)*v*r + thetaHat(4)*u*abs(u) + thetaHat(5)*(u1+u2) ;
    vdotVec(5) = thetaHat(6)*v + thetaHat(7)*r + thetaHat(8)*u*v + thetaHat(9)*u*r + thetaHat(10)*v*abs(v) + thetaHat(11)*r*abs(r) + thetaHat(12)*(u1-u2);
    vdotVec(6) = thetaHat(13)*v + thetaHat(14)*r + thetaHat(15)*u*v + thetaHat(16)*u*r + thetaHat(17)*v*abs(v) + thetaHat(18)*r*abs(r) + thetaHat(19)*(u1-u2);
end