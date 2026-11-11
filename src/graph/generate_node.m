function node = generate_node(center, q_closest)
% Square funnel around center with one edge touching the closest obstacle
% point q_closest (tangential orientation): the center-to-q_closest vector,
% rotated by -45 degrees, spans the square.
node.q = center;
node.center = center;
vector1 = q_closest - center;
vector1 = [cos(-pi/4) -sin(-pi/4); sin(-pi/4) cos(-pi/4)]*vector1;
vector2 = [0 -1; 1 0]*vector1;
corner1 = center + vector1;
corner2 = center + vector2;
corner3 = center - vector1;
corner4 = center - vector2;
node.q_closest = corner1;
node.box = [corner1 corner2 corner3 corner4];
node.scale = norm(vector1) * sqrt(2);
node.angle = angle([1 1i]*vector1) + 3*pi/4;
end
