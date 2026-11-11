function new_node = enlarge(node, obstacles, limits)
% Grows the square funnel of generate_node into a rectangle: from the edge
% at the obstacle, first along that edge (factor 1.5 per step) and then
% away from it, each until the box would leave the arena or overlap an
% obstacle (obstacles: 2xN closed polygons separated by NaNs).
scale_ratio = 1.5;
new_node = node;
theta = node.angle;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
scale = node.scale;
hscale = scale;
vscale = scale;
new_node.hscale = hscale;
new_node.vscale = vscale;
box = zeros(2, 4);
ref = (node.box(:, 1)+node.box(:, 2))/2;

% along the obstacle
while true
    hscale = scale_ratio * hscale;
    box(:, 1) = ref + rot*[-hscale/2; 0];
    box(:, 2) = ref + rot*[hscale/2; 0];
    box(:, 3) = ref + rot*[hscale/2; scale];
    box(:, 4) = ref + rot*[-hscale/2; scale];
    if hits(box, obstacles, limits)
        break;
    end
    new_node.box = box;
    new_node.center = mean(box, 2);
    new_node.hscale = hscale;
end

% away from the obstacle
while true
    vscale = scale_ratio * vscale;
    box(:, 1) = ref + rot*[-new_node.hscale/2; 0];
    box(:, 2) = ref + rot*[new_node.hscale/2; 0];
    box(:, 3) = ref + rot*[new_node.hscale/2; vscale];
    box(:, 4) = ref + rot*[-new_node.hscale/2; vscale];
    if hits(box, obstacles, limits)
        break;
    end
    new_node.box = box;
    new_node.center = mean(box, 2);
    new_node.vscale = vscale;
end
end

function out = hits(box, obstacles, limits)
x1 = box(1,:);
y1 = box(2,:);
out = (sum(x1 < limits(1)) + sum(x1 > limits(2)) + ...
       sum(y1 < limits(3)) + sum(y1 > limits(4))) > 0;
if ~out
    x_temp = intersect(polyshape(x1,y1),polyshape(obstacles(1,:),obstacles(2,:)));
    out = x_temp.NumRegions > 0;
end
end
