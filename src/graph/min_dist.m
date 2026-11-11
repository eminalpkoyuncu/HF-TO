function [d_min, q_closest] = min_dist(q, limits, map)
% Distance from q to the nearest obstacle or arena boundary (map: cell of
% 2xN polygons), and the nearest point.
l = length(map);
d_o = zeros(1, l+1);
q_o = zeros(2, l+1);

xmin = limits(1);
xmax = limits(2);
ymin = limits(3);
ymax = limits(4);

% boundary polygon
boundary = [xmin xmin xmax xmax; ymin ymax ymax ymin];
[d_o(:, l+1), q_o(:, l+1)] = dist_poly(q, boundary);

% iterate through obstacles
for o = 1:length(map)
    [d_o(:, o), q_o(:, o)] = dist_poly(q, map{o});
end

[y, i] = min(d_o);
d_min = y;
q_closest = q_o(:, i(1));
end
