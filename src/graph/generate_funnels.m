function nodes = generate_funnels(map, graph_params) %#ok<INUSD>
% Funnel generation of MPC-Graph (Atasoy et al., 2023), paper Fig. (sng):
% points are sampled uniformly in the free workspace; at each point a square
% touching the nearest obstacle (tangential orientation) is grown into the
% largest rectangle that still avoids the obstacles, first along the
% obstacle and then away from it. Sampling stops when the coverage test of
% apply_term_cond.m is met. nodes{i}.box holds the four corners.

% all obstacles as one closed polygon, separated by NaNs
obstacle_poly = [map.obstacle{1} map.obstacle{1}(:,1)];
for i = 2:length(map.obstacle)
    obstacle_poly = [obstacle_poly [NaN; NaN] map.obstacle{i} map.obstacle{i}(:,1)]; %#ok<AGROW>
end

covered = [];
nodes = {};
a = 1;
while true
    % m: samples that fell into existing funnels before a new free one
    [m,rndq] = test_nodes(map.limits,obstacle_poly,covered,a);
    if apply_term_cond(m)
        break;
    end
    [~, q_obs] = min_dist(rndq, map.limits, map.obstacle);
    node = generate_node(rndq, q_obs);
    node = enlarge(node, obstacle_poly, map.limits);
    nodes{a} = node; %#ok<AGROW>
    covered(:, 6*a-5:6*a) = [[NaN; NaN] nodes{a}.box nodes{a}.box(:,1)];
    a = a+1;
end
end
