function [nodes,path] = dijkstra(nodes,q_start,q_goal,dijkstra_cost)
% Lowest-cost funnel sequence from the funnel containing q_start to the one
% containing q_goal (the largest one if several do). The goal funnel becomes
% node 1 (the root of the search tree) and the start funnel node 2; every
% node gets a back-pointer .path towards the goal and its .depth. path lists
% the funnel indices from the start to the goal.

n = length(nodes);

starting_node = [];
goal_node = [];
for i = 1:length(nodes)
    % find nodes that contain starting node and choose the one that has
    % largest area
    if inpolygon(q_start(1),q_start(2),nodes{i}.box(1,:),nodes{i}.box(2,:))
        if isempty(starting_node)
            starting_node = i;
        elseif polyarea(nodes{starting_node}.box(1,:),nodes{starting_node}.box(2,:)) < polyarea(nodes{i}.box(1,:),nodes{i}.box(2,:))
            starting_node = i;
        end
    end
    % find nodes that contain goal node and choose the one that has
    % largest area
    if inpolygon(q_goal(1),q_goal(2),nodes{i}.box(1,:),nodes{i}.box(2,:))
        if isempty(goal_node)
            goal_node = i;
        elseif polyarea(nodes{goal_node}.box(1,:),nodes{goal_node}.box(2,:)) < polyarea(nodes{i}.box(1,:),nodes{i}.box(2,:))
            goal_node = i;
        end
    end
end
% rearrange nodes for dijkstra
dummy = nodes{1};
nodes{1} = nodes{goal_node};
nodes{goal_node} = dummy;
nodes{1}.depth = 0;
nodes{1}.path = 1;

dummy = nodes{2};
nodes{2} = nodes{starting_node};
nodes{starting_node} = dummy;

for i=n:-1:1
    box = nodes{i}.box;
    polygons(i) = polyshape(box(1,:), box(2,:));
    centers(i, :) = nodes{i}.center;
end

connectivity = overlaps(polygons) - eye(n);
distances = pdist2(centers, centers);

if strcmp(dijkstra_cost,'C2C')==1
    G = distances.*connectivity + 1e5*(1-connectivity);
elseif strcmp(dijkstra_cost,'C2I2C')==1
    G = kinematic_cost(nodes,connectivity,0);
elseif strcmp(dijkstra_cost,'C2I2CWA')==1
    G = kinematic_cost(nodes,connectivity,1);
else
    error('Dijkstra cost must be defined')
end
% in-place Dijkstra algorithm
% convert the graph to a tree
% !!! assumes node-1 is the root

Q = ones(1, n);
distance = 1e10*ones(1, n);
distance(1) = 0;

while sum(Q) > 0
    [m, u] = min(distance);
    Q(u) = 0;
    
    for i=1:n
        if Q(i) > 0 && G(u,i) + m < distance(i)
            distance(i) = G(u,i) + m;
            nodes{i}.path = u;
            nodes{i}.depth = nodes{u}.depth + 1;
        end
    end
    
    distance(u) = 1e10;
end

% generate the path using back-pointers
path_len = nodes{2}.depth+1;

path = zeros(path_len, 1);
t = 2;
for i = 1:path_len
    path(i) = t;
    t = nodes{t}.path;
end
end
