function cost = kinematic_cost(nodes,connectivity,add_area)
% Edge cost of the funnel graph: distance from the center of funnel i to the
% waypoint in the overlap and on to the center of funnel j, plus
% 1/(overlap area) if add_area (favours wide transitions); 1e5 if the
% funnels do not overlap.
for i = 1:length(connectivity)
    for j = 1:length(connectivity)
        if connectivity(i,j) == 1
            [ref,area] = get_ref(nodes{i},nodes{j},'CENTROID');
            cost(i,j) = norm(nodes{i}.center-ref)+norm(nodes{j}.center-ref)+1/area*add_area; %#ok<AGROW>
        else
            cost(i,j) = 1e5; %#ok<AGROW>
        end
    end
end
end
