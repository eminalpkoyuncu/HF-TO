function [map,nodes,path,graph_params,regions] = build_funnels(cfg)
% Map, funnel graph and lowest-cost funnel sequence. Deterministic for a
% given cfg.seed, so every method and experiment sees the same funnels.
graph_params = funnel_graph_params;
rng(cfg.seed);
map = init_arena(cfg.mapNo);
nodes = generate_funnels(map,graph_params);
[nodes,path] = dijkstra(nodes,map.start,map.goal,graph_params.DIJKSTRA_COST);
regions = cellfun(@(j) nodes{j}.box,num2cell(path(:).'),'UniformOutput',false);
end
