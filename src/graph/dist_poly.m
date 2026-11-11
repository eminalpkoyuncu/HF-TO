function [d_min, q_closest] = dist_poly(q, polygon)
% Distance from q (2x1) to a polygon (2xN, not closed), and the nearest
% point on its edges.
N = size(polygon, 2); %column number of the polygon
qN = repmat(q, 1, N); % [p p ... p]
q_to_poly = qN - polygon;   % vectors from corners to q

edges = diff([polygon polygon(:,1)], 1, 2); % [e1 e2 ... eN]

edge_len = sqrt(diag(edges'*edges))';
proj_len = diag(q_to_poly'*edges)' ./ edge_len;

% limit between 0 and edge_len
proj_len(proj_len < 0) = 0;
proj_len(proj_len > edge_len) = edge_len(proj_len > edge_len);

% project onto each line segment
proj_len = proj_len ./ edge_len;
q_cl = polygon + [proj_len; proj_len] .* edges;

n_cl = sqrt(diag((q_cl - qN)'*(q_cl - qN)));

[y, i] = min(n_cl);
d_min = y;
q_closest = q_cl(:, i(1));
end
