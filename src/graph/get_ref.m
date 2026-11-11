function [ref, area] = get_ref(current_node, next_node, REF)
% Waypoint (outlet) between two consecutive funnels: the mean of the
% vertices of their overlap ('CENTROID') or the sample point of the next
% funnel ('CLOSEST'). area is the overlap area ('CENTROID' only).
area = [];
if strcmp(REF, 'CLOSEST') == 1
    ref = next_node.q;
elseif strcmp(REF, 'CENTROID') == 1
    box1 = current_node.box;
    [x1, y1] = poly2cw(box1(1,:), box1(2,:));
    box2 = next_node.box;
    [x2, y2] = poly2cw(box2(1,:), box2(2,:));
    [x_temp, y_temp] = polybool('intersection', x1, y1, x2, y2);
    area = polyarea(x_temp,y_temp);
    % polybool closes the polygon: skip the repeated first vertex
    ref = [mean(x_temp(2:end)); mean(y_temp(2:end))];
else
    error('Reference mode must be CLOSEST or CENTROID.');
end
end
