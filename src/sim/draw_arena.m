function draw_arena(map)
% Draws the arena boundary and the obstacles.
xmin = map.limits(1);
xmax = map.limits(2);
ymin = map.limits(3);
ymax = map.limits(4);
line([xmin xmin xmax xmax xmin], [ymin ymax ymax ymin ymin],'Color','k');
for i = 1:length(map.obstacle)
    obstacle = map.obstacle{i};
    patch(obstacle(1, :), obstacle(2, :),'black');
end
axis tight;
axis equal;
end
