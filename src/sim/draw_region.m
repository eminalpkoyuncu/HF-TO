function h = draw_region(node,transparency,color)
% Draws a funnel box, filled grey, behind the other graphics.
h = fill(node.box(1,:),node.box(2,:),[0.5 0.5 0.5],'FaceAlpha',transparency,'EdgeColor',color);
hold on
uistack(h,'bottom');
end
