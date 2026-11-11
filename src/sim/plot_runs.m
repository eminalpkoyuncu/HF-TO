function plot_runs(results,map,nodes,path,xnom,label)
% Successful and failed runs on the funnels, and the clearance histogram.
% xnom (optional, 2xT) is the HF-TO nominal path.
succ = [results.success];
figure('Name',[label ' trajectories']);
titles = ["Successful runs","Failed runs"];
sets = {find(succ),find(~succ)};
for p = 1:2
    subplot(1,2,p); hold on
    draw_arena(map);
    for j = 1:numel(path)
        draw_region(nodes{path(j)},0.3,[0.5 0.5 0.5]);
    end
    for k = sets{p}
        plot(results(k).xy(1,:),results(k).xy(2,:),'Color',[0.85 0.2 0.2 0.25]);
    end
    if ~isempty(xnom)
        plot(xnom(1,:),xnom(2,:),'k','LineWidth',1.5);
    end
    title(sprintf('%s: %s (%d)',label,titles(p),numel(sets{p})),'Interpreter','latex');
    axis equal
end
figure('Name',[label ' clearance']);
histogram([results.minClear],30);
xlabel('Minimum obstacle clearance [m]','Interpreter','latex');
ylabel('Runs','Interpreter','latex');
title(label,'Interpreter','latex');
end
