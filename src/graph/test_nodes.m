function [m,rndq] = test_nodes(arena_limits,obstacle_poly,covered,a)
% Samples points uniformly in the arena until one lies neither in an
% obstacle nor in the a-1 funnels in covered. m counts the samples that fell
% into a funnel (the coverage statistic of apply_term_cond.m).
m = 0;
while true
    rndq = [arena_limits(1) + rand*(arena_limits(2) - arena_limits(1)) ...
           ;arena_limits(3) + rand*(arena_limits(4) - arena_limits(3))];
    if inpolygon(rndq(1), rndq(2), obstacle_poly(1,:), obstacle_poly(2,:))
        continue;
    end
    if ~isempty(covered) && inpolygon(rndq(1), rndq(2), covered(1,1:6*(a-1)), covered(2,1:6*(a-1)))
        m = m+1;
        continue;
    end
    break;
end
end
