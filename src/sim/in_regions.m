function inside = in_regions(P,regions)
% True for each column of P (2xM) that lies in the union of the regions.
inside = false(1,size(P,2));
for j = 1:numel(regions)
    inside = inside | inpolygon(P(1,:),P(2,:),regions{j}(1,:),regions{j}(2,:));
end
end
