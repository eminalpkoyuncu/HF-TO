function [cnstrnt_pos, bnd_pos] = region_halfspaces(box,offset)
% Half-space form C p <= b of a quadrilateral funnel box (2x4 corners), paper
% Eq. (region). The lines through the edges are computed relative to a
% point offset inside the box (for conditioning) and shifted back; an
% axis-aligned box falls back to the bounds in x and y. Rows of C are not
% normalized.

corner1 = (box(:,1) - offset);
corner2 = (box(:,2) - offset);
corner3 = (box(:,3) - offset);
corner4 = (box(:,4) - offset);

coefficients1(1) = (corner2(2)-corner1(2))/(corner2(1)-corner1(1));
coefficients1(2) = corner1(2)-coefficients1(1)*corner1(1);

coefficients2(1) = (corner3(2)-corner2(2))/(corner3(1)-corner2(1));
coefficients2(2) = corner2(2)-coefficients2(1)*corner2(1);

coefficients3(1) = (corner4(2)-corner3(2))/(corner4(1)-corner3(1));
coefficients3(2) = corner3(2)-coefficients3(1)*corner3(1);

coefficients4(1) = (corner1(2)-corner4(2))/(corner1(1)-corner4(1));
coefficients4(2) = corner4(2)-coefficients4(1)*corner4(1);

if sum(isinf(coefficients1)) || sum(isinf(coefficients2)) || sum(isinf(coefficients3)) || sum(isinf(coefficients4))
    upper_bndx = max(box(1,:)-offset(1));
    lower_bndx = min(box(1,:)-offset(1));
    
    upper_bndy = max(box(2,:)-offset(2));
    lower_bndy = min(box(2,:)-offset(2));
    
    %%% x
    if upper_bndx<0
        bnd_pos(1) = -1;
        cnstrnt_pos(1,1) = -1/upper_bndx;
    else
        bnd_pos(1) = 1;
        cnstrnt_pos(1,1) = 1/upper_bndx;
    end
    if lower_bndx>0
        bnd_pos(2) = -1;
        cnstrnt_pos(2,1) = -1/lower_bndx;
    else
        bnd_pos(2) = 1;
        cnstrnt_pos(2,1) = 1/lower_bndx;
    end
    cnstrnt_pos(1:2,2) = zeros(2,1);
    
    %%% y
    if upper_bndy<0
        bnd_pos(3) = -1;
        cnstrnt_pos(3,2) = -1/upper_bndy;
    else
        bnd_pos(3) = 1;
        cnstrnt_pos(3,2) = 1/upper_bndy;
    end
    if lower_bndy>0
        bnd_pos(4) = -1;
        cnstrnt_pos(4,2) = -1/lower_bndy;
    else
        bnd_pos(4) = 1;
        cnstrnt_pos(4,2) = 1/lower_bndy;
    end
    cnstrnt_pos(3:4,1) = zeros(2,1);
    bnd_pos= bnd_pos(:);    
    
else
    cnstrnt_pos = [ coefficients1(1)/(-coefficients1(2)) -1/(-coefficients1(2));...
                    coefficients2(1)/(-coefficients2(2)) -1/(-coefficients2(2));...
                    coefficients3(1)/(-coefficients3(2)) -1/(-coefficients3(2));...
                    coefficients4(1)/(-coefficients4(2)) -1/(-coefficients4(2))];
   

    bnd_pos = [1;1;1;1];
end
bnd_offset = cnstrnt_pos*offset;
bnd_pos = bnd_pos + bnd_offset;
end
