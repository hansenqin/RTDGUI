% Takes zonotope and rotates by h, translates along xy. 
% Mirroring is missing some dimensions, mostly just used for quick plotting.
function zono_out = transform_zono(zono, h, xy, mirror)
    Z = zono.Z;
    if mirror
%         Z([2 7 8 9],:) = -Z([2 7 8 9],:);
        Z([2 11 12],:) = -Z([2 11 12],:);
    end
    Z(1:2,:) = rotmat(h) * Z(1:2,:);
    Z(1:2,1) = Z(1:2,1) + xy;
    zono_out = zonotope(Z);
end