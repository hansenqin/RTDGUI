function [verts, faces] = get_cross_points(X, Y)
    scale = 0.05;
    X_pt = [0 2 3 1 3 2 0 -2 -3 -1 -3 -2]*scale;
    Y_pt = [1 3 2 0 -2 -3 -1 -3 -2 0 2 3]*scale;
    
    verts = [X_pt'+X, Y_pt'+Y, repmat(0.01, length(X_pt), 1)];
    faces = 1:1:length(X_pt);

end

