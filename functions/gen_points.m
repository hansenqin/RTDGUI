function [vert,faces] = gen_points(x,y)
    % Given an x,y  position, this function generates a vertices and face matrix
    % that's used for plotting a circle for a patch object
    

    faces = [];
    vert = [];
    points_per_circle = (0:2*pi/50:2*pi)';
    num_points_pre_circle = length(0:2*pi/50:2*pi);
    for i=1:length(x)
        vert = [vert;0.2*cos(points_per_circle)+x(i),0.2*sin(points_per_circle)+y(i),repmat(0.01,num_points_pre_circle,1)];
        faces = [faces;
                (1:1:num_points_pre_circle)+(i-1)*num_points_pre_circle];
    end
    
end