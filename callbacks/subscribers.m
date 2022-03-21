classdef subscribers < handle
    % This is a class used to store ROS subscriber objects. By setting any
    % of the subscribers to [], we effectively turn the subscriber off and
    % stop recieving data from it
    
    
    properties
        sub_velocities;
        sub_obstacles_seg;
        sub_waypoints;
        sub_initial_point;
        sub_goal_point;
        sub_obstacles_zono;
        sub_tf;
        sub_mocap;
        sub_velocities_desired; 
    end
end
