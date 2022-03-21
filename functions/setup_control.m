
%A series of functions for handling turning subscribers on and off

function setup_control(sub, cb, u_arrow, r_arrow, obs, way_points, initial_point, goal_point,obs_zono)

    fig = uifigure('Position',[0 70 150 180]);
    cb1 = uicheckbox(fig,'Position',[10 16 150 15], 'Text', 'Velocities', ...
          'ValueChangedFcn',@(cbx,event) cb1Changed(cbx, sub, cb, u_arrow, r_arrow),...
          'Value', 0);
    cb2 = uicheckbox(fig,'Position',[10 38 150 15], 'Text', 'Obstacles_LineSeg', ...
          'ValueChangedFcn',@(cbx,event) cb2Changed(cbx, sub, cb, obs),...
          'Value', 0);
    cb3 = uicheckbox(fig,'Position',[10 60 91 15], 'Text', 'Waypoints', ...
          'ValueChangedFcn',@(cbx,event) cb3Changed(cbx, sub, cb, way_points),...
          'Value', 0);
    cb4 = uicheckbox(fig,'Position',[10 82 91 15], 'Text', 'Initial_Points', ...
          'ValueChangedFcn',@(cbx,event) cb4Changed(cbx, sub, cb, initial_point),...
          'Value', 0);
    cb5 = uicheckbox(fig,'Position',[10 104 91 15], 'Text', 'Goal_Point', ...
          'ValueChangedFcn',@(cbx,event) cb5Changed(cbx, sub, cb, goal_point),...
          'Value', 0);
    cb6 = uicheckbox(fig,'Position',[10 126 150 15], 'Text', 'Obstacle_Zonotope', ...
          'ValueChangedFcn',@(cbx,event) cb6Changed(cbx, sub, cb, obs_zono),...
          'Value', 0);
    cb7 = uicheckbox(fig,'Position',[10 148 150 15], 'Text', 'Velocities_Desired', ...
          'ValueChangedFcn',@(cbx,event) cb7Changed(cbx, sub, cb),...
          'Value', 0);
    cb8 = uibutton(fig,'Position',[10 170 150 15], 'Text', 'Clear', ...
          'ValueChangedFcn',@(cbx,event) cb8Changed(cbx, sub, cb),...
          'Value', 0);
end

function cb1Changed(cbx, sub, cb, u_arrrow_patch, r_arrow_patch)
    val = cbx.Value;
    if val
        try
           sub.sub_velocities = rossubscriber('/vesc/odom', @cb.odom_cb, 'DataFormat', 'struct');
           sub.sub_velocities.NewMessageFcn = {@cb.odom_cb};
           cb.arrows_toggle = 1;
           disp('started subscriber for /vesc/odom');
        catch
           disp('/vesc/odom topic not detected')
        end
    else
        disp('stopped subscriber for /vesc/odom');
        cb.arrows_toggle = 0;
        sub.sub_velocities = [];
        u_arrrow_patch.Vertices = [0 0];
        u_arrrow_patch.Faces = 1;
        r_arrrow_patch.Vertices = [0 0];
        r_arrrow_patch.Faces = 1;

    end
end



function cb2Changed(cbx, sub, cb, obs)
    val = cbx.Value;
    if val
        try
            sub.sub_obstacles_seg = rossubscriber('/obstacles', @cb.obstacle_seg_cb, 'DataFormat', 'struct');
            sub.sub_obstacles_seg.NewMessageFcn = {@cb.obstacle_seg_cb, obs};
            disp('started subscriber for /obstacles');
        catch
            disp('/obstacles topic not detected')
        end
    else
        disp('stopped subscriber for /obstacles');
        sub.sub_obstacles_seg = [];
        obs.Vertices = [0 0];
        obs.Faces = 1;

    end
end

function cb3Changed(cbx, sub, cb, way_points)
    val = cbx.Value;
    if cbx.Value
        try
            sub.sub_waypoints = rossubscriber('/sPath', @cb.path_cb, 'DataFormat', 'struct');
            sub.sub_waypoints.NewMessageFcn = {@cb.path_cb, way_points};
            disp('started subscriber for /sPath');    
        catch
            disp('/sPath topic not detected')
        end
    else
        disp('stopped subscriber for /sPath');    
        sub.sub_waypoints = [];
        way_points.Vertices = [0 0];
        way_points.Faces = 1;

    end
end

function cb4Changed(cbx, sub, cb, initial_point)
    val = cbx.Value;
    if cbx.Value
        try
            sub.sub_initial_point = rossubscriber('/initialpose', @cb.initialPoint_cb, 'DataFormat', 'struct');
            sub.sub_initial_point.NewMessageFcn = {@cb.initialPoint_cb, initial_point};
            disp('started subscriber for /initialpose');
        catch
            disp('/initialpose topic not detected')
        end
        
    else
        disp('stopped subscriber for /initialpose');
        sub.sub_initial_point = [];
        initial_point.Vertices = [0 0];
        initial_point.Faces = 1;

    end
end

function cb5Changed(cbx, sub, cb, goal_point)
    val = cbx.Value;
    if cbx.Value
        try
            sub.sub_goal_point = rossubscriber('/move_base_simple/goal', @cb.goal_cb, 'DataFormat', 'struct');
            sub.sub_goal_point.NewMessageFcn = {@cb.goal_cb, goal_point}; 
            disp('started subscriber for /move_base_simple/goal');
        catch
            disp('/move_base_simple/goal topic not detected')
        end
        
    else
        disp('stopped subscriber for /move_base_simple/goal');
        sub.sub_goal_point = [];
        goal_point.Vertices = [0 0];
        goal_point.Faces = 1;

    end
end

function cb6Changed(cbx, sub, cb, obs_zono)
    val = cbx.Value;
    if cbx.Value
        try
            sub.sub_obstacles_zono = rossubscriber('/zonotope_visualization', @cb.obs_zonotope_cb, 'DataFormat', 'struct');
            sub.sub_obstacles_zono.NewMessageFcn = {@cb.obs_zonotope_cb, obs_zono}; 
            disp('started subscriber for /zonotope_visualization');
        catch
            disp('/zonotope_visualization topic not detected')
        end
    else
        disp('stopped subscriber for /zonotope_visualization');
        sub.sub_obstacles_zono = [];
        obs_zono.Vertices = [0 0];
        obs_zono.Faces = 1;

    end
end

function cb7Changed(cbx, sub, cb)
    val = cbx.Value;
    if cbx.Value
        try
            sub.sub_velocities_desired = rossubscriber('/state_out/rover_debug_state_out', @cb.debug_cb, 'DataFormat', 'struct');
            disp('started subscriber for /state_out/rover_debug_state_out');
        catch
            disp('/state_out/rover_debug_state_out topic not detected')
        end
    else
        disp('stopped subscriber for /state_out/rover_debug_state_out');
        sub.sub_velocities_desired = [];
       
    end
end

function cb8Changed(cbx, sub, cb)
    val = cbx.Value;
    if cbx.Value
        try
            sub.sub_initial_point = rossubscriber('/initialpose', @cb.initialPoint_cb, 'DataFormat', 'struct');
            sub.sub_initial_point.NewMessageFcn = {@cb.initialPoint_cb, initial_point};
            disp('started subscriber for /initialpose');
        catch
            disp('/initialpose topic not detected')
        end
        
    else
        disp('stopped subscriber for /initialpose');
        sub.sub_initial_point = [];
        initial_point.Vertices = [0 0];
        initial_point.Faces = 1;

    end
end
