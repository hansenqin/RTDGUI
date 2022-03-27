


%A series of functions for handling turning subscribers on and off

function setup_control(sub, cb, u_arrow, r_arrow, obs, way_points, initial_point, goal_point,obs_zono)

    fig = uifigure('Position',[0 70 200 210]);
    cb1 = uicheckbox(fig,'Position',[10 16 150 15], 'Text', 'Velocities', ...
          'ValueChangedFcn',@(cbx,event) cb1Changed(cbx, sub, cb, u_arrow, r_arrow),...
          'Value', 0);
    cb2 = uicheckbox(fig,'Position',[10 38 150 15], 'Text', 'Obstacles_LineSeg', ...
          'ValueChangedFcn',@(cbx,event) cb2Changed(cbx, sub, cb, obs),...
          'Value', 0);
    cb3 = uicheckbox(fig,'Position',[10 60 91 15], 'Text', 'Waypoints', ...
          'ValueChangedFcn',@(cbx,event) cb3Changed(cbx, sub, cb),...
          'Value', 0);
    cb4 = uicheckbox(fig,'Position',[10 82 200 15], 'Text', 'Local_Frame_Obstacles', ...
          'ValueChangedFcn',@(cbx,event) cb4Changed(cbx, sub, cb),...
          'Value', 0);
    cb5 = uicheckbox(fig,'Position',[10 104 91 15], 'Text', 'Goal_Point', ...
          'ValueChangedFcn',@(cbx,event) cb5Changed(cbx, sub, cb, goal_point),...
          'Value', 0);
    cb6 = uicheckbox(fig,'Position',[10 126 150 15], 'Text', 'Obstacle_Zonotope', ...
          'ValueChangedFcn',@(cbx,event) cb6Changed(cbx, sub, cb, obs_zono),...
          'Value', 0);
    cb7 = uicheckbox(fig,'Position',[10 148 150 15], 'Text', 'Record', ...
          'ValueChangedFcn',@(cbx,event) cb7Changed(cbx,cb),...
          'Value', 0);
    cb8 = uicheckbox(fig,'Position',[10 170 150 15], 'Text', 'First-Person Camera', ...
          'ValueChangedFcn',@(cbx,event) cb8Changed(cbx,cb),...
          'Value', 0);
    cb9 = uicheckbox(fig,'Position',[10 192 150 15], 'Text', 'Sliced Zonotopes', ...
          'ValueChangedFcn',@(cbx,event) cb9Changed(cbx,sub,cb),...
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

function cb3Changed(cbx, sub, cb)
    if cbx.Value
        try   
            sub.sub_waypoints = rossubscriber('/matlab_plot_info', 'rover_control_msgs/MatlabPlotInfo', @(pub, msg) cb.plot_cb(msg));
            disp('started subscriber for /matlab_plot_info');    
        catch
            disp('/matlab_plot_info')
        end
    else
        sub.sub_waypoints = [];
        disp('stopped subscriber for /matlab_plot_info');    
        
        for i=1:length(cb.p_0)
            cb.p_0(i).XData = [];
            cb.p_0(i).YData = [];
        end
        
        for i=1:length(cb.p_0)
            cb.p_pred(i).XData = [];
            cb.p_pred(i).YData = [];
        end
        
        for i=1:length(cb.p_0)
            cb.p_wp(i).XData = [];
            cb.p_wp(i).YData = [];
        end
        
        for i=1:length(cb.path_list)
            cb.path_list(i).XData = [];
            cb.path_list(i).YData = [];
        end
        cb.p_0 = [];
        cb.p_pred = [];
        cb.p_wp = [];
        cb.path_list = [];
        cb.plot_legend.Visible = 'off';
    end
end

function cb4Changed(cbx, sub, cb)
    if cbx.Value
        try
            sub.sub_obstacles_zono = rossubscriber('/local_obstacles_center',@cb.plot_local_obs_cb, 'DataFormat', 'struct');
            cb.local_mode = 1;
            disp('started subscriber for /local_obstacles_center');
        catch
            disp('/global_obstacles topic not detected')
        end
    else
        disp('stopped subscriber for /local_obstacles_centers');
        
        num = length(cb.local_obs);
        for i=1:num
            cb.local_obs(i).XData = [];
            cb.local_obs(i).YData = [];
        end
        
        cb.local_mode = 0;
        cb.local_obs = [];
        sub.sub_obstacles_local = [];

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


function cb7Changed(cbx, cb)
    if cbx.Value
        try
            cb.recorder =  VideoWriter('curr_recording', 'MPEG-4'); % New
            cb.recorder.FrameRate = 5;
            cb.recorder.Quality = 100;
            open(cb.recorder);
            
            disp('started recording');
            
            while(1)
                frame = getframe(gcf);
                writeVideo(cb.recorder,frame);
            end
        catch
            disp('cannot start recording')
        end
    else
        disp('stopped recording');
        close(cb.recorder);
        cb.recorder = 0;
    end
end


function cb8Changed(cbx, cb)
    if cbx.Value
        disp('Starting first person camera view');
        cb.FP_cam = 1;
    else
        disp('stopped recording');
        cb.FP_cam = 0;
    end
end


function cb9Changed(cbx, sub, cb)
     if cbx.Value
        try   
            sub.sub_zonotopes = rossubscriber('/matlab_plot_info', 'rover_control_msgs/MatlabPlotInfo', @(pub, msg) cb.frs_cb(msg));
            disp('started subscriber for /matlab_plot_info');    
        catch
            disp('/matlab_plot_info')
        end
    else
        sub.sub_zonotopes = [];
        disp('stopped subscriber for /matlab_plot_info');    
        
        cb.frs_list.XData = [];
        cb.frs_list.YData = [];
    end
end


