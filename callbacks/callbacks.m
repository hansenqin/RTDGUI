classdef callbacks < handle
    %This class is used to handle call back functions. The data obtained
    %from the callback functions are stored in the function member
    %variables 
    
    properties
        x;
        y;
        h;
        u;
        v;
        r;
        rot;
        pos;
        arrows_toggle;
        
        %For delaying tf message to sync up with 
        msg_queue = [];
        
        t_list;
        u_list;
        v_list;
        r_list;
        td_list;
        ud_list;
        vd_list;
        rd_list;
       
        ax;
        ax2;
        init;
        d_init;
        t_init;
        td_init;
        
        %MATLAB visulization message stuff
        p_0;
        p_pred;
        p_wp;
        path_list;
        plot_legend;
        curr_pos_and_predicted_pos_toggle;
        box_region_X;
        box_region_Y;
        
        %Video recording
        recorder;
        
        %Local frame mode
        local_mode;
        local_obs;
        
        %First-Person Camera mode
        FP_cam;
        
        %FRS
        frs_file;
        frs_low_file;
        frs_list;
        queue;
        
        %Traveled path
        traveled_path;
        traveled_path_toggle;
        auto_flag;
        
        %debug
        debug_counter;
        
        %marking boxes
        obs_zono_box;
        mark_box_toggle;
      
        
    end
    
    methods
        %Constructor
        function obj = callbacks(u,v,r,rot,pos,ax)
            
            % set up u, v, r, plot          
            obj.u = u;
            obj.v = v;
            obj.r = r;
            obj.rot=rot;
            obj.pos=pos;
            obj.arrows_toggle = 0;
            obj.init = 1;
            obj.init = 1;
            obj.FP_cam = 0;
            obj.traveled_path_toggle = 0;
            obj.debug_counter = 0;
            
            obj.ax = ax;
            obj.auto_flag = 1;
            obj.local_mode = 0;
            obj.frs_file = load('FRS_Rover_19-Dec-2021_no_force.mat');
            obj.frs_low_file = load('FRS_Rover_04-Jan-2022_low_spd.mat');
            queue = {};
            
            obj.mark_box_toggle = 0;
            obj.box_region_X = [0,0];
            obj.box_region_Y = [0,0];
            

            
        end
        
        
        function odom_cb(obj, src, msg) 
            
            t1 = msg.Header.Stamp.Sec;
            t1 = cast(t1, 'double');
            t2 = msg.Header.Stamp.Nsec;
            t2 = cast(t2, 'double');
            
            t = t1+t2/1e9;
           
            if obj.init == 1
                obj.t_init = t;
                obj.init = 0; 
            end
            obj.u = msg.Twist.Twist.Linear.X;
            obj.v = msg.Twist.Twist.Linear.Y;
            obj.r = msg.Twist.Twist.Angular.Z;
            t = t - obj.t_init;
            disp(["u is: ", obj.u]);
            
            obj.t_list = [obj.t_list, t];
            obj.u_list = [obj.u_list, obj.u];
            obj.v_list = [obj.v_list, obj.v];
            obj.r_list = [obj.r_list, obj.r];
            
        end
        
        
        function tf_cb(obj, src, msg, car,vert,u_arrow, r_arrow, traversed_path)
            
            FrameID = msg.Transforms.Header.FrameId;
            ChildFrameId = msg.Transforms.ChildFrameId;

            if FrameID == "map" && ChildFrameId == "base_link"
                translation = msg.Transforms.Transform.Translation; 
                rotatation = msg.Transforms.Transform.Rotation; 
                obj.x = translation.X;
                obj.y = translation.Y;


                if obj.local_mode == 1
                    obj.pos = [0, 0, 0.085];
                    obj.rot(1:2, 1:2) = [0, -1;1 0];
                    obj.h = rotm2eul(obj.rot);
                    obj.h = obj.h(1);
                else
                    obj.pos = [translation.X, translation.Y, 0.085];
                    quat = [rotatation.W;
                        rotatation.X;
                        rotatation.Y; 
                        rotatation.Z]';
                    obj.rot = quat2rotm(quat); 
                    obj.h = rotm2eul(obj.rot);
                    obj.h = obj.h(1)-pi/2;
                end

                car.Vertices = (obj.rot'*vert')' + repmat(obj.pos,[size(vert,1),1]);
                current_campos = campos(obj.ax);

                % camera angle
                if obj.FP_cam == 1
                    FP_cam_pos = obj.pos(1:2)' - obj.rot(1:2, 1:2)*[0;20];
                    campos(obj.ax, [FP_cam_pos', 10]);
                else
                    pos_diff = obj.pos(1:2) - current_campos(1:2);
                    campos(obj.ax, current_campos+[(pos_diff-[0,15]) 0]);
                end

                % Traveled path
                if obj.traveled_path_toggle && obj.auto_flag
                    traversed_path.XData(end+1) = translation.X;
                    traversed_path.YData(end+1) = translation.Y;
                end

                camtarget(obj.ax, obj.pos); 
                
                %box drawing 
                if obj.mark_box_toggle  

                    obj.ax2.XLim = [obj.x-10, obj.x+10];
                    obj.ax2.YLim = [obj.y-10, obj.y+10];  
                end


                if obj.arrows_toggle == 1
                    draw_arrows(obj.u, obj.v, obj.r, obj.pos, obj.rot, u_arrow, r_arrow)
                end

            end 
            
            
 
        end
        
        
        function path_cb(obj, src, msg, way_points) 

            X = [];
            Y = [];

            num_data_points = length(msg.Poses);
   
            for i=1:num_data_points
                X(end+1) = msg.Poses(i).Pose.Position.X;
                Y(end+1) = msg.Poses(i).Pose.Position.Y;
            end

            [vert,faces] = gen_points(X,Y);

            way_points.Vertices = vert;
            way_points.Faces = faces;
   
        end
        
        
        function goal_cb(obj, src, msg, goal_point)
%             X = msg.Pose.Position.X;
%             Y = msg.Pose.Position.Y;

            X = msg.Point.X;
            Y = msg.Point.Y;
            
            [vert,faces] = gen_rect_points(X,Y);

            goal_point.Vertices = vert;
            goal_point.Faces = faces;
        end
        
        
        function obs_zonotope_cb(obj, src, msg, obs_zono, obs_zono_box)
            Polygons = msg.Polygons;
            verts = [];
            obs_verts = [];
            num_zono = length(Polygons);
            Points = [];
             
            T_max = eye(4);
%             T_max(1:3, 1:3) = obj.rot
%             T_max(1:3, 4) = obj.pos;
%             
            for i = 1:num_zono
                
                Points = Polygons(i).Polygon.Points;
                add_to_box_obs = false;

                for j = 1:length(Points)
                    verts_new = T_max * [Points(j).X; Points(j).Y; 0.0; 1];
                    verts_new(3) = 0.05;
                    verts = [verts; verts_new(1:3)']; 

                    
                    if any(verts_new(1)>obj.box_region_X(:,1) & verts_new(1)<obj.box_region_X(:,2) &...
                       verts_new(2)>obj.box_region_Y(:,1) & verts_new(2)<obj.box_region_Y(:,2))
                        add_to_box_obs = 1;
                    end
                end
                
                if add_to_box_obs
                    for j = 1:length(Points)
                        verts_new = T_max * [Points(j).X; Points(j).Y; 0.0; 1];
                        verts_new(3) = 0.05;
                        obs_verts = [obs_verts; verts_new(1:3)'];
                        

                    end
                end

            end
            clear('Polygons')
            
            % Faces are matrices that specify which vetex connects to which. A new
            % line means a new group of vertex. 
            obs_zono.Vertices = verts; 
            obs_zono.Faces = reshape((1:num_zono*length(Points)),4,[])';
            
            if obj.mark_box_toggle
                obj.obs_zono_box.Vertices = verts; 
                obj.obs_zono_box.Faces = reshape((1:num_zono*length(Points)),4,[])';
            end
            
            
            if size(obs_verts,1) >0
                obs_zono_box.Vertices = obs_verts; 
                obs_zono_box.Faces = reshape(1:size(obs_verts,1),4,[])';
            end

        end
        
        
        function obstacle_seg_cb(obj,src,msg, obs)
            seg = msg.Segments;
            X = [];
            Y = [];
            Z = [];

            for i=1:length(seg)

                x_first = seg(i).FirstPoint.X;
                y_first = seg(i).FirstPoint.Y;

                x_last = seg(i).LastPoint.X;
                y_last = seg(i).LastPoint.Y;

                X = [X;x_first; x_first; x_last; x_last];
                Y = [Y;y_first; y_first; y_last; y_last];
                Z = [Z;0; 0.4; 0.4; 0 ];
            end
            
            T_max = eye(4);
            T_max(1:3, 1:3) = obj.rot;
            T_max(1:3, 4) = obj.pos;
            XYZ = T_max * [X';Y';Z';ones(1, length(X))];
            X = XYZ(1,:)';
            Y = XYZ(2,:)';
%             Z = XYZ(3,:)';
            
            obs.Vertices = [X, Y, Z];
            
            
            num_points = length(X);

            obs.Faces = reshape((1:num_points),4,[])';
        end
        
        
        function mocap_cb(obj, src, msg, car,vert,u_arrow, r_arrow)

            obj.pos = [msg.Pose.Position.X/1000, msg.Pose.Position.Z/1000, 0.085];
            quat = [msg.Pose.Orientation.X;
                    msg.Pose.Orientation.Y;
                    msg.Pose.Orientation.Z; 
                    msg.Pose.Orientation.W]'; 
                
            %rotation matrix returned by mocap is about the Y axis, so we
            %need to do a change of coordinate (similarity transform) for
            %the rotation matrix to be about the Z axis 
            
            rot1 = [1  0  0;
                    0  0 -1;
                    0  1  0]; %Initial car rotation


            obj.rot = rot1'*quat2rotm(quat)*rot1; 
            car.Vertices = (obj.rot'*vert')' + repmat(obj.pos,[size(vert,1),1]);
            current_campos = campos(obj.ax);
            pos_diff = obj.pos(1:2) - current_campos(1:2);
            campos(obj.ax, current_campos+[(pos_diff-[0,15]) 0]);
            camtarget(obj.ax, obj.pos); 
            if obj.arrows_toggle == 1
                draw_arrows(obj.u, obj.v, obj.r, obj.pos, obj.rot, u_arrow, r_arrow)
            end
            
        end
 
        function plot_local_obs_cb(obj,src, msg)
            num = length(obj.local_obs);
        
            for i=1:num
                obj.local_obs(i).XData = [];
                obj.local_obs(i).YData = [];
            end
            
            
            i = 1;
            while i <= length(msg.Data)-1
                obj.local_obs = [obj.local_obs, circle([msg.Data(i), msg.Data(i+1)], 0.2, 1000,'--')];
            i = i+2;
            end
            
            disp(["Predicted h is: ", msg.Data(i-1)]);
            
        end
        
        function plot_cb(obj, msg, way_points)

            state_0 = [msg.X0; msg.Y0; msg.H0; msg.U0; msg.V0; msg.R0];
            state_pred = [msg.PredX; msg.PredY; msg.PredH; ...
                          msg.PredU; msg.PredV; msg.PredR];
            waypoint_state = [msg.WpX; msg.WpY; msg.WpH];
%             path_msg_tmp = msg.FullRobPath;

            
            if obj.curr_pos_and_predicted_pos_toggle
                obj.p_0.XData = [];
                obj.p_0.YData = [];

                obj.p_pred.XData = [];
                obj.p_pred.YData = [];

                obj.p_0 = plot_state(state_0, 'r', 'x_0');
                obj.p_pred = plot_state(state_pred, 'b', 'Predicted');
            end
            
%             obj.p_wp.XData = [];
%             obj.p_wp.YData = [];
%             obj.p_wp = plot_state(waypoint_state, 'r', 'Waypoint');
%             
%             X = msg.Pose.Position.X;
%             Y = msg.Pose.Position.Y;

            if(obj.debug_counter > 7)
                %vid 1
                msg.WpX = msg.WpX + 0.5;
            end

            [vert,faces] = get_cross_points(msg.WpX,msg.WpY);

            way_points.Vertices = vert;
            way_points.Faces = faces;

            
%             %curr executing path
%             rob_path = path_msg_tmp; % msg.FullRobPath;
% 
%             num_path_pts = length(rob_path.Poses);
%             path_pts = zeros(2, num_path_pts);
%             
%             for i=1:length(obj.path_list)
%                 obj.path_list(i).XData = [];
%                 obj.path_list(i).YData = [];
%             end
%             
%             for i = 1:num_path_pts
%                 pos_i = rob_path.Poses(i).Pose.Position;
%                 path_pts(:, i) = [pos_i.X; pos_i.Y];
%             end
%             obj.path_list = plot(path_pts(1,:), path_pts(2,:), 'k--', 'LineWidth', 2, 'DisplayName', 'Path');
%             
            
            %legends
%             if obj.curr_pos_and_predicted_pos_toggle
%                 obj.plot_legend = legend([obj.p_0(1), obj.p_pred(1), obj.p_wp(1)], 'x_0', 'Predicted', 'Waypoint');
%             else
%                 obj.plot_legend = legend(obj.p_wp(1), 'Waypoint');
%             end
            set(gca,'fontsize', 25)
        end
        
        
        function frs_cb(obj, msg)
            state_pred = [msg.PredX; msg.PredY; msg.PredH; ...
                          msg.PredU; msg.PredV; msg.PredR];
            frs_indices = [msg.U0Idx, msg.Idx0, msg.Idx1]
            obj.debug_counter = obj.debug_counter+1;
            obj.debug_counter
            k_param = msg.KParam
            
%             uvrk = [state_pred(4:6); k_param];
%             uvrk = [obj.u;obj.v;obj.r;k_param];
%             if obj.debug_counter > 12
%                  pause(1)
%             end
%             
            if obj.debug_counter <3
                 frs_indices(1) = 6;
            end
            
            state_pred = [obj.x; obj.y; obj.h; obj.u; obj.v; obj.r];
            if msg.IsLow
                frs_to_use = obj.frs_low_file;
            else
                frs_to_use = obj.frs_file;
            end
            if(obj.debug_counter == 8)
                k_param = msg.KParam-10.5;
                frs_indices(1) = 2;
            end
            
            if(obj.debug_counter == 9)
                k_param = -1;
                frs_indices(1) = 1;
            end

               
            obj.queue = [obj.queue; {frs_to_use, msg.ManuType, frs_indices, k_param}];
            if length(obj.queue(:,1)) > 1
                delete(obj.frs_list);
                curr_frs_info = obj.queue(1,:);
                uvrk = [obj.u;obj.v;obj.r;curr_frs_info{4}];
                obj.frs_list = plot_sliced_frs(curr_frs_info{1}, curr_frs_info{2}, curr_frs_info{3}, state_pred, uvrk);
                obj.queue(1,:) = []; 
            end
%              obj.frs_list = plot_sliced_frs(frs_to_use, msg.ManuType, frs_indices, state_pred, uvrk);
            
        end
        
        function select_boxes(obj)
            figure(2)
            [x_,y_] = ginput(2);
            x_ = sort(x_);
            y_ = sort(y_);
            
            obj.box_region_X = [obj.box_region_X; x_'];
            obj.box_region_Y = [obj.box_region_Y; y_'];
        end
        
        function auto_flag_cb(obj, msg)
            obj.auto_flag = msg.RunningAuto;
            
        end

    end
end
 