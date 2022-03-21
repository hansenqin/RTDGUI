classdef callbacks < handle
    %This class is used to handle call back functions. The data obtained
    %from the callback functions are stored in the function member
    %variables 
    
    properties
        u;
        v;
        r;
        rot;
        pos;
        arrows_toggle;
       
        
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

    end
    
    methods
        %Constructor
        function obj = callbacks(u,v,r,rot,pos,ax)
            
            % set up u, v, r, plot
            fig2 = figure(2);
            fig2.Position = [0 500 450 550];
          
            obj.u = u;
            obj.v = v;
            obj.r = r;
            obj.rot=rot;
            obj.pos=pos;
            obj.arrows_toggle = 0;
            obj.init = 1;
            obj.init = 1;
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
            
            
            obj.t_list = [obj.t_list, t];
            obj.u_list = [obj.u_list, obj.u];
            obj.v_list = [obj.v_list, obj.v];
            obj.r_list = [obj.r_list, obj.r];
            
        end
        
        
        function tf_cb(obj, src, msg, car,vert,u_arrow, r_arrow)
            FrameID = msg.Transforms.Header.FrameId;
            ChildFrameId = msg.Transforms.ChildFrameId;
            
            if FrameID == "map" && ChildFrameId == "base_link"
                translation = msg.Transforms.Transform.Translation; 
                rotatation = msg.Transforms.Transform.Rotation; 

                obj.pos = [translation.X, translation.Y, 0.085];
                quat = [rotatation.W;
                        rotatation.X;
                        rotatation.Y; 
                        rotatation.Z]'; 

                obj.rot = quat2rotm(quat);
                car.Vertices = (obj.rot'*vert')' + repmat(obj.pos,[size(vert,1),1]);
                current_campos = campos(obj.ax);
                pos_diff = obj.pos(1:2) - current_campos(1:2);
                campos(obj.ax, current_campos+[(pos_diff-[0,15]) 0]);
                camtarget(obj.ax, obj.pos); 
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
            X = msg.Pose.Position.X;
            Y = msg.Pose.Position.Y;

            [vert,faces] = gen_points(X,Y);

            goal_point.Vertices = vert;
            goal_point.Faces = faces;
        end
        
        
        function obs_zonotope_cb(obj, src, msg, obs_zono)
            Polygons = msg.Polygons;
            verts = [];
            num_zono = length(Polygons);
            disp(num_zono)
            Points = [];
            
            T_max = eye(4);
            T_max(1:3, 1:3) = obj.rot;
            T_max(1:3, 4) = obj.pos;
            
            for i = 1:num_zono

                Points = Polygons(i).Polygon.Points;

                for j = 1:length(Points)
                    verts_new = T_max * [Points(j).X; Points(j).Y; 0.02; 1];
                    
                    verts = [verts; verts_new(1:3)'];
                end

            end
            clear('Polygons')
            obs_zono.Vertices = verts;

            % Faces are matrices that specify which vetex connects to which. A new
            % line means a new group of vertex. 
            
            obs_zono.Faces = reshape((1:num_zono*length(Points)),4,[])';

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
                Z = [Z;0; 0.1; 0.1; 0 ];
            end
            
            T_max = eye(4);
            T_max(1:3, 1:3) = obj.rot;
            T_max(1:3, 4) = obj.pos;
            XYZ = T_max * [X';Y';Z';ones(1, length(X))];
            X = XYZ(1,:)';
            Y = XYZ(2,:)';
            Z = XYZ(3,:)';
            
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
        
        function debug_cb(obj, src, msg)
            t1 = msg.Header.Stamp.Sec;
            t1 = cast(t1, 'double');
            t2 = msg.Header.Stamp.Nsec;
            t2 = cast(t2, 'double');
            
            t = t1+t2/1e9;
           
            if obj.init == 1
                obj.td_init = t;
                obj.d_init = 0; 
            end
            
            td = t-obj.td_init;
            ud = msg.Ud;
            vd = msg.Vd;
            rd = msg.Rd;
            
            obj.td_list = [obj.td_list, td];
            obj.ud_list = [obj.ud_list, ud];
            obj.vd_list = [obj.vd_list, vd];
            obj.rd_list = [obj.rd_list, rd];
            

        end
        
        
        function reset_cb(obj, src, msg, car,vert,u_arrow, r_arrow)

            rot1 = [1 0 0;
                    0 -1 0;
                    0 0 -1]; %Initial car rotation

            rot2 = [0 1 0;
                   -1 0 0;
                   0 0 1];   %Initial car rotation

            rot = rot1*rot2;
            pos = [30,18,0.085]; %Initial car position

            vert = car.Vertices;
            car.Vertices = (rot*vert')' + repmat(pos,[size(vert,1),1]);

            % Camera stuffme†me

            campos(pos-[0,15,-7]);
            camtarget(pos);
            camva(7); 
            camlight('headlight');
            
        end
        
        function clear(obj, src, msg, car,vert,u_arrow, r_arrow)

            rot1 = [1 0 0;
                    0 -1 0;
                    0 0 -1]; %Initial car rotation

            rot2 = [0 1 0;
                   -1 0 0;
                   0 0 1];   %Initial car rotation

            rot = rot1*rot2;
            pos = [30,18,0.085]; %Initial car position

            vert = car.Vertices;
            car.Vertices = (rot*vert')' + repmat(pos,[size(vert,1),1]);

            % Camera stuffme†me

            campos(pos-[0,15,-7]);
            camtarget(pos);
            camva(7); 
            camlight('headlight');
            
        end
        
    end
end
 