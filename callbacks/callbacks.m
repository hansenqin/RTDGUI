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

    end
    
    methods
        function obj = callbacks(u,v,r,rot,pos)
            obj.u = u;
            obj.v = v;
            obj.r = r;
            obj.rot=rot;
            obj.pos=pos;
            arrows_toggle = 0;
        end
        
        
        function odom_cb(obj, src, msg) 
            obj.u = msg.Twist.Twist.Linear.X;
            obj.v = msg.Twist.Twist.Linear.Y;
            obj.r = msg.Twist.Twist.Angular.Z;
    
     
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
                current_campos = campos;
                pos_diff = obj.pos(1:2) - current_campos(1:2);
                campos(current_campos+[(pos_diff-[0,15]) 0]);
                camtarget(obj.pos); 
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

            for i = 1:num_zono

                Points = Polygons(i).Polygon.Points;

                for j = 1:length(Points)
                    verts = [verts;[Points(j).X Points(j).Y, 0.02]];
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

            obs.Vertices = [X, Y, Z];
            num_points = length(X);

            obs.Faces = reshape((1:num_points),4,[])';
        end
    end
end
 