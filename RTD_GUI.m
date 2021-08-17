%RTD GUI
%Author: Hansen Qin
%Email: qinh@umich.edu
%Created: 8/16/2021
%Modified: 8/16/2021
%
%Purpose: Visualize real time ROS data for the rover. Including obstacle
%         zonotopes, goal point, initial point, waypoint, obstacle line segments,
%         and velocities.

clear global
close all force
clear


%% Connect to ros master 
try
    rosinit('http://35.3.93.125:11311/')
catch
    disp('Unable to connect to ROS Master')
end

%% Scene setup
fig = figure;
set(fig,'KeyPressFcn',@keyboard_cb);
ax = gca;
set(ax, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold(ax,'on');
fig.Position = [800 720 1300 720];
ax.Visible = 'off';
ax.Clipping = 'off';
ax.Projection = 'perspective';
cam_height = 60;
axis([-100 100 -100 100 -100 100])
    
%% Add the stl and prepare patches
car_stl = stlread('car_scaled.stl');
cone = stlread('cone.stl');
car = patch(ax, car_stl,'FaceColor',     [0/255,102/255,1], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
material('metal')



% Initialize patch object. Patch object are passed by refrence 
obs = patch(0,0,0, 'm');
u_arrow = patch(0,0,0,'c');
v_arrow = patch(0,0,0, 'g');
r_arrow = patch(0,0,0, [0/255,102/255,0]);
way_points = patch(0,0,0,[0.7 0 0]);
initial_point = patch(0,0,0,[0.7,0.7,0.7]);
goal_point = patch(0,0,0,[0.2,0.2,0.2]);
obs_zono = patch(0,0,0);
traversed_path = plot(0,0,'r');
    
   

%% Add the ground + textures 
ground = 1000*ones(100,100)-1000;
groundSurf = surf(linspace(-70,70,size(ground,1)),linspace(-70,70,size(ground,2)),ground);
groundSurf.FaceColor = [1,1,1];
% texture = imread('Mcity.png');
%     texture = imread('Reduced_Resolution_Map.bmp');
%     textureSize = size(texture);
%     ground2 = zeros(72,79);
%     flatGround = surf(linspace(-39,40,79),linspace(-36,36,72),ground2);
%     set(flatGround, 'linestyle', 'none');
%     flatGround.FaceColor = 'texturemap';
%     flatGround.CData = texture;

%% Initial placement of car 

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
    
%% Camera stuff

campos(pos-[0,15,-7]);
camtarget(pos);
camva(7); 
camlight('headlight');

    
%% ROS subscriber functions setup

sub = subscribers;
cb = callbacks(0,0,0,rot,pos);

setup_control(sub, cb, u_arrow, r_arrow, obs, way_points, initial_point, goal_point,obs_zono);

% TF topic will be set up by default, all other topics must be turned on
% manuallyu
sub.sub_tf = rossubscriber('/tf', @cb.tf_cb, 'DataFormat', 'struct');
sub.sub_tf.NewMessageFcn = {@cb.tf_cb, car,vert, u_arrow, r_arrow};
disp('started subscriber for /tf');



