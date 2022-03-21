function draw_arrows(u, v, r, pos, rot, u_arrow_patch, r_arrow_patch)
    scaling_factor = 0.02;
    c_u = 10;
    c_r = 10;

    init_rot = [0 -1 0; 1 0 0;0 0 1];

    u_arrow = [0, c_u*u, c_u*u, (c_u*abs(u)+5)*sign(u), c_u*u, c_u*u,0;
               -1,  -1, -2, 0, 2, 1, 1;
                2,2,2, 2, 2,2,2;]*scaling_factor;

    u_arrow = rot*init_rot*u_arrow+[pos(1);pos(2);0];
    u_arrow(3,:) = 0.002;

    u_arrow_patch.Vertices = u_arrow';
    u_arrow_patch.Faces = [1,2,3,4,5,6,7];

    r = r/c_r;

    r_arrow_head = [ -15, -13, -16, -19,-17;
                       0,  0, 4*sign(-r), 0, 0;
                      1, 1,1, 1, 1]*scaling_factor;

    r_arrow_rot = [cos(2*r) -sin(2*r) 0;
                   sin(2*r)  cos(2*r) 0;
                        0               0 1]';

    r_arrow_head = r_arrow_rot'*r_arrow_head;

    interval = 2*abs(r)/30;
    

    r_arrow_body_inner = [-15*cos(sign(r)*(0:interval:2*abs(r)));
                          -15*sin(sign(r)*(0:interval:2*abs(r)));
                          repmat(1,1,length(sign(r)*(0:interval:2*abs(r))))]*scaling_factor;

    r_arrow_body_outer = [-17*cos(sign(r)*(0:interval:2*abs(r)));
                          -17*sin(sign(r)*(0:interval:2*abs(r)));
                          repmat(1,1,length(sign(r)*(0:interval:2*abs(r))))]*scaling_factor;

    r_arrow = [r_arrow_body_inner, r_arrow_head, flip(r_arrow_body_outer,2)];



    r_arrow = rot*init_rot*r_arrow +[pos(1);pos(2);0.3];

    r_arrow_patch.Vertices = r_arrow';
    r_arrow_patch.Faces = 1:1:length(r_arrow);
    
end