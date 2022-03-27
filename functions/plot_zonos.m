function p_list = plot_zonos(vehrs, xyh, should_mirror, p_color)
    x = xyh(1);
    y = xyh(2);
    h = xyh(3);
    num_zonos = length(vehrs);
    p_list = [];
    for zono_idx = 1:num_zonos
        zono = vehrs{zono_idx};
        tz = transform_zono(zono, h, [x; y], should_mirror);
        p = plot(tz);
        p_color(4) = 1;
        p.Color = p_color;
        p_list = [p_list, p];
    end
end