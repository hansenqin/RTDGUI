function p_list = plot_sliced_frs(frs_file, manu_type_int, frs_indices, state_pred, uvrk)
    disp('Plot sliced');
    xyh = state_pred(1:3);
    u0_idx = frs_indices(1);
    idx0 = frs_indices(2);
    idx1 = frs_indices(3);
    mega = frs_file.M_mega{u0_idx};
    switch char(manu_type_int)
        case double('0')
            tb = mega('Au');
            frs = tb{idx0};
            param_dim = 11;
        case double('1')
            tb = mega('dir');
            frs = tb{idx0, idx1};
            param_dim = 12;
        case double('2')
            tb = mega('lan');
            frs = tb{idx0, idx1};
            param_dim = 12;
        case double('N')
            return;
        otherwise
            return;
    end
    disp('Slicing');
    slc_dims = [7; 8; 9; param_dim];
%     uvrk = frs.vehRS_save{1}.Z(slc_dims, 1);
    k_param = uvrk(4);
    should_mirror = k_param < 0;
    zono_color = 'g';
    uvrk_new = [uvrk(1:3); k_param];
    frs_sliced = slice_frs(frs, uvrk_new, slc_dims, true);
    disp('Sliced');
    p_list = plot_zonos(frs_sliced, xyh, should_mirror, zono_color);
%     frs_sliced = slice_frs(frs, uvrk, slc_dims, false);
%     plot_zonos(frs_sliced, xyh, should_mirror);
end