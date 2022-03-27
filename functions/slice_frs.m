function vehrs_out = slice_frs(frs, uvrk, slc_dims, act_slice)
    vehrs = frs.vehRS_save;
    vehrs_out = {}; 
    num_zonos = length(vehrs);
    for zono_idx = 1:num_zonos
        zono = vehrs{zono_idx};
        try
            vehrs_out{end+1} = zonotope_slice(zono, slc_dims, uvrk);
        catch
            vehrs_out{end+1} = zono;
        end
    end
end