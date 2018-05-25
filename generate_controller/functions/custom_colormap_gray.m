function colmap_custom = custom_colormap_gray(l_colmap)
%CUSTOM_COLORMAP generates a gray custom colormap
%    generates black/gray and white colormap for publication figures

dkgry = 220/255;
ltgry1 = 50/255;
ltgry2 = 90/255;
colmap_custom = repmat(interp1([1,floor(l_colmap/2),l_colmap],...
    [ltgry1,dkgry,ltgry2],1:l_colmap)',[1 3]);
end

