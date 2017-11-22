function displayPointsInWorldAndPixels(points_in_world,points_in_pixels,camera_parameters)
%DISPLAYPOINTSINWORLDANDPIXELS Scatter plot in 3d and 2d of points
close all
colors = {'r' 'g' 'b' 'm' 'c' 'y' 'k'};
color_iter = 1;
line_width = 2;
subplot(1,2,1)
for point_iter = 1:size(points_in_world,1)
    scatter3(points_in_world(point_iter,1), points_in_world(point_iter,2), points_in_world(point_iter,3),...
        colors{color_iter}, 'linewidth',line_width);
    color_iter = color_iter + 1;
    if color_iter > length(colors)
        color_iter = 1;
        line_width = line_width + 2;
    end
    hold on
end
color_iter = 1;
line_width = 2;
% image coordinates are (1,1) for top left corner so need to invert y
pixel_u_max = camera_parameters.IntrinsicMatrix(3,1) * 2;
pixel_v_max = camera_parameters.IntrinsicMatrix(3,2) * 2;
subplot(1,2,2)
for point_iter = 1:size(points_in_pixels,1)
    scatter(points_in_pixels(point_iter,1), pixel_v_max+1 - points_in_pixels(point_iter,2),...
        colors{color_iter},'linewidth',line_width);
    color_iter = color_iter + 1;
    if color_iter > length(colors)
        color_iter = 1;
        line_width = line_width + 2;
    end
    hold on
end
axis image;
xlim([1 pixel_u_max])
ylim([1 pixel_v_max])
end
