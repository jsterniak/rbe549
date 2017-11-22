function makeCubeImages
%MAKECUBEIMAGES Summary of this function goes here
%   Detailed explanation goes here
camera_intrinsics = makeIdealCameraParameters;

% cube_points = makeCubeInSpace(1, [0 3 0]);
cube_points = makeTwoCubeFacesInSpace(1, [0 3 0]);

close all
world_figure = figure(1);
hold on;
colors = {'r' 'g' 'b' 'm' 'c' 'y' 'k'};
display3dPoints(cube_points, colors, world_figure);

% defaut camera orientation is looking down z, while world is z is up
% [0 0 -pi/2] to look straight down world y axis, orientation is ZYX euler
% angles
camera_start_position = [-1.2 0 0];
camera_start_orientation = [-0.2 0 -pi/2];
camera_mid_position = [-2 0 -0.2];
camera_mid_orientation = [-0.6 0 -pi/2];
camera_end_position = [-2 2 -0.5];
camera_end_orientation = [-1.2 0 -1.3];

start_to_mid_interpolations = 1;
mid_to_end_interpolations = 1;

camera_positions_phase_1 = interpolateVectors(camera_start_position, camera_mid_position, start_to_mid_interpolations);
camera_positions_phase_2 = interpolateVectors(camera_mid_position, camera_end_position, mid_to_end_interpolations);
camera_positions = [camera_positions_phase_1; camera_positions_phase_2(2:end,:)];

camera_orientations_phase_1 = interpolateVectors(camera_start_orientation, camera_mid_orientation, start_to_mid_interpolations);
camera_orientations_phase_2 = interpolateVectors(camera_mid_orientation, camera_end_orientation, mid_to_end_interpolations);
camera_orientations = [camera_orientations_phase_1; camera_orientations_phase_2(2:end,:)];

camera_views_figure = figure(2);
hold on;
points_in_pixels = zeros(size(camera_positions,1),size(cube_points,1),2);
for view_iter = 1:size(camera_positions,1)
    displayCameraInWorld(camera_positions(view_iter,:), camera_orientations(view_iter,:), world_figure);

    figure(camera_views_figure);
    subplot(3, ceil(size(camera_positions,1)/3), view_iter);
    points_in_pixels(view_iter,:,:) = getPointsInPixelSpace(cube_points,...
        camera_positions(view_iter,:), camera_orientations(view_iter,:),...
        camera_intrinsics);
    displayViewInPixels(squeeze(points_in_pixels(view_iter,:,:)), camera_intrinsics, camera_views_figure, colors);
end
save('sfm_cube.mat', 'camera_intrinsics', 'cube_points', 'camera_positions', 'camera_orientations', 'points_in_pixels');
end

function displayViewInPixels(points, camera_intrinsics, camera_views_figure, color_key_cell)
% image coordinates are (1,1) for top left corner so need to invert y
pixel_u_max = camera_intrinsics.IntrinsicMatrix(3,1) * 2;
pixel_v_max = camera_intrinsics.IntrinsicMatrix(3,2) * 2;
color_iter = 1;
line_width = 2;
assert(2 == size(points,2), 'points must be nX2');
assert(iscell(color_key_cell) && isvector(color_key_cell),...
    'color_key_cell must be a cell vector of colors');
figure(camera_views_figure);
for point_iter = 1:size(points,1)
    scatter(points(point_iter,1), pixel_v_max+1 - points(point_iter,2),...
        color_key_cell{color_iter},'linewidth',line_width);
    color_iter = color_iter + 1;
    if color_iter > length(color_key_cell)
        color_iter = 1;
        line_width = line_width + 2;
    end
    hold on
end
axis image;
xlim([1 pixel_u_max])
ylim([1 pixel_v_max])
end

function displayCameraInWorld(camera_position, camera_orientation, world_figure)
    figure(world_figure);
    scatter3(camera_position(1), camera_position(2), camera_position(3),...
        'x', 'linewidth', 3);
    assert(isvector(camera_position) && 3 == length(camera_position),...
        'camera_position must be a 3d vector');
    if 1 == size(camera_position,1)
        camera_position = camera_position';
    end
    view_endpoint = eul2rotm(camera_orientation) * [0; 0; 1] + camera_position;
    line([camera_position(1) view_endpoint(1)],...
         [camera_position(2) view_endpoint(2)],...
         [camera_position(3) view_endpoint(3)]);
end

function display3dPoints(points_3d, color_key_cell, world_figure)
color_iter = 1;
line_width = 2;
assert(3 == size(points_3d,2), 'points_3d must be nX3');
assert(iscell(color_key_cell) && isvector(color_key_cell),...
    'color_key_cell must be a cell vector of colors');
figure(world_figure);
for point_iter = 1:size(points_3d,1)
    scatter3(points_3d(point_iter,1), points_3d(point_iter,2), points_3d(point_iter,3),...
        color_key_cell{color_iter}, 'linewidth',line_width);
    color_iter = color_iter + 1;
    if color_iter > length(color_key_cell)
        color_iter = 1;
        line_width = line_width + 2;
    end
    hold on
end
end

function interpolated_vectors = interpolateVectors(start_vector, end_vector, interpolation_count)
assert(isvector(start_vector) && isvector(end_vector), 'start_vector and end_vector must be vectors');
assert(length(start_vector) == length(end_vector), 'start_vector and end_vector must have same length');

if 1 ~= size(start_vector,1)
    start_vector = start_vector';
end
if 1 ~= size(end_vector,1)
    end_vector = end_vector';
end

assert(interpolation_count >=0, 'interpolation count must be non-negative');
interpolation_factor = (0:1/(interpolation_count+1):1)';

interpolated_vectors = start_vector + interpolation_factor * (end_vector-start_vector);
end