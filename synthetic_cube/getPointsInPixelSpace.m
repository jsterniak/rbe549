function points_in_image = getPointsInPixelSpace(object_points,...
    camera_location, camera_orientation, camera_parameters)
%GETPOINTSINPIXELSPACE Converts the points from world coordinates to pixel
%   Projects the object_points onto the image plane for a given camera with
%   camera_parameters at camera_location with camera_orientation (ZYX euler
%   angles)
assert(isvector(camera_orientation) && 3 == length(camera_orientation),...
    'camera_orientation must be ZYX euler angle vector');
% invert angles and apply in reverse sequence to determine transformation
% matrix
world_orientation = -[camera_orientation(3), camera_orientation(2), camera_orientation(1)];
world_to_camera_rotation_matrix = eul2rotm(world_orientation,'XYZ');
world_to_camera_rotation_matrix = [world_to_camera_rotation_matrix zeros(3,1); zeros(1,3) 1];

assert(isvector(camera_location) && 3 == length(camera_location),...
    'camera_location must be x,y,z triplet in world coordinates');
if 1 == size(camera_location,1)
    camera_location = camera_location';
end
world_to_camera_translation_matrix = [eye(3,3) -camera_location; zeros(1,3) 1];

object_points = [object_points ones(size(object_points,1),1)];
points_in_image_transposed = world_to_camera_rotation_matrix * world_to_camera_translation_matrix * object_points';

% NaN out points that have a Z less than focal length since these would not
% be visible and could cause singularity in normalization
max_focal_length = max(camera_parameters.IntrinsicMatrix(1,1),...
    camera_parameters.IntrinsicMatrix(2,2));
max_focal_length_m = max_focal_length / 1000;
behind_viewing_plane_points = points_in_image_transposed(3,:) < max_focal_length_m;
points_in_image_transposed(:,behind_viewing_plane_points) = NaN; 

% normalize to get resulting 2-D point vector with unity 3rd row by
% dividing by lambda (homogeneous factor = Z of point)
% lambda*(u; v; 1) = (K 0) * P
for point_dimension = 1:3
    points_in_image_transposed(point_dimension,:) =...
        points_in_image_transposed(point_dimension,:) ./ points_in_image_transposed(3,:);
end
% convert from normalized image space to pixel space
intrinsic_matrix = [camera_parameters.IntrinsicMatrix; 0 0 0];
points_in_image_transposed = intrinsic_matrix' * points_in_image_transposed;
points_in_image = points_in_image_transposed(1:2,:)';

% round to nearest pixel
points_in_image = round(points_in_image);

% NaN points that are out of viewing area
out_of_viewing_angle_points = points_in_image(:,1) < 1 | points_in_image(:,2) < 1 |...
    points_in_image(:,1) > camera_parameters.IntrinsicMatrix(3,1) * 2 |...
    points_in_image(:,2) > camera_parameters.IntrinsicMatrix(3,2) * 2;
points_in_image(out_of_viewing_angle_points,:) = NaN;

end

