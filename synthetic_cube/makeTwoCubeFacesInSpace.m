function cube_points = makeTwoCubeFacesInSpace(side_length, cube_center)
%MAKETWOCUBEFACESINSPACE creates points on two faces of a cube
%   generates 15 x,y,z triplets representing points on a cube centered
%   at the x,y,z triplet center with sides of length side_length
unit_cube = makeTwoUnitCubeFacesWithVertexAtOrigin;

unit_cube_center = 0.5*ones(1,3);
assert(isvector(cube_center) && 3 == length(cube_center), 'cube_center must be x,y,z vector');
translation = cube_center - unit_cube_center;

cube_points = side_length * unit_cube + translation;
end

function cube_points = makeTwoUnitCubeFacesWithVertexAtOrigin
x_axis = [1 0 0];
y_axis = [0 1 0];
z_axis = [0 0 1];
axis_scalers = [0 0.5 1];
cube_points = [];
for z_scaler_iter = 1:length(axis_scalers)
    for x_scaler_iter = 1:length(axis_scalers)
        cube_points(end+1,:) = x_axis * axis_scalers(x_scaler_iter) +...
            z_axis * axis_scalers(z_scaler_iter);
    end
    for y_scaler_iter = 2:length(axis_scalers)
        cube_points(end+1,:) = y_axis * axis_scalers(y_scaler_iter) +...
            z_axis * axis_scalers(z_scaler_iter);
    end
end
end
