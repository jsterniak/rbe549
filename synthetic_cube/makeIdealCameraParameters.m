function ideal_camera_parameters = makeIdealCameraParameters
%MAKEIDEALCAMERAPARAMETERS creates a generic camera parameter set
%   Generates a Matlab cameraparameters object with 700 mm x and y focal
%   length, no skew, 400 pixel x and y principle point, and no distortion
focal_length = [700 700];
optical_center = [400 400];
skew = 0;
radial_distortion_coefficients = [0 0 0];

projection_matrix = [focal_length(1) 0 0;...
                     skew focal_length(2) 0;...
                     optical_center(1) optical_center(2) 1];

ideal_camera_parameters = cameraParameters('IntrinsicMatrix', projection_matrix,...
                                           'RadialDistortion', radial_distortion_coefficients);
end
