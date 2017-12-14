function features = get_sift_features(image, x, y, feature_width, scale, show_progress)
%GET_SIFT_FEATURES Computes SIFT feature vectors
%   Computes SIFT feature vectors for each (x, y) image keypoint pair with
%   feature_width in pixels
assert(isvector(x) && isvector(y), 'x and y must be 1-D');
assert(true == min(length(x) == length(y)), 'x and y must have same length');
if 6 > nargin
    show_progress = true;
end

% for now, no keypoint orientation is used
keypoint_orientation = 0;

[gradient_magnitudes, gradient_orientations] = compute_gradient_pyramids(image);

features = [];
if show_progress
    fprintf('\nFeature Descriptor Generation Progress:\n');
    fprintf([repmat('.',1,floor(length(x)/200)) '\n\n']);
end
parfor keypoint = 1:length(x)
    x_at_level = get_other_pyramid_level_axis_index(x(keypoint),...
        size(cell2mat(gradient_magnitudes(1)),2),...
        size(cell2mat(gradient_magnitudes(scale(keypoint))),2));
    y_at_level = get_other_pyramid_level_axis_index(y(keypoint),...
        size(cell2mat(gradient_magnitudes(1)),1),...
        size(cell2mat(gradient_magnitudes(scale(keypoint))),1));
    features(keypoint,:) = my_sift_features(cell2mat(gradient_magnitudes(scale(keypoint))),...
        cell2mat(gradient_orientations(scale(keypoint))),...
        x_at_level, y_at_level, keypoint_orientation, feature_width);
    
    if show_progress && 0 == mod(keypoint, 200)
        fprintf('\b|\n');
    end
end
end

function [gradient_magnitudes, gradient_orientations] = compute_gradient_pyramids(...
    image)
[G, ~] = create_sift_laplacian_pyramid(image);
% compute image gradient magnitudes and orientations over entire image and
% supply to sift descriptor for all keypoints
gradient_magnitudes = cell(length(G), 1);
gradient_orientations = gradient_magnitudes;
for level = 1:length(G)
    Gy = compute_image_derivative(cell2mat(G(level)), 1, 'gaussian', 1.5);
    Gx = compute_image_derivative(cell2mat(G(level)), 2, 'gaussian', 1.5);
    gradient_magnitudes(level) = {sqrt(Gx .* Gx + Gy .* Gy)};
    gradient_orientations(level) = {atan2(Gy, Gx)};
end
end
