function [x, y, confidence, scale, orientation] = getSiftKeypoints(image, feature_width)
%GETSIFTKEYPOINTS finds keypoint locations at different scales
%  finds keypoints using the harris corner detector in image, which can
%  be grayscale or color with keypoints spaced at feature_width and returns
%  the keypoint coordinates in the keypoint number length x and y vectors
%  with the confidence of each corner in the confidence vector, the
%  Gaussian pyramid level at which the keypoint was strongest in the scale
%  vector, both of which are keypoint number length. The primary
%  orientation of the keypoint would go in the keypoint vector, but it is
%  currently not supported and only returns a scalar of 0.

[G, L] = create_sift_laplacian_pyramid(image);
% for i = 1:5
%     subplot(3,5,i)
%     imshow(cell2mat(G(i)))
%     subplot(3,5,i+5)
%     imshow(cell2mat(L(i)) + 0.5)
%     subplot(3,5,i+10)
%     hist(cell2mat(L(i)));
% end

x = [];
y = [];
confidence = [];
scale = [];
for level = 1:size(G)
    [x_at_level, y_at_level, cornerness] = get_harris_points(G{level}, feature_width);
    [x_maximal_L, y_maximal_L, confidence_maximal_L] =...
        remove_non_maximal_L_keypoints(L, level, x_at_level, y_at_level, cornerness);
    % convert to base image axes for ground truth comparison
    x(end+1:end+1+length(x_maximal_L)-1,1) = get_other_pyramid_level_axis_index(x_maximal_L,...
        size(G{level},2), size(G{1},2));
    y(end+1:end+1+length(y_maximal_L)-1,1) = get_other_pyramid_level_axis_index(y_maximal_L,...
        size(G{level},1), size(G{1},1));
    confidence(end+1:end+1+length(confidence_maximal_L)-1,1) = confidence_maximal_L;
    scale(end+1:end+1+length(x_maximal_L)-1,1) = level*ones(length(x_maximal_L),1);
end

orientation = 0;

% points_to_show = 400;
% [~, sorted_ind] = sort(confidence);
% x = x(sorted_ind(1:points_to_show));
% y = y(sorted_ind(1:points_to_show));
% scale = scale(sorted_ind(1:points_to_show));
% imshow(image);
% hold on
% sigma = 2;
% subsample_factor = 2;
% centers = [x y];
% viscircles(centers, sigma*scale.*(subsample_factor.^(scale-1)));
end

function [x_maximal_L, y_maximal_L, confidence_maximal_L] =...
        remove_non_maximal_L_keypoints(L, level, x_at_level, y_at_level, cornerness)
x_maximal_L = [];
y_maximal_L = [];
confidence_maximal_L = [];
for keypoint = 1:length(x_at_level)
    extremal = true;
    if level > 1
        extremal = compare_L(L, level, level-1, x_at_level(keypoint), y_at_level(keypoint));
    end
    if extremal && level < length(L)
        extremal = compare_L(L, level, level+1, x_at_level(keypoint), y_at_level(keypoint));
    end
    if extremal
        x_maximal_L(end+1,1) = x_at_level(keypoint);
        y_maximal_L(end+1,1) = y_at_level(keypoint);
        confidence_maximal_L(end+1,1) = cornerness(keypoint);
    end
end
end

function is_extremal = compare_L(L, current_level, other_level, x_at_level, y_at_level)
current_L = L{current_level};
keypoint_L_current_level = current_L(y_at_level, x_at_level);

other_L = L{other_level};
x_at_other_level = get_other_pyramid_level_axis_index(x_at_level,...
    size(current_L,2),size(other_L,2));
y_at_other_level = get_other_pyramid_level_axis_index(y_at_level,...
    size(current_L,1),size(other_L,1));
keypoint_L_other_level = other_L(y_at_other_level, x_at_other_level);
if sign(keypoint_L_current_level) == sign(keypoint_L_other_level) &&...
        abs(keypoint_L_current_level) < abs(keypoint_L_other_level)
    is_extremal = false;
else
    is_extremal = true;
end
end