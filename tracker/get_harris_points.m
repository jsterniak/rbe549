function [x, y, cornerness] = get_harris_points(image, feature_width)
%GET_HARRIS_POINTS Gets image keypoints using Harris corner detector
%   Computes (x, y) points at pixel locations in image and returns pairs in
%   N x 1 vectors for column and row, respectively. Feature_width specifies
%   order of magnitude with which to filter out small areas of high local
%   cornerness. The cornerness of the keypoints is returned in an N x 1
%   vector.
cornerness_metric = 'harmonic_mean';

image_size = size(image);
assert(2 == length(image_size) || 3 == length(image_size), 'image must be 2-D or 3-D');
if 2 == length(image_size)
    image_size(end+1) = 1;
end

% iterate over color channels and take maximum keypoint calc at each pixel
harris_max = -inf * ones(image_size(1), image_size(2));
harmonic_mean_max = harris_max;
for color_channel = 1 : image_size(3)
    Ix = compute_image_derivative(image(:,:,color_channel), 2,...
                                  'gaussian', 1);
    Iy = compute_image_derivative(image(:,:,color_channel), 1,...
                                  'gaussian', 1);
    
    [harris, harmonic_mean] = compute_cornerness(Ix,Iy,1);
    
    harris_max = max(harris_max, harris);
    harmonic_mean_max = max(harmonic_mean_max, harmonic_mean);
end

% create thresholded binary thresholded image of max keypoint calcs
if strcmp(cornerness_metric, 'harmonic_mean')
    binary_corner_image = threshold_feature(harmonic_mean_max, 1);
    raw_corner_image = harmonic_mean_max;
else
    binary_corner_image = threshold_feature(harris_max, 1);
    raw_corner_image = harris_max;
end

[x, y, cornerness] = corner_matrix_to_keypoints(binary_corner_image, raw_corner_image, feature_width);
[~, cornerness_index] = sort(cornerness, 'descend');

x = x(cornerness_index(:));
y = y(cornerness_index(:));
cornerness = cornerness(cornerness_index(:));
% close all;
% imshow(image);
% hold on;
% scatter(x,y,'o', 'linewidth', 2);
end

function [x, y, cornerness] = corner_matrix_to_keypoints(binary_corner_image, raw_corner_image, feature_width)
cornerness_relative_threshold = 0.9;
min_blob_size = feature_width;
min_feature_spacing = feature_width;
connected_blobs = bwconncomp(binary_corner_image);

border_y_lower_keepout = 20; % pixels
border_y_upper_keepout = 20; % pixels
border_x_keepout = 20;

x = [];
y = [];
cornerness = [];
for blob = 1:length(connected_blobs.PixelIdxList)
    % skip tiny blobs
    if min_blob_size > length(cell2mat(connected_blobs.PixelIdxList(blob)))
        continue;
    end
    % locate maximum values within blobs and make them keypoints
    blob_pixel_list = connected_blobs.PixelIdxList{blob};
    blob_vector = raw_corner_image(blob_pixel_list);
    [sorted_pixels, sorted_indices] = sort(blob_vector,'descend');
    for pixel = 1:length(sorted_pixels)
        if sorted_pixels(pixel) / max(sorted_pixels) > cornerness_relative_threshold
            [feature_y, feature_x] = ind2sub(connected_blobs.ImageSize,...
                                    blob_pixel_list(sorted_indices(pixel)));
            if feature_x < border_x_keepout || connected_blobs.ImageSize(2) - feature_x < border_x_keepout ||...
                    feature_y < border_y_upper_keepout ||...
                    connected_blobs.ImageSize(1) - feature_y < border_y_lower_keepout
                continue;
            end
            if isempty(x) || min(sqrt(feature_x - x).^2 + (feature_y - y).^2) >= min_feature_spacing
                x(end+1,1) = feature_x;
                y(end+1,1) = feature_y;
                cornerness(end+1,1) = sorted_pixels(pixel);
            end
        else
            continue;
        end
    end
end
end
