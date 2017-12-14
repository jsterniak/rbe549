function features = get_image_patches(image, x, y, patch_width, normalize)
%GET_IMAGE_PATCHES Returns intensity patches for all color
%channels at an image location
%   Builds a N x (patch_width^2 * color channel depth) feature vector from
%   the supplied grayscale or color image at each of the provided (x,y)
%   keypoints in the image that are normalized if normalize is set to true

image_size = size(image);
assert(2 == length(image_size) || 3 == length(image_size), 'image must be 2-D or 3-D matrix');
if 2 == length(image_size)
    image_size(end+1) = 1;
end

assert(true == min(size(x) == size(y)), 'x and y must have same size');
assert(isscalar(x) || isvector(x), 'x and y must be 1-D');
assert(islogical(normalize),'normalize must be logical value');
features = zeros(length(x), patch_width^2 * image_size(3));
for keypoint = 1:length(x)
    for color_channel = 1:image_size(3)
        [patch, ~, ~] = subsample_matrix(image(:,:,color_channel),...
            x(keypoint), y(keypoint), patch_width, patch_width,...
            'duplicate', false);
        if normalize
            patch = patch - mean2(patch);
            patch_standard_deviation = std2(patch);
            if eps < abs(patch_standard_deviation)
              patch = patch / patch_standard_deviation;
            else
              patch = NaN * patch;
            end
        end
        features(keypoint,:) = reshape(patch,[1 numel(patch)]);
    end
end
end
