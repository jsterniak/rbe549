function d_image = compute_image_derivative(image, dimension, method, sigma)
%COMPUTE_IMAGE_DERIVATIVE computes derivative of pixels in image
%   Computes the derivative of the single color/intensity channel image
%   along dimension using method 'difference' or 'gaussian' with sigma

% check inputs
image_dimensions = size(image);

assert(length(image_dimensions) == 2, 'single color/intensity channel must be supplied');
assert(2 <= nargin, 'image and dimension must be supplied');
assert(isnumeric(dimension) && dimension >= 1 && dimension <= 2, 'dimension must be 1 or 2');
assert(3 > nargin || strcmp(method,'gaussian') || strcmp(method,'difference'), 'method must be gaussian or difference');
% convert image to single if not already
if isinteger(image)
    image = single(image);
end

if nargin < 3
    method = 'difference';
end

if nargin < 4
    sigma = 1;
end

% create differentiating filter
kernel = [-2 -1 0 1 2]; % original Harris detector filter
% transpose if differentiating y
if 1 == dimension
    kernel = kernel';
end
% convolve with gaussian if gaussian is requested
if strcmp('gaussian',method)
    kernel = imfilter(fspecial('Gaussian', sigma*6+1, sigma), kernel, 'replicate');
end

d_image = imfilter(image, kernel);

% close all;
% subplot(1,2,1);
% imagesc(kernel);
% subplot(1,2,2);
% imagesc(d_image);
end

