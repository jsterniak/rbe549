function [G, L] = create_laplacian_pyramid(image,...
    pyramid_levels, subsample_factor, sigma, DoG_approximation)
%CREATE_LAPLACIAN_PYRAMID Creates a Laplacian pyramid
%  Creates Gaussian (G) and Laplacian (L) pyramids of level pyramid_levles
%  from image. G and L are cell where G{i}, L{i} stores the i-th level of
%  Gaussian and Laplacian pyramid, respectively. Each level is subsampled
%  by subsample_factor and filtered with Gaussian sigma. The DoG
%  approximation can be used for faster run time and approximates the
%  Laplacian by taking the difference of the gaussian blurred image at each
%  pyramid level.
assert(ismatrix(image),'image must be 2-D');

% create empty cells for pyramids
% store 0 level as well so storage for N+1 levels is needed
G = cell(pyramid_levels+1,1);
L = cell(pyramid_levels+1,1);

% compute levels of pyramid, L(level) = G(level) - G(level+1)
% at level level==level_max, L=G so that the base image can be fully reconstructed
% store original image as G0
G(1) = {image};
if ~DoG_approximation
    log_kernel = fspecial('log',6*sigma+1, sigma);
end
for level = 0 : pyramid_levels - 1
    % matlab index notation
    current_level_index = level + 1;
    next_level_index = (level+1) + 1;
    
    % low pass filter the current pyramid level to obtain next level
    G_full_resolution = imgaussfilt(cell2mat(G(current_level_index)), sigma);
    % subsample the image to create the next level G
    G(next_level_index) = {imresize(G_full_resolution, 1/subsample_factor,'bicubic')};
    
    if DoG_approximation
        % interpolate next level G to current level size to calculate L
        % make sure image size is retained
        current_level_size = size(G_full_resolution);
        next_G_upsampled = imresize(cell2mat(G(next_level_index)),current_level_size,'bicubic');
        % compute current level L
        L(current_level_index) = {cell2mat(G(current_level_index)) - next_G_upsampled};
    else
        L(current_level_index) = {imfilter(cell2mat(G(current_level_index)), log_kernel, 'replicate')*sigma^2};
    end
end
% set last level L equal to last level G
L(end) = G(end);
end