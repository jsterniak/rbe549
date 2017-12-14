function [G, L] = create_sift_laplacian_pyramid(image)
%CREATE_SIFT_LAPLACIAN_PYRAMID Wrapper so sift and keypoint detection use
%  same laplacian pyramid
sigma = 2;
subsample_factor = 2;
pyramid_levels = 5;
DoG_approximation = true;
[G, L] = create_laplacian_pyramid(image, pyramid_levels,...
    subsample_factor, sigma, DoG_approximation);
end

