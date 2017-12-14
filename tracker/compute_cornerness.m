function [harris, harmonic_mean] = compute_cornerness(Ix, Iy, sigma)
%COMPUTE_CORNERNESS Computes the harris and harmonic mean cornerness
%matrices
%   Computes the harris feature and harmonic mean for all image pixels from
%   the x and y image derivatives Ix and Iy with Gaussian smoothing with
%   sigma
Ix_size = size(Ix);
Iy_size = size(Iy);
assert(true == min(Ix_size == Iy_size), 'Ix and Iy must be the same size');
if nargin < 2
    sigma = 2;
end

% compute products of derivatives at each pixel
Ix2 = Ix .* Ix;
Ixy = Ix .* Iy;
Iy2 = Iy .* Iy;

% smooth products
kernel = fspecial('Gaussian',sigma*6+1, sigma);
Sx2 = imfilter(Ix2, kernel);
Sxy = imfilter(Ixy, kernel);
Sy2 = imfilter(Iy2, kernel);

% compute features over all pixels
% manually compute over all pixels with matrix operations for speed
H_det = Sx2 .* Sy2 - Sxy .* Sxy;
H_tr = Sx2 + Sy2;

% computer harris cornerness
harris_alpha = 0.06;
harris = H_det - harris_alpha * H_tr .* H_tr;

% compute harmonic mean
% handle divide by zero
trace_near_zero = abs(H_tr) < eps;
harmonic_mean = zeros(Ix_size);
harmonic_mean(~trace_near_zero) = H_det(~trace_near_zero) ./...
                                  H_tr(~trace_near_zero);
% ensure zero trace points are pegged to minimum harmonic mean value so
% they don't falsely appear as keypoints
harmonic_mean(trace_near_zero) = min(min(harmonic_mean));
end

