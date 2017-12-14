function [subsample, sample_x_axis, sample_y_axis] = subsample_matrix(matrix,...
    x_center, y_center, x_width, y_width, extrap_method, interpolate)
%SUBSAMPLE_MATRIX Subsamples a 2-D matrix
%   Genereates a 2-D matrix centered around (x_center, y_center) with
%   x_width and y_width length sides with border handling according to
%   extrap_method 'duplicate' or 'zero'. (x_center, y_center) are in matlab
%   index convention, i.e. first index is 1, and can be a pair of
%   any real numbers, interpolation can be performed if set to true to
%   place subsample center directly at (x_center, y_center), otherwise,
%   (x_center, y_center) is rounded to the nearest cell and becomes the top
%   left cell of the 4 cells that make up the center if an even width is
%   specified or the center cell if an odd width is specified.
%   sample_x_axis and sample_y_axis return the axis coordinates of the
%   of the pixels that make up the subsample. If interpolated, the values
%   may be fractional
matrix_size = size(matrix);
assert(2 == length(matrix_size), 'matrix must be 2-D');
assert(strcmp(extrap_method, 'duplicate') || strcmp(extrap_method, 'zero'), 'method must be ''duplicate'' or ''zero''');
assert(isscalar(x_center) && isscalar(y_center), '(x_center, y_center) must be single point');

% if interpolating, we need to grab borders to be able to interpolate into
% them
if interpolate
    x_width_padded = x_width + 2;
    y_width_padded = y_width + 2;
else
    x_width_padded = x_width;
    y_width_padded = y_width;
end
x_top_left_center_cell_index = round(x_center);
y_top_left_center_cell_index = round(y_center);
patch = matrix_slice(matrix,...
    x_top_left_center_cell_index, y_top_left_center_cell_index,...
    x_width_padded, y_width_padded, extrap_method);

% zero point moves to top left so shift canonical axes of patch
patch_x_axis = ceil(calc_canonical_axis(x_width_padded));
patch_y_axis = ceil(calc_canonical_axis(y_width_padded));
if interpolate
    % compute final desired axes without padding
    sample_x_axis = calc_canonical_axis(x_width);
    sample_y_axis = calc_canonical_axis(y_width);
    % interpolate to final axes
    [patch_x_grid, patch_y_grid] = meshgrid(patch_x_axis, patch_y_axis);
    [sample_x_grid, sample_y_grid] = meshgrid(sample_x_axis, sample_y_axis);
    % todo: check if 'linear' or 'cubic' or some other interpolation yields
    % best results
    subsample = interp2(patch_x_grid, patch_y_grid, patch,...
                        sample_x_grid, sample_y_grid, 'cubic');
else
    subsample = patch;
    sample_x_axis = patch_x_axis;
    sample_y_axis = patch_y_axis;
end
end

function subsample = matrix_slice(matrix, x_top_left_center, y_top_left_center,...
    x_width, y_width, extrap_method)
matrix_size = size(matrix);
[min_x, max_x] = calc_min_max_indices(x_top_left_center, x_width);
[min_y, max_y] = calc_min_max_indices(y_top_left_center, y_width);

[left_spill, right_spill] = calc_spill(matrix_size(2), min_x, max_x);
[top_spill, bottom_spill] = calc_spill(matrix_size(1), min_y, max_y);

% convert method to padarray convention
if strcmp(extrap_method, 'zero')
    extrap_method = 0;
elseif strcmp(extrap_method, 'duplicate')
    extrap_method = 'replicate';
end
vertical_pad = double(max(top_spill, bottom_spill));
horizontal_pad = double(max(left_spill, right_spill));
padded_matrix = padarray(matrix, [vertical_pad, horizontal_pad],...
                         extrap_method, 'both');

subsample = padded_matrix(vertical_pad + min_y:vertical_pad + max_y,...
                          horizontal_pad + min_x:horizontal_pad + max_x);
end

function [min_index, max_index] = calc_min_max_indices(center, width)
min_index = floor(center) - ceil(width / 2) + 1;
max_index = min_index + width - 1;
end

function [bottom_spill, top_spill] = calc_spill(max_matrix_index, min_index, max_index)
bottom_spill = max(0, 1 - min_index);
top_spill = max(0, max_index - max_matrix_index);
end

function canonical_axis = calc_canonical_axis(axes_indices_count)
axis_max_index = (axes_indices_count - 1) / 2;
canonical_axis = linspace(-axis_max_index, axis_max_index, axes_indices_count);
end