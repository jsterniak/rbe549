function feature_vector = my_sift_features(gradient_magnitude, gradient_orientation,...
                                            x, y, orientation, feature_width)
%MY_SIFT_FEATURES computes SIFT feature vector 
%   Creates a SIFT feature_vector for the keypoint centered at (x,y) in the
%   supplied image for a patch of size feature_width x feature_width pixels

assert(ismatrix(gradient_magnitude) && ismatrix(gradient_orientation), 'gradients must be 2-D');

% choose how many cells wide the feature should be
% default sift is 4
feature_width_cells = 4;
% make sure we can break feature_width into the width in cells
feature_width_pixels = feature_width_cells * ceil(feature_width / feature_width_cells);
% choose how many orientations should be binned in histogram
% default sift is 8
feature_orientations = 8;
% this gives feature vector length = 4 x 4 x 8 = 128

% resample gradients to create image grid centered on keypoint
% we will softly bin orientations so we need to get a patch larger than
% feature_width by feature_width_pixels/feature_width_cells / 2 * 2
patch_width = feature_width_pixels * (1 + 1/feature_width_cells);
[magnitude_patch, patch_axis, ~] = subsample_matrix(gradient_magnitude, x, y,...
    patch_width, patch_width, 'duplicate', true);
[orientation_patch, ~, ~] = subsample_matrix(gradient_orientation, x, y,...
    patch_width, patch_width, 'duplicate', true);

feature_vector = bin_gradients(magnitude_patch, orientation_patch, patch_axis,...
    feature_width_cells, feature_width_pixels, feature_orientations, orientation);

% normalize feature vector
feature_vector = normalize_vector(feature_vector);
% clip feature vector
feature_vector(feature_vector > 0.2) = 0.2;
% re-normalize feature vector
feature_vector = normalize_vector(feature_vector);
end

function normed_vector = normalize_vector(raw_vector)
vector_norm = norm(raw_vector);
normed_vector = raw_vector;
if eps < vector_norm
    normed_vector = raw_vector / vector_norm;
end
end

function feature_vector = bin_gradients(magnitude_patch, orientation_patch, patch_axis,...
    feature_width_cells, feature_width_pixels, feature_orientations, orientation)
% iterate through all patch points and update bin values in feature vector
feature_vector = zeros(1, feature_width_cells * feature_width_cells * feature_orientations);
cell_width_pixels = feature_width_pixels / feature_width_cells;
for gradient_row = 1:size(magnitude_patch, 1)
    for gradient_column = 1:size(magnitude_patch, 2)
        pixel_orientation = orientation_patch(gradient_row, gradient_column) + orientation;
        weight_magnitude = magnitude_patch(gradient_row, gradient_column);
        weight_keypoint_distance = calc_keypoint_distance_weight(patch_axis(gradient_row),...
                                                          patch_axis(gradient_column));
        
        weight_vector_cell_membership = calc_cell_membership_weight_vector(...
                patch_axis(gradient_column), patch_axis(gradient_row),...
                feature_width_cells, cell_width_pixels,...
                feature_orientations);

        weight_vector_orientation_bin = calc_orientation_membership_weight_vector(...
            pixel_orientation, feature_orientations, feature_width_cells);

        feature_vector = feature_vector + weight_magnitude * weight_keypoint_distance *...
            weight_vector_cell_membership .* weight_vector_orientation_bin;
    end
end
end

function weight_vector = calc_orientation_membership_weight_vector(...
    orientation, feature_orientations, feature_width_cells)
% softly bin orientations into 3 bins. weight decreases linearly from peak
% at bin center to 0 at n bin widths away
weight_decay_distance_in_bins = 1.5;
peak_weight = 1 / weight_decay_distance_in_bins;
orientation_index = get_orientation_index(orientation, feature_orientations);
neighbor_up_bin = wrap_circular_index(orientation_index + 1, feature_orientations);
neighbor_down_bin = wrap_circular_index(orientation_index - 1, feature_orientations);

bin_width_angle = 2*pi / feature_orientations;

relative_angle_from_bin_center = calc_relative_distance_from_bin_center(...
    orientation, bin_width_angle);
bin_weight = peak_weight * relative_angle_from_bin_center;

if down_bin_is_closer(orientation, bin_width_angle)
    down_bin_angle_from_bin_center = 1 - relative_angle_from_bin_center;
    up_bin_angle_from_bin_center = 1 + relative_angle_from_bin_center;
else
    down_bin_angle_from_bin_center = 1 + relative_angle_from_bin_center;
    up_bin_angle_from_bin_center = 1 - relative_angle_from_bin_center;
end
down_bin_weight = peak_weight * down_bin_angle_from_bin_center;
up_bin_weight = peak_weight * up_bin_angle_from_bin_center;

orientation_weights = zeros(feature_orientations, feature_width_cells, feature_width_cells);
orientation_weights(orientation_index,:,:) = bin_weight;
orientation_weights(neighbor_down_bin,:,:) = down_bin_weight;
orientation_weights(neighbor_up_bin,:,:) = up_bin_weight;
weight_vector = reshape(orientation_weights, 1, numel(orientation_weights));
end

function is_closer = down_bin_is_closer(value, bin_width)
value_in_bin_widths = value / bin_width;
is_closer = round(value_in_bin_widths) == floor(value_in_bin_widths);
end

function index = wrap_circular_index(index, total_indices)
while index > total_indices
    index = index - total_indices;
end
while index < 1
    index = index + total_indices;
end
end

function weight_vector = calc_cell_membership_weight_vector(...
                x, y, feature_width_cells, cell_width_pixels,...
                feature_orientations)
% softly bin based on pixel location relative to cells. Cell count
% across feature must be even to end up with 3 nearest cells rather than 8.
% weight in cell decreases linearly from 1 at cell center to 0 at 1 cell
% width away
assert(0 == mod(feature_width_cells,2), 'feature_width_cells must be even');

[cell_x_index, nearest_x_index] = calc_current_and_neighbor_cell_indices(...
    x, cell_width_pixels, feature_width_cells);
[cell_y_index, nearest_y_index] = calc_current_and_neighbor_cell_indices(...
    y, cell_width_pixels, feature_width_cells);

[distance_weight, x_same_y_near_neighbor_distance_weight,...
    x_near_neighbor_y_same_distance_weight, far_neighbor_distance_weight] =...
    calc_distance_weights(x, y, cell_width_pixels);

weight_matrix = zeros(feature_orientations, feature_width_cells, feature_width_cells);

x_in_bounds = check_cell_index_bounds(cell_x_index, feature_width_cells);
y_in_bounds = check_cell_index_bounds(cell_y_index, feature_width_cells);
neighbor_x_in_bounds = check_cell_index_bounds(nearest_x_index, feature_width_cells);
neighbor_y_in_bounds = check_cell_index_bounds(nearest_y_index, feature_width_cells);
if x_in_bounds && y_in_bounds
    % index must be consistent with orientation assumption
    weight_matrix(:, cell_x_index, cell_y_index) = distance_weight;
end
if x_in_bounds && neighbor_y_in_bounds
    % index must be consistent with orientation assumption
    weight_matrix(:, cell_x_index, nearest_y_index) = x_same_y_near_neighbor_distance_weight;
end
if neighbor_x_in_bounds && y_in_bounds
    %  index must be consistent with orientation assumption
    weight_matrix(:, nearest_x_index, cell_y_index) = x_near_neighbor_y_same_distance_weight;
end
if neighbor_x_in_bounds && neighbor_y_in_bounds
    %  index must be consistent with orientation assumption
    weight_matrix(:, nearest_x_index, nearest_y_index) = far_neighbor_distance_weight;
end
weight_vector = reshape(weight_matrix, 1, numel(weight_matrix));
end

function [distance_weight, x_same_y_near_neighbor_distance_weight,...
    x_near_neighbor_y_same_distance_weight, far_neighbor_distance_weight] =...
    calc_distance_weights(x, y, cell_width_pixels)
% calculate distances in units of cell width
x_distance_from_origin = calc_relative_distance_from_bin_center(x, cell_width_pixels);
y_distance_from_origin = calc_relative_distance_from_bin_center(y, cell_width_pixels);
assert(0<=x_distance_from_origin && 0.5>=x_distance_from_origin,'x must be [0,0.5]');
assert(0<=y_distance_from_origin && 0.5>=y_distance_from_origin,'y must be [0,0.5]');
neighbor_x_distance_from_origin = 1 - x_distance_from_origin;
neighbor_y_distance_from_origin = 1 - y_distance_from_origin;

distance = norm([x_distance_from_origin y_distance_from_origin]);
x_same_y_near_neighbor_distance = norm([x_distance_from_origin neighbor_y_distance_from_origin]);
x_near_neighbor_y_same_distance = norm([neighbor_x_distance_from_origin y_distance_from_origin]);
far_neighbor_distance = norm([neighbor_x_distance_from_origin neighbor_y_distance_from_origin]);

% weight drops linearly as distance from cell origin increases from 1 to 0
% at 1 cell width away
distance_weight = max(0, 1-distance);
x_same_y_near_neighbor_distance_weight = max(0, 1-x_same_y_near_neighbor_distance);
x_near_neighbor_y_same_distance_weight = max(0, 1-x_near_neighbor_y_same_distance);
far_neighbor_distance_weight = max(0, 1-far_neighbor_distance);
end

function index_in_bounds = check_cell_index_bounds(cell_index, feature_width_cells)
if 1 <= cell_index && feature_width_cells >= cell_index
    index_in_bounds = true;
else
    index_in_bounds = false;
end
end

function [cell_axis_index, nearest_neighbor_cell_axis_index] =...
    calc_current_and_neighbor_cell_indices(coordinate, cell_width_pixels,...
    feature_width_cells)
cell_count_from_center = fix(coordinate / cell_width_pixels);
nearest_index_from_center = round(coordinate / cell_width_pixels);
closer_to_cell_bottom = cell_count_from_center == nearest_index_from_center;

% shift so most negative cell is at 0
cell_count_from_left = cell_count_from_center + feature_width_cells/2;
% negative axis requires one less shift
if -1 == sign(coordinate)
    cell_count_from_left = cell_count_from_left -1;
end
% correct for matlab notation
cell_axis_index = cell_count_from_left + 1;

if closer_to_cell_bottom
    nearest_neighbor_cell_axis_index = cell_axis_index - sign(coordinate);
else
    nearest_neighbor_cell_axis_index = cell_axis_index + sign(coordinate);
end
end

function relative_distance_from_bin_center = calc_relative_distance_from_bin_center(...
    coordinate, bin_width)
coordinate_in_bins = coordinate / bin_width;
relative_distance_from_bin_edge = coordinate_in_bins - round(coordinate_in_bins);
relative_distance_from_bin_center = 0.5 - abs(relative_distance_from_bin_edge);
end

function weight_keypoint_distance = calc_keypoint_distance_weight(pixel_x, pixel_y)
membership_sigma = 16/2; % pixels
% keypoint is at origin
distance_from_keypoint = norm([pixel_x, pixel_y]);
weight_keypoint_distance = gaussmf(distance_from_keypoint, [membership_sigma 0]);
end

function orientation_index = get_orientation_index(orientation, feature_orientations)
% get orientation in range [0, 2*pi)
while orientation < 0
    orientation = orientation + 2*pi;
end
while orientation >= 2*pi
    orientation = orientation - 2*pi;
end

orientation_bin_size = 2*pi / feature_orientations;

% convert to matlab notation
orientation_index = floor(orientation / orientation_bin_size) + 1;
end