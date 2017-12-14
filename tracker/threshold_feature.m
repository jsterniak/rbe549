function binary_output_matrix = threshold_feature(feature_matrix, sigma_cutoff)
%THRESHOLD_FEATURE Thresholds an input matrix and produces a binary output
%   Assumes a normal distribution of data in the input feature_matrix and
%   supresses all points less than sigma_cutoff sigmas and greater than
%   -sigma_cutoff sigmas from the mean
matrix_size = size(feature_matrix);
assert(2 == length(matrix_size), 'feature_matrix must be 2-D');

feature_mean = mean2(feature_matrix);
feature_std = std2(feature_matrix);

binary_output_matrix = feature_matrix > feature_mean +...
                                        sigma_cutoff*feature_std |...
                       feature_matrix < feature_mean -...
                                        sigma_cutoff*feature_std;
end

