function [matches, confidences] = getNndrMatches(features1, features2, NNDR_match_threshold)
%GETNNDRMATCHES Nearest Neighbor Detection Ratio feature matching
%   Matches features in feature1 and feature2 based on euclidean distance
%   between the (features X feature vector) contained in features1 and
%   features2. The ratio of nearest neighbor distance to next nearest
%   neighbor distance must be less than NNDR_match_threshold to be added to
%   the matches. matches is size good matches X 2, and confidences is a
%   vector with length equal to the row count of matches
[distance_from_current_feature, index_of_distance] = pdist2(features2, features1,...
    'euclidean', 'Smallest', 2);

matches = [1:size(features1,1); index_of_distance(1,:)];

distance_ratio = distance_from_current_feature(1,:) ./ distance_from_current_feature(2,:);

confidences = 1 - distance_ratio;

good_match = eps < distance_from_current_feature(2,:) & NNDR_match_threshold > distance_ratio;

matches(:,~good_match) = [];
matches = matches';

confidences = confidences(good_match)';
end

