classdef image_feature_tracker
    properties
        max_track_length_
        max_keypoints_to_track_
        %tracks_ is a number of keypoints X length of track X image
        %coordinate dimension (row = 1, col = 2) matrix
        tracks_
        last_feature_descriptors_
        track_insertions_
        nndr_match_threshold_
    end
    methods
        function obj = image_feature_tracker(max_track_length, max_keypoints_to_track, nndr_match_threshold)
            obj.max_track_length_ = max_track_length;
            obj.max_keypoints_to_track_ = max_keypoints_to_track;
            obj.tracks_ = NaN * ones(max_keypoints_to_track, max_track_length, 2);
            obj.last_feature_descriptors_ = [];
            obj.track_insertions_ = 0;
            obj.nndr_match_threshold_ = nndr_match_threshold;
        end
        function obj = addToTrack(obj, keypoints_x, keypoints_y, feature_descriptors)
            % for best results, sort keypoints in decreasing confidence
            % before attempting to addToTrack
            assert(isvector(keypoints_x) && isvector(keypoints_y),...
                'keypoints_x and keypoints_y must be vectors');
            assert(length(keypoints_x) == length(keypoints_y),...
                'keypoints_x and keypoints_y must have the same length');
            if length(keypoints_x) ~= size(keypoints_x,1)
                keypoints_x = keypoints_x';
            end
            if length(keypoints_y) ~= size(keypoints_y,1)
                keypoints_y = keypoints_y';
            end
            
            if obj.track_insertions_ >= obj.max_track_length_
                warning('tracker is full, discarding frame');
            elseif 0 >= obj.track_insertions_
                obj.track_insertions_ = 1;
                
                keypoints_to_add = min(obj.max_keypoints_to_track_, length(keypoints_x));
                obj.tracks_(1:keypoints_to_add,1,1) = keypoints_x(1:keypoints_to_add);
                obj.tracks_(1:keypoints_to_add,1,2) = keypoints_y(1:keypoints_to_add);
                
                obj.last_feature_descriptors_ = feature_descriptors(1:keypoints_to_add,:);
            else
                obj.track_insertions_ = obj.track_insertions_ + 1;
                
                [matches, ~] = getNndrMatches(obj.last_feature_descriptors_,...
                    feature_descriptors, obj.nndr_match_threshold_);
                obj.tracks_(matches(:,1),obj.track_insertions_,1) = keypoints_x(matches(:,2));
                obj.tracks_(matches(:,1),obj.track_insertions_,2) = keypoints_y(matches(:,2));

                % only update recent feature descriptor if a match was
                % found, otherwise hold on to earlier descriptor in hopes
                % it pops back up in subsequent frame
                
                % todo: should this have some model or weighting rather
                % than just letting it change to closest match?
                weight_factor = 1;
                descriptor_delta = feature_descriptors(matches(:,2),:) - obj.last_feature_descriptors_(matches(:,1),:);
                obj.last_feature_descriptors_(matches(:,1),:) = obj.last_feature_descriptors_(matches(:,1),:) +...
                    weight_factor * descriptor_delta;
            end
        end
        function track_lengths = getTrackLengths(obj)
            track_lengths = sum(~isnan(obj.tracks_(:,:,1)),2);
        end
        function top_tracks = getLongestTracks(obj, track_count)
            track_lengths = getTrackLengths(obj);
            % sort tracks by length
            [~, sorted_track_indices] = sort(track_lengths,'descend');
            tracks_to_return = min(track_count,size(sorted_track_indices,1));
            top_tracks = obj.tracks_(sorted_track_indices(1:tracks_to_return),:,:);
            top_tracks(end+1:track_count,:,:) = NaN;
        end
        function moving_tracks = getMovingTracks(obj, minimum_track_length)
            track_lengths = getTrackLengths(obj);
            minimum_track_length = max(3,minimum_track_length);
            tracks_with_motion = track_lengths >= minimum_track_length &...
                (std(obj.tracks_(:,:,1),0,2,'omitnan') > 10 |...
                std(obj.tracks_(:,:,2),0,2,'omitnan') > 10);
            moving_tracks = obj.tracks_(tracks_with_motion,:,:);
        end
        function showTracks(obj, tracks, track_count_to_show, image_frame)
            imshow(image_frame);
            hold on;
            
            colors = {'r' 'g' 'b' 'm' 'y' 'c'};
            markers = {'x-' 'o-' 'd-'};
            color_iter = 1;
            marker_iter = 1;
            for track_iter = 1:min(track_count_to_show, size(tracks,1))
                plot(tracks(track_iter,:,1),...
                    tracks(track_iter,:,2),...
                    [colors{color_iter} markers{marker_iter}]);
                color_iter = color_iter + 1;
                marker_iter = marker_iter + 1;
                if color_iter > length(colors)
                    color_iter = 1;
                end
                if marker_iter > length(markers)
                    marker_iter = 1;
                end
            end
        end
        function showTopTracks(obj, track_count_to_show, image_frame)
            top_tracks = getLongestTracks(obj, track_count_to_show);
            obj.showTracks(top_tracks, track_count_to_show, image_frame); 
        end
    end
end
        