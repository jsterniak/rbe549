function trackerDemo

track_sequence_length = 5;

camera_object = webcam;

keypoints_to_track = 500;

% FS = stoploop('STOP');
max_loop_count = 10;
loop_count = 0;
show_features = true;
keypoints_to_show = 500;
while max_loop_count > loop_count% && ~FS.Stop()
    tracker = image_feature_tracker(track_sequence_length, keypoints_to_track, 0.7);
    
    frames_to_process = getWebcamFrames(camera_object, track_sequence_length, true);

    tracker_start_time = tic;

    close all;
    if show_features
        figure(1)
    end
    
    for frame_iter = 1:length(frames_to_process)
        image_frame = frames_to_process{frame_iter};
        
        [keypoints_x, keypoints_y, features] = getImageFeatures(image_frame, 'sift');
        
        if show_features
            image_rows = 2;
            figure(1)
            subplot(image_rows,ceil(track_sequence_length/image_rows),frame_iter)
            imshow(image_frame);
            hold on;
            keypoints_to_show = min(keypoints_to_show,length(keypoints_x));
            radius = 4;
            centers = [keypoints_x keypoints_y];
            viscircles(centers, radius*ones(size(keypoints_x)));
        end
        
        tracker = tracker.addToTrack(keypoints_x, keypoints_y, features);
    end
    
    if show_features
        pause;
    end
    
    tracker_computation_time = toc(tracker_start_time);
    fprintf('Tracking run time: %f s \n', tracker_computation_time);
    
    display_frames = {frames_to_process{1}; frames_to_process{end}};
    displayTracks(display_frames, tracker);
    
    loop_count = loop_count + 1;
end

clear camera_object;
end

function displayTracks(display_frames, tracker)
figure(2);
for frame_iter = 1:length(display_frames)
    subplot(1,length(display_frames),frame_iter)
    camera_image = display_frames{frame_iter};
    minimum_track_length = ceil(tracker.max_track_length_/2);
    tracks_to_show = tracker.getMovingTracks(minimum_track_length);
    track_count_to_show = 25;
    tracker.showTracks(tracks_to_show, track_count_to_show, camera_image);
%     tracker.showTopTracks(3, camera_image);
end
pause;
end

function [keypoints_x, keypoints_y, features] = getImageFeatures(image_frame, method)
switch lower(method)
    case 'sift'
        feature_width = 16;
        [keypoints_x, keypoints_y, keypoints_cornerness, scale, ~] = getSiftKeypoints(image_frame, feature_width);
        features = get_sift_features(image_frame, keypoints_x, keypoints_y, feature_width, scale, false);
    case 'image_patch'
        keypoint_minimum_spacing = 16;
        [keypoints_x, keypoints_y, keypoints_cornerness] = get_harris_points(image_frame, keypoint_minimum_spacing);
        patch_size = 16;
        features = get_image_patches(image_frame, keypoints_x, keypoints_y, patch_size, true);
    otherwise
        assert(false,'invalid feature method');
end

[~, sorted_indices] = sort(keypoints_cornerness);
keypoints_x = keypoints_x(sorted_indices);
keypoints_y = keypoints_y(sorted_indices);
end

function image_frames = getWebcamFrames(camera_object, frame_count, smooth_and_bw_image)
image_frames = cell(frame_count,1);
for frame_iter = 1:frame_count
    frame_start_time = tic;
    camera_image = snapshot(camera_object);
    if smooth_and_bw_image
        % smooth images since webcam is grainy, convert to bw for
        % processing
        camera_image = rgb2gray(camera_image);
        camera_image = imgaussfilt(camera_image, 2);
        camera_image = imresize(camera_image,0.5);
    end
    image_frames{frame_iter} = camera_image;
    frame_capture_time = toc(frame_start_time);
    fprintf('frame capture time: %f s \n',frame_capture_time);
    pause(0.5);
end
end

