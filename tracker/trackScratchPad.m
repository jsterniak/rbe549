if ~exist('camera_images','var')
    load('test_frames.mat');
end
close all

tracker = image_feature_tracker(10, 500, 0.7);

for frame_iter = 1:length(camera_images)
    camera_image = camera_images{frame_iter};
    camera_image = rgb2gray(camera_image);
    camera_image = imgaussfilt(camera_image, 3);
    camera_image = imresize(camera_image,0.5);
    figure(1)
    subplot(2,2,frame_iter)
    imshow(camera_image);
    hold on;
%     [keypoints_x, keypoints_y, keypoints_cornerness] = get_harris_points(camera_image, 2);
    [keypoints_x, keypoints_y, keypoints_cornerness, scale, ~] = getSiftKeypoints(camera_image, 16);
    keypoints_to_show = 500;
    keypoints_to_show = min(keypoints_to_show,length(keypoints_x));
    [~, sorted_indices] = sort(keypoints_cornerness);
    keypoints_x = keypoints_x(sorted_indices(1:keypoints_to_show));
    keypoints_y = keypoints_y(sorted_indices(1:keypoints_to_show));
    scale = scale(sorted_indices(1:keypoints_to_show));
    radius = 4;
    centers = [keypoints_x keypoints_y];
    viscircles(centers, radius*ones(size(keypoints_x)));
    
%     features = get_image_patches(camera_image, keypoints_x, keypoints_y, 16, true);
    features = get_sift_features(camera_image, keypoints_x, keypoints_y, 16, scale, false);
    tracker = tracker.addToTrack(keypoints_x, keypoints_y, features);
end
pause(0.5)
figure(2)
for frame_iter = 1:length(camera_images)
    subplot(1,length(camera_images),frame_iter)
    camera_image = camera_images{frame_iter};
    camera_image = rgb2gray(camera_image);
    camera_image = imgaussfilt(camera_image, 2);
    camera_image = imresize(camera_image,0.5);
    tracker.showTopTracks(25, camera_image);
end