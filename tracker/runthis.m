function runthis
close all;
camera_object = webcam;
FS = stoploop('STOP');
while ~FS.Stop()
    camera_image = snapshot(camera_object);
    camera_image = rgb2gray(camera_image);
    camera_image = imgaussfilt(camera_image, 2);
    camera_image = imresize(camera_image,0.5);
    imshow(camera_image);
    hold on;
    [keypoints_x, keypoints_y, keypoints_cornerness] = get_harris_points(camera_image, 16);
    keypoints_to_show = 500;
    keypoints_to_show = min(keypoints_to_show,length(keypoints_x));
    [~, sorted_indices] = sort(keypoints_cornerness);
    keypoints_x = keypoints_x(sorted_indices(1:keypoints_to_show));
    keypoints_y = keypoints_y(sorted_indices(1:keypoints_to_show));
    radius = 4;
    centers = [keypoints_x keypoints_y];
    viscircles(centers, radius*ones(size(keypoints_x)));
    drawnow;
end

clear camera_object;
end

