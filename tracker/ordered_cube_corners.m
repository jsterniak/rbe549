files = dir('ordered_cube/*.jpg');

for file_iter = 1:length(files)
    filename = fullfile(files(file_iter).folder, files(file_iter).name);
    cube_image = rgb2gray(im2single(imread(filename)));
    cube_image = imgaussfilt(cube_image, 6);
    cube_image = imresize(cube_image,0.25);
    [keypoints_x, keypoints_y, keypoints_cornerness] = get_harris_points(cube_image, 16);
    keypoints_to_show = 500;
    keypoints_to_show = min(keypoints_to_show,length(keypoints_x));
    [~, sorted_indices] = sort(keypoints_cornerness);
    keypoints_x = keypoints_x(sorted_indices(1:keypoints_to_show));
    keypoints_y = keypoints_y(sorted_indices(1:keypoints_to_show));
    radius = 4;
    centers = [keypoints_x keypoints_y];
    close all;
    figure(1)
    imshow(cube_image);
    hold on;
    viscircles(centers, radius*ones(size(keypoints_x)));

%     keypoints = detectMinEigenFeatures(cube_image);
%     figure(2)
%     imshow(cube_image);
%     hold on;
%     plot(keypoints);

    pause;
end