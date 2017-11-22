function runSfm
% https://www.mathworks.com/help/vision/examples/structure-from-motion-from-multiple-views.html?requestedDomain=www.mathworks.com#d119e5950
%%
% load the dataset
close all;
synthetic_data = load('sfm_cube.mat');
cameraParams = synthetic_data.camera_intrinsics;
%%
% perfect feature detection
prevPoints = squeeze(synthetic_data.points_in_pixels(1,:,:));

% perfect feature description, point index is the descriptor
prevFeatures = (1:size(synthetic_data.points_in_pixels,2))';

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints), 'Location', ...
    zeros(1, 3, 'like', prevPoints));
%%
for i = 2:size(synthetic_data.points_in_pixels,1)
    % Detect, extract and match features.
    currPoints   = squeeze(synthetic_data.points_in_pixels(i,:,:));
    currFeatures = prevFeatures;
    indexPairs = [prevFeatures currFeatures];

    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1),:);
    matchedPoints2 = currPoints(indexPairs(:, 2),:);

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, cameraParams);

    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));

    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};

    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);

    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;
end
%%
% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-3, loc1(1)+3]);
ylim([loc1(2)-2, loc1(2)+2]);
zlim([loc1(3)-1, loc1(3)+6]);
camorbit(0, -30);

title('Refined Camera Poses');
%%
% Initialize the point tracker.
prevPoints = squeeze(synthetic_data.points_in_pixels(1,:,:));

% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:size(synthetic_data.points_in_pixels,1)
    % Track the points.
    currPoints = squeeze(synthetic_data.points_in_pixels(i,:,:));
    validIdx = (1:size(synthetic_data.points_in_pixels,2))';

    % Clear the old matches between the points.
    if i < size(synthetic_data.points_in_pixels,1)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);

    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);
%%
% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-3, loc1(1)+3]);
ylim([loc1(2)-2, loc1(2)+2]);
zlim([loc1(3)-1, loc1(3)+6]);
camorbit(0, -30);

title('Dense Reconstruction');
end

