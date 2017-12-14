function grabFrames
close all;
camera_object = webcam;
pause
frame_count = 4;
camera_images = cell(frame_count,1);
for frame_iter = 1:frame_count
    camera_image = snapshot(camera_object);
    camera_images{frame_iter} = camera_image;
    pause(1);
    fprintf('frame: %d \n',frame_iter);
end

save('test_frames.mat','camera_images');

clear camera_object;
end

