% pic_orig = camera image in RGB 
% numObj = # of objects on the board
% cameraParams 

% returns a list of centroids and areas sorted according to the x
% coordinate of the centroid
function [centroids, areaList] = diskSize(pic_orig, cameraParams)
%% BLOB ANALYSIS
[pic, ~] = undistortImage(pic_orig, cameraParams, 'OutputView', 'full');
BW_img = detectDisk(pic);
imshow(BW_img)

blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', true,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 750, 'ExcludeBorderBlobs', true);
[areas, centroids, boxes] = step(blobAnalysis, BW_img);

objList = [0,0,0];
XYZ_robot = [0,0,0];

% convert centroids to robot frame before adding to list
for i = 1:size(areas)
    XYZ_robot = camera2robot(cameraParams,centroids(i,:));
    XYZ_robot = XYZ_robot.';
    objList(i,1:2) = XYZ_robot(1,1:2);
    objList(i,3) = areas(i,1);
end

% objList = sortrows(objList,1,'ascend');
objList = sortrows(objList,1,'ascend');
centroids = objList(:,1:2);
areaList = objList(:,3);

%% DEBUGGING
% % Sort connected components in descending order by area
% [~, idx] = sort(areas, 'Descend');
% 
% % Gets numObj boxes
% boxes = double(boxes(idx(1:numObj), :));
% 
% % Insert labels for the coins.
% imDetectedDisks = insertObjectAnnotation(pic, 'rectangle', boxes, 'disk');
% figure; 
% imshow(imresize(imDetectedDisks,[480,640]));
% title('Detected Disks');
end