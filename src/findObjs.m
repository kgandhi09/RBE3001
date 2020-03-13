function [ROBOTFRAMEPOSE, COLORS, DISKSIZE] = findObjs(imOrig, cameraParams)
% FINDOBJS implements a sequence of image processing steps to detect
% any objects of interest that may be present in an RGB image.
%
% Note: this function contains several un-implemented sections - it only
% provides a skeleton that you can use as reference in the development of
% your image processing pipeline. Feel free to edit as needed (or, feel
% free to toss it away and implement your own function).
%
%   Usage
%   -----
%   [ROBOTFRAMEPOSE,COLORS,DISKSIZE] = findObjs(IMORIG, TCHECKER2ROBOT, TCAM2CHECKER, CAMERAPARAMS)
%
%   Inputs
%   ------
%   CAMERAPARAMS - an object containing the camera's intrinsic and
%   extrinsic parameters, as returned by MATLAB's camera calibration app.
%
%   Outputs
%   -------
%   ROBOTFRAMEPOSE - the coordinates of the objects expressed in the robot's
%   reference frame
%
%   COLORINFO - color of the objects found
%
%   DISKINFO - size of disks found
%
%   Authors
%   -------
%   Nathaniel Dennler  <nsdennler@wpi.edu>
%   Sean O'Neil        <stoneil@wpi.edu> 
%   Loris Fichera      <lfichera@wpi.edu>
%
%   Latest Revision
%   ---------------
%   2/12/2019


%%  1. First things first - undistort the image using the camera parameters
[im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

%%  2. Segment the image to find the objects of interest.
% Find the centroids of yellow/blue/green objects
[blueObj,BW_b] = findColor(im, "blue");
[yellowObj, BW_y] = findColor(im, "yellow");
[greenObj, BW_g] = findColor(im, "green");

colorMatrix = [];
objs = [];

% Assign Colors Numbers only if the object is found
% BLUE  = 1 
% GREEN  = 2
% YELLOW = 3
if ~isempty(blueObj) %if the color is there
    disp("blueObj")
    disp(blueObj)
    blueObj = camera2robot(cameraParams, blueObj); %convert camera to robot frame
    objs = [objs;blueObj.']; %append to the obj list
    colorMatrix = [colorMatrix;1]; %append to the color list
end
if ~isempty(greenObj)
    disp("greenObj")
    disp(greenObj)
    greenObj = camera2robot(cameraParams, greenObj);
    objs = [objs;greenObj.'];
    colorMatrix = [colorMatrix;2];
end
if ~isempty(yellowObj)
    disp("yellowObj")
    disp(yellowObj)
    yellowObj = camera2robot(cameraParams, yellowObj);
    objs = [objs;yellowObj.'];
    colorMatrix = [colorMatrix;3];
end

objsAndColor = [objs,colorMatrix]; %combine obj and color list
% disp(objsAndColor)

%%  3. Retrieve data from image.
% sort rows from least to greatest in terms of X coord
objList = sortrows(objsAndColor, 1, 'ascend');

centroidList = objList(:,1:2);

COLORS = objList(:,4);

% retrieves disk sizes
[centroidDisk, DISKSIZE] = diskSize(im, cameraParams);

% show the BW_img of the detected objects
a = imfuse(BW_b,BW_g,'blend','Scaling','joint');
a = xor(a,0);
b = imfuse(a,BW_y,'blend','Scaling','joint');
hold on
plot(objs(:,1), objs(:,2), 'b*');
hold off

% You can easily convert image pixel coordinates to 3D coordinates (expressed in the
% checkerboard reference frame) using the following transformations:
ROBOTFRAMEPOSE = centroidList;
end