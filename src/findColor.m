% takes in an image and a color and checks the workspace for an object of
% that color

% returns a centroid and a BW image of that color object segmented
function [centroid, BW3] = findColor(image, color)
%% Mask the Workspace 

bounds =[1 480; 1 123; 153 123; 81 443; 570 440; 493 124; 640 123; 640 480];
x = bounds(:,1);
y = bounds(:,2);
imshow(image);
figure(3)
hold on
fill(transpose(x),transpose(y),'r')
rectangle('Position',[1 1 640 160],'FaceColor','r','EdgeColor','r')
rectangle('Position',[1 1 20 480],'FaceColor','r','EdgeColor','r')
rectangle('Position',[620 1 640 480],'FaceColor','r','EdgeColor','r')
rectangle('Position',[1 480 640 460],'FaceColor','r','EdgeColor','r')
cdata = print('-RGBImage');
hold off

%% Apply Color Filters
if color == "blue"
    [BW,maskedRGBImage] = blueMask2(cdata);
    hue = 1;
elseif color == "yellow"
    [BW,maskedRGBImage] = yellowMask2(cdata);
    hue = 2;
elseif color == "green"
    [BW,maskedRGBImage] = greenMask2(cdata);
    hue = 3;
else
    disp('Color not Valid!!');
end

%% Perform image post-processing & Grab Centroids 
SE = strel('arbitrary',eye(7));
BW0 = edge(BW,'canny');
BW1 = imfill(BW0, 'holes');
BW2 = imerode(BW1, SE);
BW3 = imdilate(BW2,SE);
stats = regionprops(BW3, 'centroid');
centroid = cat(1,stats.Centroid);

BW3 = imresize(BW3,[480,640]);

blobAnalysis = vision.BlobAnalysis('AreaOutputPort', false,...
    'CentroidOutputPort', true,...
    'BoundingBoxOutputPort', false,...
    'MinimumBlobArea', 300, 'ExcludeBorderBlobs', true);
centroid = step(blobAnalysis, BW3);

end

