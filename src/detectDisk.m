% takes in an image (pic) and outputs a BW image segmenting the disk
function BW = detectDisk(pic)
% need to draw red to get rid of black extrusion
bounds =[1 480; 1 123; 153 123; 81 443; 570 440; 493 124; 640 123; 640 480];
x = bounds(:,1);
y = bounds(:,2);
imshow(pic);
figure(1)
hold on
fill(transpose(x),transpose(y),'r')
rectangle('Position',[1 1 640 160],'FaceColor','r','EdgeColor','r')
rectangle('Position',[1 1 20 480],'FaceColor','r','EdgeColor','r')
rectangle('Position',[620 1 640 480],'FaceColor','r','EdgeColor','r')
rectangle('Position',[1 480 640 460],'FaceColor','r','EdgeColor','r')
cdata = print('-RGBImage');
hold off

[BW,maskedRGBImage] = diskMask(cdata);

BW = imresize(BW,[480,640]);

imshow(BW);
end