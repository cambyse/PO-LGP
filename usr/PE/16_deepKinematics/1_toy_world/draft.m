clear;
A = imread('z.ppm');
B = imread('z.d.ppm');
pts = load('pts.dat');
cols = load('cols.dat');
depth_image = load('depth_image.dat');
% captureDepth = load('captureDepth.dat');
% B = double(B);
% B = (B-min(min(B)))/(max(max(B))-min(min(B)));
% C = double(C);
% C = (C-min(min(C)))/(max(max(C))-min(min(C)));
% figure(1);clf; imagesc(A); 
% figure(2);clf; imagesc(B); colorbar;
% figure(3);clf; plot3(pts(:,1),pts(:,2),pts(:,3),'.');
% figure(4);clf;imagesc(depth_image); colorbar;
% figure(5);clf;imagesc(captureDepth); colorbar;
figure(6);clf; pcshow(pts,cols); axis equal;
%% world coordinates
% set(gca,'Position',[0 0 .64 .48])
% set(gca,'CameraPosition',[0,-.5,.5])
% set(gca,'CameraTarget',[0 0 0])
% % camproj('perspective')
% set(gca,'CameraViewAngle',40)

%% camera coordinates
set(gca,'CameraPosition',[0,0,0])
set(gca,'CameraTarget',[0 1e-6 -1.])
camproj('perspective')
set(gca,'CameraViewAngle',40)