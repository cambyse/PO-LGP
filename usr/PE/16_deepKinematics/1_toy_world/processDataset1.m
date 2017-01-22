clear;
folder = 'dataset1/';

id = 0;
w=640;
h=480;

pts = load([folder,num2str(id),'_pts.dat']);
cols = load([folder,num2str(id),'_cols.dat']);
theta = load([folder,num2str(id),'_theta.dat']);
z = imread([folder,num2str(id),'_z.ppm']);
d = imread([folder,num2str(id),'_d.ppm']);

% subsample images
scale = [1 1]*.1;
oldSize = size(z);
newSize = max(floor(scale.*oldSize(1:2)),1); 

rowIndex = min(round(((1:newSize(1))-0.5)./scale(1)+0.5),oldSize(1));
colIndex = min(round(((1:newSize(2))-0.5)./scale(2)+0.5),oldSize(2));


%% plot image
figure(1);
subplot(2,3,1);
imagesc(z);axis tight;axis equal;

z2 = z(rowIndex,colIndex,:);
subplot(2,3,4);
imagesc(z2);axis tight; axis equal;

subplot(2,3,2);
imagesc(d);axis tight;axis equal;

d2 = d(rowIndex,colIndex,:);
subplot(2,3,5);
imagesc(d2);axis tight; axis equal;


% convert cols array to rgb image
cols_ic = reshape(cols,w,h,3);
cols_ic = permute(cols_ic,[2,1,3]);
cols_ic = uint8(cols_ic*255);
cols_ic = flipud(cols_ic);
subplot(2,3,3);
imagesc(cols_ic);axis tight;axis equal;
sum(sum(sum(abs(cols_ic-z))));

% convert pts array to image coordinates
pts_ic = reshape(pts,w,h,3);
pts_ic = permute(pts_ic,[2,1,3]);
pts_ic = flipud(pts_ic);
pts_ic2 = pts_ic(rowIndex,colIndex,:);

pts2 = reshape(pts_ic2,newSize(1)*newSize(2),3);



%%
figure(2);clf;hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3),5,cols,'filled');
scatter3(pts2(:,1),pts2(:,2),pts2(:,3),1);
axis equal;axis tight;
set(gca,'CameraPosition',[0,-.5,.5]);
set(gca,'CameraTarget',[0 0 0]);
camproj('perspective');
set(gca,'CameraViewAngle',40);

%%
figure(3);clf;hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3),5,cols);
axis equal;
scatter3(pts2(:,1),pts2(:,2),pts2(:,3));