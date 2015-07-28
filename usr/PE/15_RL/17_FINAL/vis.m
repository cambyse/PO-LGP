clear;
myDefs;
folder = ['data/door2/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

jointNames = {'worldTranslationRotation1',
 'worldTranslationRotation2',
 'worldTranslationRotation3',
 'torso lift joint',
 'head pan joint',
 'laser tilt mount joint',
 'r shoulder pan joint',
 'l shoulder pan joint',
 'head tilt joint',
 'r shoulder lift joint',
 'l shoulder lift joint',
 'r upper arm roll joint',
 'l upper arm roll joint',
 'r elbow flex joint',
 'l elbow flex joint',
 'r forearm roll joint',
 'l forearm roll joint',
 'r wrist flex joint',
 'l wrist flex joint',
 'r wrist roll joint',
 'l wrist roll joint',
 'r gripper r finger joint',
 'l gripper r finger joint',
 'r gripper joint',
 'l gripper joint'};

%% compute error between desired and actual trajectory
error = sum((Xdes-Xact).^2,1)
figure(6);clf;hold on;
bar(error);
set(gca,'XTickLabel',jointNames,'XTick',1:numel(jointNames));
rotateXLabels(gca(),90);

figure(8);clf;hold on;
plot(Tact,FLact(:,6))
sum(FLact(:,6))

figure(9);clf;hold on;
plot3(Mact(:,1),Mact(:,2),Mact(:,3),'.');
axis equal; grid on;

%% plotting
col={'r','b','k','m','c'};
for i =1:5:size(Xact,2)
 figure(1+round(i/5));clf;hold on;
 for j=0:4
  plot(Xact(:,i+j),col{j+1});
 end
 legend(jointNames{i:i+4})
 for j=0:4
  plot(Xdes(:,i+j),col{j+1},'LineStyle','--');
  plot(Xact(:,i+j),col{j+1});
 end
end
