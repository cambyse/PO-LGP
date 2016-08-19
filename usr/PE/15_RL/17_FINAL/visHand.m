clear;
myDefs;
folder = ['data/door1/'];
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

%% plotting
col={'r','b','k','m','c'};


x = Xact(:,23);
y = Xact(:,25);

feat = [sin(x),cos(x),x,ones(size(Xact,1),1)]
% x = [x,ones(size(Xact,1),1)]
beta = inv(feat'*feat)*feat'*y

y_pred = feat*beta;

figure(1);clf;hold on;
plot(x,y,col{2});
plot(x,y_pred,col{2});

figure(2);clf;hold on;
plot(y./x);
plot(y_pred./x,'r.');
plot(y_pred2./x,'g.');
norm(y_pred - y)