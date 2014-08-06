clear
addpath('/home/peter/Dropbox/research/code/matlab_rosbag-0.3-linux64/');

% bag = ros.Bag('../catkin_ws/real_test.bag');
% bag = ros.Bag('../sim4/real_test.bag');
% bag = ros.Bag('subset.bag');
bag = ros.Bag('rr2.bag');

jointMsgs = bag.readAll('/r_arm_controller/state');
nJoints = size(jointMsgs{1}.joint_names,1);

for i = 1:length(jointMsgs)
  seq(i) = jointMsgs{i}.header.seq;
  joint_time(i) = jointMsgs{i}.header.stamp.time;
  pos(i,:) = jointMsgs{i}.actual.positions;
  vel(i,:) = jointMsgs{i}.actual.velocities;
  effort(i,:) = jointMsgs{i}.actual.effort
end

joint_time = joint_time - joint_time(1);
myfig(1);
hist(diff(joint_time),10);
title('joint time');
myfig(2);
plot(diff(seq))

% plot joint position,velocity,effort
f = myfig(3);
set(f,'Name','Joint position, velocities, efforts');
for i=1:nJoints
  subplot(nJoints,3,i*3-2);
  plot(joint_time,pos(:,i)); hold on;
 % title(jointList(i));
  subplot(nJoints,3,i*3-1);
  plot(joint_time,vel(:,i),'m'); hold on;
 % title(jointList(i));
  subplot(nJoints,3,i*3);
%   plot(joint_time,effort(:,i),'r'); hold on;
 % title(jointList(i));
end


% write to file
save('test.mat','pos','vel');

return



