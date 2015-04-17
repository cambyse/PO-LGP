clear
addpath('/home/englerpr/Dropbox/research/code/matlab_rosbag-0.3-linux64/');

bag = ros.Bag('../data/run1.bag');

jointList = {'r_shoulder_pan_joint',
  'r_shoulder_lift_joint',
  'r_upper_arm_roll_joint',
  'r_elbow_flex_joint',
  'r_forearm_roll_joint',
  'r_wrist_flex_joint',
  'r_wrist_roll_joint'
  };

jointMsgs = bag.readAll('/joint_states');

for i=1:length(jointList)
  jointIdx(i) = strmatch(jointList(i),jointMsgs{1}.name);
end

dt = 0.01;

for i = 1:length(jointMsgs)
%   joint_time(i) = jointMsgs{i}.header.stamp.time;
  pos(i,:) = jointMsgs{i}.position(jointIdx);
  vel(i,:) = jointMsgs{i}.velocity(jointIdx);
  effort(i,:) = jointMsgs{i}.effort(jointIdx);
%   seq(i,:) = jointMsgs{i}.header.seq;
  i/length(jointMsgs)
end


save('run1.mat','pos','vel','effort');

return;

