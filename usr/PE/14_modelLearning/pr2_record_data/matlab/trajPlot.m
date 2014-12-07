clear
addpath('/home/peter/Dropbox/research/code/matlab_rosbag-0.3-linux64/');

bag = ros.Bag('../catkin_ws/traj_test.bag');

jointList = {'r_shoulder_pan_joint',
             'r_shoulder_lift_joint',
             'r_upper_arm_roll_joint',
             'r_forearm_roll_joint',
             'r_elbow_flex_joint',
             'r_wrist_flex_joint',
             'r_wrist_roll_joint'
  };

timeMsgs = cell2mat(bag.readAll('pfc_time'));
startTime = timeMsgs(1);
timeMsgs = timeMsgs-startTime;
dt = 0.04;

jointMsgs = bag.readAll('pfc_currState');

for i=1:length(jointList)
  jointIdx(i) = strmatch(jointList(i),jointMsgs{1}.name);
end

for i = 1:length(jointMsgs)
   joint_time(i) = jointMsgs{i}.header.stamp.time;
   pose(i,:) = jointMsgs{i}.position(jointIdx);
   vel(i,:) = jointMsgs{i}.velocity(jointIdx);
   effort(i,:) = jointMsgs{i}.effort(jointIdx);
end

joint_time = joint_time - joint_time(1);

%time = time - time(1);
%% plot joint position,velocity,effort
f = figure(1);clf;
set(f,'Name','Joint position, velocities, efforts');
for i=1:length(jointList)
  subplot(length(jointList),3,i*3-2);
  plot(joint_time,pose(:,i)); hold on;
 % title(jointList(i));
  subplot(length(jointList),3,i*3-1);
  plot(joint_time,vel(:,i)); hold on;
 % title(jointList(i));
  subplot(length(jointList),3,i*3);
  plot(joint_time,effort(:,i)); hold on;
 % title(jointList(i));
end

%% plot desired joint position,velocity
goalMsgs = bag.readAll('pfc_goalTraj');
% goalMsgs = goalMsgs{1}.trajectory.points;%

for i = 1:length(goalMsgs{1}.trajectory.points)
   t = getfield(goalMsgs{1}.trajectory.points,{i},'time_from_start');
   goal_time(i) = t.time;
   goal_pose(i,:) = getfield(goalMsgs{1}.trajectory.points,{i},'positions');
end

goal_time = goal_time;

for i=1:length(jointList)
  subplot(length(jointList),3,i*3-2);
  plot(goal_time,goal_pose(:,i),'r'); hold on;
%   subplot(length(jointList),3,i*3-1);
%   plot(goal_time,goal_vel(:,i),'r'); hold on;
%   subplot(length(jointList),3,i*3);
%   plot(goal_time,goal_acc(:,i),'r'); hold on;
end


%% analzye time
figure(2);clf;hold on;
plot(diff(goal_time),'.');
plot(diff(joint_time),'.r');
plot(diff(timeMsgs),'k.');

