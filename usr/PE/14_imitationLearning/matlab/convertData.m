clear
addpath('/home/englerpr/Dropbox/research/code/matlab_rosbag-0.3-linux64/');
myDefs
path = '../13_PR2_DOOR/data/run3/';
filename = 'pr2';
bag = ros.Bag([path,filename,'.bag']);

jointList = {
 'torso_lift_joint' 
 'l_shoulder_pan_joint',
 'l_shoulder_lift_joint',
 'l_upper_arm_roll_joint',
 'l_elbow_flex_joint',
 'l_forearm_roll_joint',
 'l_wrist_flex_joint',
 'l_wrist_roll_joint'
 };

tf = bag.readAll('/tf');

jointMsgs = bag.readAll('/joint_states');
markerMsgs = bag.readAll('/ar_pose_marker');

for i=1:length(jointList)
 jointIdx(i) = strmatch(jointList(i),jointMsgs{1}.name);
end
for i = 1:length(jointMsgs)
 joint_time(i) = jointMsgs{i}.header.stamp.time;
 joint_pos(i,:) = jointMsgs{i}.position(jointIdx);
 joint_vel(i,:) = jointMsgs{i}.velocity(jointIdx);
 joint_effort(i,:) = jointMsgs{i}.effort(jointIdx);
end

markerIds = [4,11,15,17];
countDoor = 1;
countWall = 1;
marker = cell(length(markerIds),1);
marker_time = cell(length(markerIds),1);
for i = 1:length(markerMsgs)
 for j= 1:size(markerMsgs{i}.markers,2)
  for k= 1:length(markerIds)
   if (markerMsgs{i}.markers(j).id == markerIds(k))
    marker{k} = [marker{k};markerMsgs{i}.markers(j).pose.pose.position'];
    marker_time{k} = [marker_time{k},markerMsgs{i}.markers(j).header.stamp.time];
   end
  end
 end
end

%% rescale time
t0 = joint_time(1);
joint_time = joint_time - t0;
for k= 1:length(markerIds)
    marker_time{k} = marker_time{k} - t0;
end

dt = 0.01;
t = 0:dt:(dt*(size(joint_pos,1)-1));

%% align data on same time axis
marker_pos = cell(length(markerIds),1);
for k= 1:length(markerIds)
 marker_pos{k} = marker{k}(1,:);
 dM = 2;
 for i=2:length(t)
  if(marker_time{k}(dM)<t(i) && length(marker_time{k})>dM+1)
   dM = dM + 1;
  end
  marker_pos{k} = [marker_pos{k};marker{k}(dM,:)];
 end
end

save([path,filename,'.mat'],'marker_pos','joint_pos','markerIds',...
 'jointList','t','dt')
display('data convertion completed')