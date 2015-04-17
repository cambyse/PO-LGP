clear
addpath('/home/englerpr/Dropbox/research/code/matlab_rosbag-0.3-linux64/');
myDefs
filename = 'push2'
bag = ros.Bag(['../data/',filename,'.bag']);

jointList = {'r_shoulder_pan_joint',
 'r_shoulder_lift_joint',
 'r_upper_arm_roll_joint',
 'r_elbow_flex_joint',
 'r_forearm_roll_joint',
 'r_wrist_flex_joint',
 'r_wrist_roll_joint'
 };

jointMsgs = bag.readAll('/joint_states');
pressureMsgs = bag.readAll('/pressure/r_gripper_motor');
pressureInfoMsgs = bag.readAll('/pressure/r_gripper_motor_info');

markerMsgs = bag.readAll('/ar_pose_marker');

pressureInfo = pressureInfoMsgs{1}.sensor.force_per_unit;

for i=1:length(jointList)
 jointIdx(i) = strmatch(jointList(i),jointMsgs{1}.name);
end
for i = 1:length(jointMsgs)
 joint_time(i) = jointMsgs{i}.header.stamp.time;
 joint_pose(i,:) = jointMsgs{i}.position(jointIdx);
 joint_vel(i,:) = jointMsgs{i}.velocity(jointIdx);
 joint_effort(i,:) = jointMsgs{i}.effort(jointIdx);
end

count = 1;
for i = 1:length(markerMsgs)
 for j= 1:size(markerMsgs{i}.markers,2)
  if (markerMsgs{i}.markers(j).id == 3)
   marker_time(count) = markerMsgs{i}.markers(j).header.stamp.time;
   markerPose(count,:) = markerMsgs{i}.markers(j).pose.pose.position;
   markerQuat(count,:) = markerMsgs{i}.markers(j).pose.pose.orientation;
   count = count+1;
  end
 end
end

for i = 1:length(pressureMsgs)
 pressure_time(i) = pressureMsgs{i}.header.stamp.time;
 pressureLeft(i,:) = double(pressureMsgs{i}.l_finger_tip)./pressureInfo;
 pressureRight(i,:) = double(pressureMsgs{i}.r_finger_tip)./pressureInfo;
end


save(['../data/',filename,'.mat'])

