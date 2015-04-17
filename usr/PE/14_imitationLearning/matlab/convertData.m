clear
addpath('/home/englerpr/Dropbox/research/code/matlab_rosbag-0.3-linux64/');
myDefs
path = '../13_PR2_DOOR/data/run7/';
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

usePressure = false;
useMarker = true;
useBase = true;

tf = bag.readAll('/tf');

jointMsgs = bag.readAll('/joint_states');
if (usePressure)
 pressureMsgs = bag.readAll('/pressure/r_gripper_motor');
 pressureInfoMsgs = bag.readAll('/pressure/r_gripper_motor_info')
end
if (useMarker)
 markerMsgs = bag.readAll('/ar_pose_marker');
end

for i=1:length(jointList)
 jointIdx(i) = strmatch(jointList(i),jointMsgs{1}.name);
end
for i = 1:length(jointMsgs)
 joint_time(i) = jointMsgs{i}.header.stamp.time;
 joint_pos(i,:) = jointMsgs{i}.position(jointIdx);
 joint_vel(i,:) = jointMsgs{i}.velocity(jointIdx);
 joint_effort(i,:) = jointMsgs{i}.effort(jointIdx);
end

if (useBase)
 cBase=1;
 for i = 1:length(tf)
  for j= 1:size(tf{i}.transforms,2)
   if (isequal(tf{i}.transforms(j).header.frame_id,'/odom_combined'))
    base_time(cBase) = tf{i}.transforms(j).header.stamp.time;
    basePos(cBase,:) = tf{i}.transforms(j).transform.translation;
    [a,b,c]=quat2angle(tf{i}.transforms(j).transform.rotation');
    [~,~,baseAngle(cBase,:)] = quat2angle(tf{i}.transforms(j).transform.rotation');
    %     markerQuat(count,:) = markerMsgs{i}.markers(j).pose.pose.orientation;
    cBase = cBase+1;
   end
  end
 end
 basePos = [basePos(:,1:2), baseAngle];
 % reset base initial pos to 0
 basePos = basePos - repmat(basePos(1,:),length(base_time),1);
%  baseAngle = baseAngle - repmat(baseAngle(1,:),length(base_time),1);
 figure(1);clf;hold on;
 plot(basePos(:,1))
 plot(basePos(:,2),'g')
 plot(basePos(:,3),'b')
end

if (useMarker)
 countDoor = 1;
 countWall = 1;
 for i = 1:length(markerMsgs)
  for j= 1:size(markerMsgs{i}.markers,2)
   if (markerMsgs{i}.markers(j).id == 4)
    markerDoor_time(countDoor) = markerMsgs{i}.markers(j).header.stamp.time;
    markerDoorPos(countDoor,:) = markerMsgs{i}.markers(j).pose.pose.position;
    markerDoorQuat(countDoor,:) = markerMsgs{i}.markers(j).pose.pose.orientation;
    countDoor = countDoor+1;
   end
   if (markerMsgs{i}.markers(j).id == 17)
    markerWall_time(countWall) = markerMsgs{i}.markers(j).header.stamp.time;
    markerWallPos(countWall,:) = markerMsgs{i}.markers(j).pose.pose.position;
    markerWallQuat(countWall,:) = markerMsgs{i}.markers(j).pose.pose.orientation;
    countWall = countWall+1;
   end
  end
 end
end

if (usePressure)
 for i = 1:length(pressureMsgs)
  pressure_time(i) = pressureMsgs{i}.header.stamp.time;
  pressureLeft(i,:) = double(pressureMsgs{i}.l_finger_tip)./pressureInfo;
  pressureRight(i,:) = double(pressureMsgs{i}.r_finger_tip)./pressureInfo;
 end
end

save([path,filename,'.mat'])
display('data convertion completed')