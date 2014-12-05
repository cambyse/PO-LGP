clear
myDefs
path = '../13_PR2_DOOR/data/run7/';
filename = 'pr2';

load([path,filename,'.mat'])

% rescale time
t0 = joint_time(1);
joint_time = joint_time - t0;
if (usePressure)
 pressure_time = pressure_time - t0;
end
if (useMarker)
 markerDoor_time = markerDoor_time - t0;
 markerWall_time = markerWall_time - t0;
end
if (useBase)
 base_time = base_time - t0;
end

dt = 0.01;
t = 0:dt:(dt*(size(joint_pos,1)-1));

%% align data on same time axis
if (usePressure)
 pressureLeft = pressureLeft(:,4:5);
 pressureRight = pressureRight(:,4:5);
end
if (useMarker)
 marker_door_pos = markerDoorPos(1,:);
 marker_wall_pos = markerWallPos(1,:);
end
if (useBase)
 base_pos = basePos(1,:);
end
dMarker = 2;
wMarker = 2;
bMarker = 2;
for i=2:length(t)
 if (useMarker)
  if(markerDoor_time(dMarker)<t(i) && length(markerDoor_time)>dMarker+1)
   dMarker = dMarker + 1;
  end
  marker_door_pos = [marker_door_pos;markerDoorPos(dMarker,:)];
  
  if(markerWall_time(wMarker)<t(i) && length(markerWall_time)>wMarker+1)
   wMarker = wMarker + 1;
  end
  marker_wall_pos = [marker_wall_pos;markerWallPos(wMarker,:)];
 end
 
 if (useBase)
  if(base_time(bMarker)<t(i) && length(base_time)>bMarker+1)
   bMarker = bMarker + 1;
  end
  base_pos = [base_pos;basePos(bMarker,:)];
 end
end

%% plot joint position
% filter position
[b,a]=butter(4,0.01,'low')
joint_pos_filt = filtfilt(b,a,joint_pos);

f = figure(1);clf;
set(f,'Name','Joint position, velocities, efforts');
for i=1:length(jointList)
 subplot(length(jointList),1,i);
 plot(joint_pos(:,i)); hold on;
 plot(joint_pos_filt(:,i),'g'); hold on;
%  plot(gradient(joint_pos_filt(:,i),dt),'r'); hold on;
%  plot(gradient(gradient(joint_pos_filt(:,i),dt),dt),'k'); hold on;
end

if (useMarker)
 figure(2);clf;hold on;axis equal
 plot3(marker_door_pos(:,1),marker_door_pos(:,2),marker_door_pos(:,3),'.')
 plot3(marker_wall_pos(:,1),marker_wall_pos(:,2),marker_wall_pos(:,3),'g.')
 
 % filter marker position
%  [b,a]=butter(4,0.008,'low')
%  marker_wall_pos = filtfilt(b,a,marker_wall_pos);
%  marker_door_pos = filtfilt(b,a,marker_door_pos);
 
%  0.033;
 marker_wall_pos(:,1) = marker_wall_pos(:,1)+0.0235;
 marker_door_pos(:,1) = marker_door_pos(:,1)-0.0075;
 % marker_wall_pos(:,3) = 0;
 % marker_door_pos(:,3) = 0;
 
 % compute initial position of door in robot coordinates
 marker_wall_pos_0 = mean(marker_wall_pos(1:10,:));
 marker_door_pos_0 = mean(marker_door_pos(1:10,:));
 marker0 = [marker_wall_pos_0;marker_door_pos_0];
 
 marker_wall_quat_0 =  mean(markerWallQuat(1:10,:));
 marker_door_quat_0 =  mean(markerDoorQuat(1:10,:));
 markerQuat0 = [marker_wall_quat_0;marker_door_quat_0];
 
 figure(3);clf;hold on;
 for i=1:size(marker_door_pos,2) 
  subplot(size(marker_door_pos,2),1,i);
  plot(marker_door_pos(:,i)); hold on;
  plot(marker_wall_pos(:,i),'g'); hold on;
 end
 k = sqrt(sum((marker_door_pos-marker_wall_pos).^2,2));
 d = .755;
 h = 1.025;
 doorAngle = acos(min((d.^2+h.^2-k.^2)./(2*d.*h),1))
 
 figure(4);clf;
 plot(rad2deg(doorAngle))
end

% joints=[base_pos,joint_pos_filt(:,1),-doorAngle,joint_pos_filt(:,2:end)];
joints=[-doorAngle,joint_pos_filt(:,2:end)];

%% save date to files for ors
dlmwrite([path,filename,'_joints'],joints,' ')
% dlmwrite([path,filename,'_marker0'],marker0,' ')
% dlmwrite([path,filename,'_marker'],marker_pos,' ')
% dlmwrite([path,filename,'_markerQuat'],marker_quat,' ')
% dlmwrite([path,filename,'_contact'],contact_info,' ')


%% extract motion
% t_start = 1;
% t_end = length(joint_time);
t_start = 11428;
t_end = 12429;


subsample = 4;
jointsS = joints(t_start:subsample:t_end,:);
% joint_pos_filt = joint_pos_filt(t_start:subsample:t_end,:);
% marker_pos = marker_pos(t_start:subsample:t_end,:);
% marker_quat = marker_quat(t_start:subsample:t_end,:);
% contact_info = contact_info(t_start:subsample:t_end,:);
% contact_info = sum(contact_info,2)>13.5;
% [~,contact_idx] = max(contact_info);

tS = t(t_start:subsample:t_end); tS = tS-t(1);
dt = dt*subsample;
filename = [filename,'_3'];
dlmwrite([path,filename,'_joints'],jointsS,' ')
dlmwrite([path,filename,'_marker0'],marker0,' ')
dlmwrite([path,filename,'_markerQuat0'],markerQuat0,' ')
% dlmwrite([path,filename,'_marker'],marker_pos,' ')
% dlmwrite([path,filename,'_markerQuat'],marker_quat,' ')
% dlmwrite([path,filename,'_contact'],contact_info,' ')
% dlmwrite([path,filename,'_contact_idx'],contact_idx,' ')
dlmwrite([path,filename,'_t'],tS,' ')
dlmwrite([path,filename,'_dt'],dt,' ')

f = figure(5);clf;
set(f,'Name','Joint position, velocities, efforts');
for i=1:size(joints,2)
 subplot(size(joints,2),1,i);
 plot(jointsS(:,i)); hold on;
%  plot(t,joint_pos_filt(:,i),'g'); hold on;
%  plot(t,gradient(joint_pos_filt(:,i),dt),'r'); hold on;
%  plot(t,gradient(gradient(joint_pos_filt(:,i),dt),dt),'k'); hold on;
end

display('data preprocessing completed')