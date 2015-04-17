clear
myDefs
filename = 'push2'

load(['../data/',filename,'.mat'])
% load('../data/location_acc_preprocessed.mat');


% rescale time
t0 = joint_time(1);
joint_time = joint_time - t0;
pressure_time = pressure_time - t0;
marker_time = marker_time - t0;

dt = 0.01;
t = 0:dt:(dt*(size(joint_pose,1)-1));

%% align data on same time axis 

pressureLeft = pressureLeft(:,4:5);
pressureRight = pressureRight(:,4:5);
contact_info = [pressureLeft(1,:),pressureRight(1,:)];
marker_pos = markerPose(1,:);
marker_quat = markerQuat(1,:);
cMarker = 2;
cPressure = 2;
for i=2:length(t)
 if(marker_time(cMarker)<t(i) && length(marker_time)>cMarker+1)
  cMarker = cMarker + 1;
 end
 marker_pos = [marker_pos;markerPose(cMarker,:)];
 marker_quat = [marker_quat;markerQuat(cMarker,:)];
 
 if(pressure_time(cPressure)<t(i) && length(pressure_time)>cPressure+1)
  cPressure = cPressure + 1;
 end
 contact_info = [contact_info;[pressureLeft(cPressure,:),pressureRight(cPressure,:)]];
end

%% plot joint position,velocity,effort

% filter position
[b,a]=butter(4,0.01,'low')
joint_pose_filt = filtfilt(b,a,joint_pose);

f = figure(1);clf;
set(f,'Name','Joint position, velocities, efforts');
for i=1:length(jointList)
 subplot(length(jointList),1,i);
 plot(t,joint_pose(:,i)); hold on;
 plot(t,joint_pose_filt(:,i),'g'); hold on;
end

figure(2);clf;hold on;axis equal
plot3(marker_pos(:,1),marker_pos(:,2),marker_pos(:,3),'.')

% filter velocity and effort
[b,a]=butter(4,0.005,'low')
contact_info_filt = filtfilt(b,a,contact_info);

figure(3);clf;hold on;
subplot(2,1,1);
plot(contact_info);hold on;
plot(contact_info_filt,'k')
subplot(2,1,2);
plot(max(contact_info_filt,[],2)>3.6)


%% save date to files for ors
dlmwrite(['../data/',filename,'_joints'],joint_pose,' ')
dlmwrite(['../data/',filename,'_marker'],marker_pos,' ')
dlmwrite(['../data/',filename,'_markerQuat'],marker_quat,' ')
dlmwrite(['../data/',filename,'_contact'],contact_info,' ')


%% extract motion
% t_start = 6200;
% t_end = 7900;
% m1
% t_start = 10200;
% t_end = 11200;
% m2
t_start = 8200;
t_end = 9100;


subsample = 4;
joint_pose_filt = joint_pose_filt(t_start:subsample:t_end,:);
joint_pose = joint_pose(t_start:subsample:t_end,:);
marker_pos = marker_pos(t_start:subsample:t_end,:);
marker_quat = marker_quat(t_start:subsample:t_end,:);
contact_info = contact_info(t_start:subsample:t_end,:);

contact_info = sum(contact_info,2)>13.5;
[~,contact_idx] = max(contact_info);

t = t(t_start:subsample:t_end); t = t-t(1);
dt = dt*subsample;
filename = [filename,'m2'];
dlmwrite(['../data/',filename,'_joints'],joint_pose_filt,' ')
dlmwrite(['../data/',filename,'_marker'],marker_pos,' ')
dlmwrite(['../data/',filename,'_markerQuat'],marker_quat,' ')
dlmwrite(['../data/',filename,'_contact'],contact_info,' ')
dlmwrite(['../data/',filename,'_contact_idx'],contact_idx,' ')
dlmwrite(['../data/',filename,'_t'],t,' ')
dlmwrite(['../data/',filename,'_dt'],dt,' ')

f = figure(1);clf;
set(f,'Name','Joint position, velocities, efforts');
for i=1:length(jointList)
 subplot(length(jointList),1,i);
 plot(t,joint_pose(:,i)); hold on;
 plot(t,joint_pose_filt(:,i),'g'); hold on;
 plot(t,gradient(joint_pose_filt(:,i),dt),'r'); hold on;
 plot(t,gradient(gradient(joint_pose_filt(:,i),dt),dt),'k'); hold on;
end