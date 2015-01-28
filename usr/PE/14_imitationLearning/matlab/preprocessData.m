clear
myDefs
path = '../13_PR2_DOOR/data/run7/';
filename = 'pr2';

load([path,filename,'.mat'])

%% filter joint & marker pos
% filter joints
[b,a]=butter(4,0.01,'low')
joint_pos_filt = filtfilt(b,a,joint_pos);

f = figure(1);clf;
set(f,'Name','Joint position, velocities');
for i=1:length(jointList)
 subplot(length(jointList),1,i);
 plot(joint_pos(:,i)); hold on;
 plot(joint_pos_filt(:,i),'g'); hold on;
end

f = figure(2);clf;
set(f,'Name','Marker');
plot(marker_pos{1}(:,1));

% filter marker
% for k= 1:length(markerIds)
%  marker_pos_filt{k} = filtfilt(b,a,marker_pos{k});
% end
marker_pos_filt=marker_pos;

%% cut off joint position
cutOffStart = 1;
cutOffEnd = size(joint_pos_filt,1);
% run 4
% cutOffStart = 985;
% cutOffEnd = 2300;
% run 5
% cutOffStart = 565;
% cutOffEnd = 1555;
% run 6
% cutOffStart = 765;
% cutOffEnd = 1915;
% run 7
cutOffStart = 850;
cutOffEnd = 2001;
% run 8
% cutOffStart = 400;
% cutOffEnd = 1501;
% run 9
% cutOffStart = 550%1;
% cutOffEnd = 1520%size(joint_pos_filt,1);
% run 10
% cutOffStart = 750
% cutOffEnd = 2250
joint_pos_filt = joint_pos_filt(cutOffStart:cutOffEnd,:);
t = t(cutOffStart:cutOffEnd);
for k= 1:length(markerIds)
    marker_pos{k} = marker_pos{k}(cutOffStart:cutOffEnd,:);
    marker_pos_filt{k} = marker_pos_filt{k}(cutOffStart:cutOffEnd,:);
end
f = figure(2);clf;
set(f,'Name','Joint position, velocities (after cutoff)');
for i=1:length(jointList)
 subplot(length(jointList),1,i);
 plot(joint_pos_filt(:,i),'g'); hold on;
end




%% compute door angle
a = norm(.76,.007);
b = norm(.76+.263,.007+.016);
c = sqrt( sum( (marker_pos_filt{1} - marker_pos_filt{4}).^2, 2 ) );
door_joint = acos( (a.^2+b.^2-c.^2)./(2*a.*b) );

[b,a]=butter(1,0.05,'low')
door_joint_filt = filtfilt(b,a,door_joint);

door_joint_filt = max(0,door_joint_filt - mean(door_joint_filt(1:100)));


% subsampling
subsample = 5;
joint_pos_filt = joint_pos_filt(1:subsample:end,:);
t=t(1:subsample:end);
door_joint_filt = door_joint_filt(1:subsample:end,:);
door_joint = door_joint(1:subsample:end,:);
dt = dt*subsample;

f = figure(2);clf;
set(f,'Name','Joint position, velocities (after cutoff)');
for i=1:length(jointList)
 subplot(length(jointList),1,i);
 plot(joint_pos_filt(:,i),'g'); hold on;
end

myfig(3)
plot(door_joint)
plot(door_joint_filt,'g')

%% save data to files for ors
joints=[joint_pos_filt(:,1),-door_joint_filt,joint_pos_filt(:,2:end)];
marker0=[];
for k= 1:length(markerIds)
 marker0=[marker0;mean(marker_pos_filt{k}(1:200,:))];
end

dlmwrite([path,filename,'_joints'],joints,' ')
dlmwrite([path,filename,'_marker0'],marker0,' ')
dlmwrite([path,filename,'_t'],t,' ')
dlmwrite([path,filename,'_dt'],dt,' ')

display('data preprocessing completed');