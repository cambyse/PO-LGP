clear
subplot = @(m,n,p) subtightplot (m, n, p, [0.02 0.05]);
joint_names = {'shoulder pan','shoulder lift','upper arm roll','elbow flex','forearm roll','wrist flex','wrist roll'};

load('../data/run2.mat');

dt = 0.01;
subN = size(pos,1);
pos = pos(1:subN,:);
vel = vel(1:subN,:);
effort = effort(1:subN,:);

[m,n] = size(pos);
time = dt:dt:m*0.01;

% filter velocity and effort
[b,a]=butter(4,0.1,'low')
pos_filtered = filtfilt(b,a,pos);
effort_filtered = filtfilt(b,a,effort);

for i=1:n
    vel_filtered(:,i) = gradient(pos_filtered(:,i),dt);
end

for i=1:n
    acc(:,i) = gradient(vel(:,i),dt);
    acc_filtered(:,i) = gradient(vel_filtered(:,i),dt);
end


% plot position
f = myfig(1);
set(f,'Name','Joint position');
for i=1:n
  subplot(n,1,i);
  plot(time,pos(:,i));
  plot(time,pos_filtered(:,i),'g');
  ylabel(joint_names{i});
end

% plot velocity
f = myfig(2); 
set(f,'Name','Joint velocity');
for i=1:n
  subplot(n,1,i);
  plot(time,vel(:,i));hold on;
  plot(time,vel_filtered(:,i),'g');
  ylabel(joint_names{i});
end

% plot effort
f = myfig(3); 
set(f,'Name','Joint effort');
for i=1:n
  subplot(n,1,i);
  plot(time,effort(:,i)); hold on;
  plot(time,effort_filtered(:,i),'g');
  ylabel(joint_names{i});
end

% plot acceleration
f = myfig(4); 
set(f,'Name','Joint acceleration');
for i=1:n
  subplot(n,1,i);
  plot(time,acc(:,i)); hold on;
  plot(time,acc_filtered(:,i),'g');
  ylabel(joint_names{i});
end
  
% save unfiltered data
save('../data/run2_unfiltered','pos','vel','acc','effort');

% save filtered data
pos = pos_filtered;
vel = vel_filtered;
effort = effort_filtered;
acc = acc_filtered;
save('../data/run2_filtered','pos','vel','acc','effort');