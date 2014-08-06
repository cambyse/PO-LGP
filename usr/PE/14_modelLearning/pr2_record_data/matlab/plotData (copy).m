clear
%subplot = @(m,n,p) subtightplot (m, n, p, [0.02 0.05]);
%figure = @(i) myfig(i);
joint_names = {'shoulder pan','shoulder lift','upper arm roll','elbow flex','forearm roll','wrist flex','wrist roll'};

data = load('../data/data_filtered.mat');
dataU = load('../data/data_unfiltered.mat');

dt = 0.01;
[m,n] = size(data.pos);
time = dt:dt:m*0.01;

% plot data.position
f = figure(1);
set(f,'Name','Joint data.position');
for i=1:n
  subplot(n,1,i);
  plot(time,dataU.pos(:,i));hold on;
  plot(time,data.pos(:,i),'g');
  ylabel(joint_names{i});
end

% plot data.velocity
f = figure(2); 
set(f,'Name','Joint data.velocity');
for i=1:n
  subplot(n,1,i);
  plot(time,dataU.vel(:,i));hold on;
  plot(time,data.vel(:,i),'g');
  ylabel(joint_names{i});
end

% plot data.effort
f = figure(3);
set(f,'Name','Joint data.effort');
for i=1:n
  subplot(n,1,i);
  plot(time,dataU.effort(:,i)); hold on;
  plot(time,data.effort(:,i),'g'); 
  ylabel(joint_names{i});
end
  
% plot data.acc
f = figure(4);
set(f,'Name','Joint data.acc');
for i=1:n
  subplot(n,1,i);
  plot(time,dataU.acc(:,i)); hold on;
  plot(time,data.acc(:,i),'g');
  ylabel(joint_names{i});
end
