clear

folder = '300114/'
folder = ''

[time r_elbow_flex] = rtpload([folder,'r_elbow_flex.rtp']);
[time r_forearm_roll] = rtpload([folder,'r_forearm_roll.rtp']);
[time r_shoulder_lift] = rtpload([folder,'r_shoulder_lift.rtp']);
[time r_shoulder_pan] = rtpload([folder,'r_shoulder_pan.rtp']);
[time r_upper_arm_roll] = rtpload([folder,'r_upper_arm_roll.rtp']);
[time r_wrist_flex] = rtpload([folder,'r_wrist_flex.rtp']);
[time r_wrist_roll] = rtpload([folder,'r_wrist_roll.rtp']);



% reformat
all = {r_elbow_flex,r_forearm_roll,r_shoulder_lift,r_shoulder_pan,...
  r_upper_arm_roll,r_wrist_flex,r_wrist_roll};
names = {'relbowflex','rforearmroll','rshoulderlift','rshoulderpan',...
  'rupperarmroll','rwristflex','rwristroll'};


plotLimts = 0;
qLim = [-133 0 80 40 44 -130 0; 0 0 -30 -130 -224 0 0]/180*pi;
qdLim = [3.3 3.6 2.1 2.1 3.27 3.1 3.6;-3.3 -3.6 -2.1 -2.1 -3.27 -3.1 -3.6];
uLim = [30 30 30 30 30 10 10;-30 -30 -30 -30 -30 -10 -10];

% Init to Time 0
t0 = min(all{1}.dt);
for i=1:7
  all{i}.dt = all{i}.dt - t0;
end

figure(1);clf;hold on;
for i=1:7
  subplot (2,4,i);hold on;
  title(names{i});
  plot(all{i}.dt,all{i}.position)
  plot(all{i}.dt,all{i}.desired_position,'r')
  if (plotLimts)
    plot([min(all{i}.dt),max(all{i}.dt)],[qLim(1,i),qLim(1,i)]);
    plot([min(all{i}.dt),max(all{i}.dt)],[qLim(2,i),qLim(2,i)]);
  end
end
legend('position','desired_position');




% Plot velocity
figure(2);clf;hold on;
for i=1:7
  subplot (2,4,i);hold on;
  title(names{i});
  
  plot(all{i}.dt,all{i}.desired_velocity,'r')
  plot(all{i}.dt,all{i}.filter_vel,'g')
  plot(all{i}.dt,all{i}.velocity)
  if (plotLimts)
    plot([min(all{i}.dt),max(all{i}.dt)],[qdLim(1,i),qdLim(1,i)]);
    plot([min(all{i}.dt),max(all{i}.dt)],[qdLim(2,i),qdLim(2,i)]);
  end
end
legend('desired velocity','filtered velocity','velocity');


% Plot effort
figure(3);clf;hold on;
for i=1:7
  
  subplot (2,4,i);hold on;
  title(names{i});
  plot(all{i}.dt,all{i}.commanded_effort,'k')
  plot(all{i}.dt,all{i}.measured_effort,'g')
  
  plot(all{i}.dt,all{i}.p_effort,'LineWidth',2)
  plot(all{i}.dt,all{i}.d_effort,'m')
  plot(all{i}.dt,all{i}.i_effort,'r')
  
  if (plotLimts)
    plot([min(all{i}.dt),max(all{i}.dt)],[uLim(1,i),uLim(1,i)]);
    plot([min(all{i}.dt),max(all{i}.dt)],[uLim(2,i),uLim(2,i)]);
  end
end
legend('u measured', 'u commanded','u_p','u_d','u_i');
