clear

addpath('~/catkin_ws/out/')


traj = load('traj.output')
trajRef = load('trajRef.output')
trajWrap = load('trajWrap.output')

%% 2D Plot
figure(11);clf;
for i=1:size(traj,2)
  subplot(2,3,i); hold on;
  plot(traj(:,i))
  plot(trajRef(:,i),'r')
  plot(trajWrap(:,i),'k')
end


%% 3D Plot
a = 0.03;
figure(12);clf;hold on;
plot3(traj(:,1),traj(:,2),traj(:,3),'.');
plot3(trajRef(:,1),trajRef(:,2),trajRef(:,3),'r.');
plot3(trajWrap(:,1),trajWrap(:,2),trajWrap(:,3),'k-');
for i=1:size(traj,1)
  plot3([traj(i,1);traj(i,1)+a*traj(i,4)],...
    [traj(i,2);traj(i,2)+a*traj(i,5)],...
    [traj(i,3);traj(i,3)+a*traj(i,6)]);
end
for i=1:size(trajRef,1)
  plot3([trajRef(i,1);trajRef(i,1)+a*trajRef(i,4)],...
    [trajRef(i,2);trajRef(i,2)+a*trajRef(i,5)],...
    [trajRef(i,3);trajRef(i,3)+a*trajRef(i,6)],'r');
end
for i=1:size(trajWrap,1)
  plot3([trajWrap(i,1);trajWrap(i,1)+a*trajWrap(i,4)],...
    [trajWrap(i,2);trajWrap(i,2)+a*trajWrap(i,5)],...
    [trajWrap(i,3);trajWrap(i,3)+a*trajWrap(i,6)],'k');
  
end
axis equal;grid on;