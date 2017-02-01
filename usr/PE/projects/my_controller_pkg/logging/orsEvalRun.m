clear

folder = 'out/';

q = load([folder,'q_bk.output']);
x = load([folder,'x_bk.output']);
goal = load([folder,'goal_bk.output']);
xRef = load([folder,'xRef.output']);
s = load([folder,'s_bk.output']);
tau_control = load([folder,'dtAmex.output']);
time = linspace(0,10,size(q,1));%load([folder,'ct_bk.output']);

nq = size(q,2);
nx = size(x,2);

% compute sum of squared accelerations
for i=1:nq
  qd(:,i) = gradient(q(:,i),tau_control);
  qdd(:,i) = gradient(gradient(q(:,i),tau_control),tau_control);
end

time = time-time(1);

% plot joint position
figure(1);clf;hold on;
for i=1:nq
  subplot(nq,1,i);hold on;
  plot(time,q(:,i));
end

% plot joint velocity
figure(2);clf;hold on;
for i=1:nq
  subplot(nq,1,i);hold on;
  plot(time,qd(:,i),'r');
end

% plot task
figure(3);clf;hold on;
for i=1:3
  subplot(nx,1,i);hold on;
  plot(time,x(:,i));
  plot(time,goal(:,i),'k');
  plot((0:size(xRef,1)-1)*0.01,xRef(:,i),'r');
end

% plot phase
figure(4);clf;hold on;
plot(time,s);

% 3d task phase
figure(4);clf;hold on;
plot3(x(:,1),x(:,2),x(:,3),'.');
plot3(xRef(:,1),xRef(:,2),xRef(:,3),'r.');
plot3(goal(:,1),goal(:,2),goal(:,3),'g.');
axis equal;

