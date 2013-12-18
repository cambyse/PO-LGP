function [acc_costs, goalPos_costs, goalVec_costs] = ...
  evaluate_motion(filename)

% compute acceleration costs (joint space)
q = load([filename,'/joints_bk.output']);
dt = load([filename,'/dt.output']);
goal = load([filename,'/goal.output']);

[n, dim] = size(q)

xdd = zeros(n,dim);


%   xdd(:,i) = gradient(gradient(q(:,i),dt),dt);
for j=2:n-1
  xdd(j,:) = (q(j+1,:)+q(j-1,:)-2*q(j,:))/(dt*dt);
end

figure(1);clf;hold on;
plot(q(:,1));
plot(xdd(:,1),'r')

acc_costs = sum(sum(xdd.^2,2));

% compute goal costs (task space)
x = load([filename,'/traj.output']);
goalPos_costs = sum((x(n,1:3)'-goal(1:3)).^2);
goalVec_costs = sum((x(n,4:6)'-goal(4:6)).^2);
end