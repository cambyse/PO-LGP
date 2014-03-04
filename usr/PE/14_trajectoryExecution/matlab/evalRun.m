% function [eval_ssa, eval_gd, eval_t, xRef, x] = evalRun(evalName,sceneName,methodName)
function [evalR] = evalRun(evalName,sceneName,methodName)
baseName = [evalName,'/',sceneName,'_',methodName];

q = load([baseName,'_q_bk.output']);
x = load([baseName,'_x_bk.output']);
goal = load([baseName,'_goal_bk.output']);
xRef = load([baseName,'_xRef.output']);
tau_control = load([baseName,'_tau_control.output']);
tau_plan = load([baseName,'_tau_plan.output']);
ct = load([baseName,'_ct_bk.output']);

nq = size(q,2);

% compute sum of squared accelerations
for i=1:nq
  qdd(:,i) = gradient(gradient(q(:,i),tau_control),tau_control);
end
evalR.eval_ssa = sum(sum(qdd.^2,2));

% conpute goal distance at final position
evalR.eval_gd = norm(x(end,1:3) - goal(end,:));

% total time of movement
evalR.eval_t = size(q,1)*tau_control;
evalR.x = x;
evalR.xRef = xRef;
evalR.goal = goal;
evalR.ct = mean(ct);
evalR.sr = norm(x(end,1:3)-goal(end,1:3))<0.05;

end