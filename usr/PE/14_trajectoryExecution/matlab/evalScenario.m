clear

% evalName = 'cupboard';
% docName = 'Task 1';

% evalName = 'ball';
% docName = 'Task 2';

% evalName = 'obstacle';
% docName = 'Task 3';

evalName = 'mobstacle';
docName = 'Task 4';

numScenes = load([evalName,'/numScenes.output']);
for iScenes=1:numScenes
  iScenes
  evalResult{iScenes} = evalScene(evalName,['scene',num2str(iScenes)]);
end
save([evalName,'.mat']);

% load([evalName,'.mat']);

% make some statistics
res_t=[];
res_ssa=[];
res_gd=[];
res_ct=[];
res_sr=[];
for iScenes=1:numScenes
  res_t = [res_t; evalResult{iScenes}.DMP.eval_t, evalResult{iScenes}.PFC.eval_t, evalResult{iScenes}.MPC.eval_t];
  res_ssa = [res_ssa; evalResult{iScenes}.DMP.eval_ssa, evalResult{iScenes}.PFC.eval_ssa, evalResult{iScenes}.MPC.eval_ssa];
  res_gd = [res_gd; evalResult{iScenes}.DMP.eval_gd, evalResult{iScenes}.PFC.eval_gd, evalResult{iScenes}.MPC.eval_gd];
  res_ct = [res_ct; evalResult{iScenes}.DMP.ct, evalResult{iScenes}.PFC.ct, evalResult{iScenes}.MPC.ct];
  res_sr = [res_sr; evalResult{iScenes}.DMP.sr, evalResult{iScenes}.PFC.sr, evalResult{iScenes}.MPC.sr];
end
% bar plot of final time
figure(1);clf;hold on;
subplot(4,1,1)
bar(res_t);
legend('DMP','PFC','MPC');
title('Final Time');

% bar plot of sum of accelerations
subplot(4,1,2)
bar(res_ssa);
legend('DMP','PFC','MPC');
title('Sum of Squared Accelerations');

% bar plot of goal distance
subplot(4,1,3)
bar(res_gd);
legend('DMP','PFC','MPC');
title('final distance to goal [m]');

% bar plot of computation time
subplot(4,1,4)
bar(res_ct);
legend('DMP','PFC','MPC');
title('computational time [s]');


% bar plot with mean and variance
res_t_mean = mean(res_t);
res_t_std = std(res_t);
figure(4);clf;hold on;
subplot(4,1,1); hold on;
bar(1,res_t_mean(1),'b');
bar(2,res_t_mean(2),'g');
bar(3,res_t_mean(3),'r');
errorbar(res_t_mean,2*(res_t_std),'k.');
legend('DMP','PFC','MPC'); axis tight;
title('Final Time');

res_ssa_mean = mean(res_ssa);
res_ssa_std = std(res_ssa);
subplot(4,1,2); hold on;
bar(1,res_ssa_mean(1),'b');
bar(2,res_ssa_mean(2),'g');
bar(3,res_ssa_mean(3),'r');
errorbar(res_ssa_mean,2*(res_ssa_std),'k.');
legend('DMP','PFC','MPC'); axis tight;
title('Sum of Squared Accelerations');

res_gd_mean = mean(res_gd);
res_gd_std = std(res_gd);
subplot(4,1,3); hold on;
bar(1,res_gd_mean(1),'b');
bar(2,res_gd_mean(2),'g');
bar(3,res_gd_mean(3),'r');
errorbar(res_gd_mean,2*(res_gd_std),'k.');
legend('DMP','PFC','MPC'); axis tight;
title('final distance to goal');

res_ct_mean = mean(res_ct);
res_ct_std = std(res_ct);
subplot(4,1,4); hold on;
bar(1,res_ct_mean(1),'b');
bar(2,res_ct_mean(2),'g');
bar(3,res_ct_mean(3),'r');
errorbar(res_ct_mean,2*(res_ct_std),'k.');
legend('DMP','PFC','MPC'); axis tight;
title('computation time');


%% 3d plot of trajectory comparison
f = figure(10);clf;hold on;
set(gcf,'Color',[1 1 1]);
for iScenes=1:4:numScenes
%   plot3(evalResult{iScenes}.DMP.x(:,1),evalResult{iScenes}.DMP.x(:,2),evalResult{iScenes}.DMP.x(:,3),'b.');
  plot3(evalResult{iScenes}.PFC.x(:,1),evalResult{iScenes}.PFC.x(:,2),evalResult{iScenes}.PFC.x(:,3),'b.','LineWidth',8);
  plot3(evalResult{iScenes}.PFC.goal(:,1),evalResult{iScenes}.PFC.goal(:,2),evalResult{iScenes}.PFC.goal(:,3),'m.','MarkerSize',20);
end
plot3(evalResult{iScenes}.PFC.xRef(:,1),evalResult{iScenes}.PFC.xRef(:,2),evalResult{iScenes}.PFC.xRef(:,3),'r.','LineWidth',15);

axis equal;axis tight;axis on;grid on;box on;
camlight; lighting gouraud;
view(-167,8)

%% 3d plot of trajectory comparison
for iScenes=1:1
  figure(10+iScenes);clf;hold on;grid on;
  plot3(evalResult{iScenes}.DMP.x(:,1),evalResult{iScenes}.DMP.x(:,2),evalResult{iScenes}.DMP.x(:,3),'b.');
  plot3(evalResult{iScenes}.PFC.x(:,1),evalResult{iScenes}.PFC.x(:,2),evalResult{iScenes}.PFC.x(:,3),'g.');
  plot3(evalResult{iScenes}.MPC.x(:,1),evalResult{iScenes}.MPC.x(:,2),evalResult{iScenes}.MPC.x(:,3),'r.');
  plot3(evalResult{iScenes}.PFC.xRef(:,1),evalResult{iScenes}.PFC.xRef(:,2),evalResult{iScenes}.PFC.xRef(:,3),'k.');
  plot3(evalResult{iScenes}.PFC.goal(:,1),evalResult{iScenes}.PFC.goal(:,2),evalResult{iScenes}.PFC.goal(:,3),'mx');
  legend('DMP','PFC','MPC','Reference Trajectory');
  axis equal;
  axis tight;
end

%% Write stuff to latex files
createLatexTable(docName,{'Success Rate','Final Time','Sum of Squared Accelerations','Computational Time'},...
  [sum(res_sr)/size(res_sr,1);res_t_mean;res_ssa_mean;res_ct_mean],...
  [nan(1,3);2*res_t_std;2*res_ssa_std;2*res_ct_std]);
sum(res_sr)/size(res_sr,1)
system('rm latex/simTask');
system('touch latex/simTask');
system('cat latex/header >> latex/simTask');
system('cat "latex/Task 1" >> latex/simTask');
system('cat "latex/Task 2" >> latex/simTask');
system('cat "latex/Task 3" >> latex/simTask');
system('cat "latex/Task 4" >> latex/simTask');
system('cat latex/ender >> latex/simTask');
system('cp latex/simTask ~/Dropbox/research/conferences/2013_IROS/results/simTask');