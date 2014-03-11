clear
% copy log files from .ros/ to local folder
NEW = 1
SIM = 1
[~, b] = unix('ls logs'); id = max(str2num(b))+NEW;
% id = 34
folder = ['logs/',num2str(id)];
if NEW
  unix(['mkdir -p ', folder]);
  if SIM
    unix(['cp ~/.ros/*.output ', folder]);
  else
    %unix(['sshpass -p *** scp pr2admin@bigbirdc1:.ros/*.output ', folder]);
    unix(['scp pr2admin@bigbirdc1:.ros/*.output ', folder]);
  end
end

% PLOT OPTIONS
pl = 2; pc =4;

d_effort_bk = load([folder,'/d_effort_bk.output']);
des_qd_bk   = load([folder,'/des_qd_bk.output']);
p_effort_bk = load([folder,'/p_effort_bk.output']);
i_effort_bk = load([folder,'/i_effort_bk.output']);
a_effort_bk = load([folder,'/a_effort_bk.output']);
measured_effort_bk = load([folder,'/measured_effort_bk.output']);
qd_bk	      = load([folder,'/qd_bk.output']);
u_bk        = load([folder,'/u_bk.output']);
des_q_bk    = load([folder,'/des_q_bk.output']);
dt_bk       = load([folder,'/dt_bk.output']);
q_bk	      = load([folder,'/q_bk.output']);
taskVec_y_bk = load([folder,'/taskVec_y_bk.output']);
taskPos_y_bk = load([folder,'/taskPos_y_bk.output']);
taskVec_yRef_bk = load([folder,'/taskVec_yRef_bk.output']);
taskPos_yRef_bk = load([folder,'/taskPos_yRef_bk.output']);
ni = load([folder,'/storage_index.output']);
nq = size(q_bk,2);
ny = size(taskVec_y_bk,2);


t = (0:(ni-1))*0.001;

figure(1);clf;hold on;
for i=1:nq
  subplot (pl,pc,i);hold on;
  plot(t,q_bk(1:ni,i));
  plot(t,des_q_bk(1:ni,i),'r');
end
legend('position','desired_position');

figure(2);clf;hold on;
for i=1:nq
  subplot (pl,pc,i);hold on;
  plot(t,gradient(q_bk(1:ni,i),0.001),'k');
  plot(t,qd_bk(1:ni,i));
  plot(t,des_qd_bk(1:ni,i),'r');
end
legend('velocity','filtered velocity','desired_velocity');

figure(3);clf;hold on;
for i=1:nq
  subplot(pl,pc,i);hold on;
  plot(t,d_effort_bk(1:ni,i),'r');
  plot(t,u_bk(1:ni,i));
  plot(t,measured_effort_bk(1:ni,i),'c');
  plot(t,p_effort_bk(1:ni,i),'k');
  plot(t,i_effort_bk(1:ni,i),'m');
  plot(t,a_effort_bk(1:ni,i),'g');
end
legend('vel u','total u','measured u','pos u', 'integral u','acc u');

figure(4);clf;hold on;
for i=1:ny
  subplot(2,3,i);hold on;
  plot(t,taskPos_y_bk(1:ni,i),'r');
  plot(t,taskPos_yRef_bk(1:ni,i));
end
for i=1:ny
  subplot(2,3,i+ny);hold on;
  plot(t,taskVec_y_bk(1:ni,i),'r');
  plot(t,taskVec_yRef_bk(1:ni,i));
end
legend('Task State','Des. Task State')

figure(5);clf;hold on; axis equal;grid on;
plot3(taskPos_y_bk(1:ni,1),taskPos_y_bk(1:ni,2),taskPos_y_bk(1:ni,3),'.');
plot3(taskPos_yRef_bk(1:ni,1),taskPos_yRef_bk(1:ni,2),taskPos_yRef_bk(1:ni,3),'.r');
for i=1:1000:ni
  p_start = taskPos_y_bk(i,:);
  p_end =  taskPos_y_bk(i,:) + 0.01*taskVec_y_bk(i,:);
  plot3([p_start(1),p_end(1)],[p_start(2),p_end(2)],[p_start(3),p_end(3)])
  p_start = taskPos_yRef_bk(i,:);
  p_end =  taskPos_yRef_bk(i,:) + 0.01*taskVec_yRef_bk(i,:);
  plot3([p_start(1),p_end(1)],[p_start(2),p_end(2)],[p_start(3),p_end(3)],'r')
end
legend('Task State','Des. Task State')

% figure(6);clf;hold on;
% plot(diff(dt_bk(1:ni)));
% legend('dt');
