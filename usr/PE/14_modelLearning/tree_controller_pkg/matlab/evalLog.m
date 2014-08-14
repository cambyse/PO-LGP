clear

% logs ab 29/

subplot = @(m,n,p) subtightplot (m, n, p, [0.02 0.05]);
figure = @(i) myfig(i);
% copy log files from .ros/ to local folder
NEW = 1
SIM = 0
[a, b] = unix('ls -l logs'); id = str2num(b(end-2:end-1))+NEW
% id = 57

folder = ['logs/',num2str(id)];
if NEW
  unix(['mkdir -p ', folder]);
  if SIM
    unix(['cp ~/.ros/*.output ', folder]);
  else
    unix(['sshpass -p  scp pr2admin@bigbirdc1:.ros/*.output ', folder]);
    %unix(['scp pr2admin@bigbirdc1:.ros/*.output ', folder]);
  end
end

% PLOT OPTIONS
pl = 2; pc =4;

d_effort_bk = load([folder,'/d_effort_bk.output']);
des_qd_bk   = load([folder,'/des_qd_bk.output']);
p_effort_bk = load([folder,'/p_effort_bk.output']);
gp_effort_bk = load([folder,'/gp_effort_bk.output']);
measured_effort_bk = load([folder,'/measured_effort_bk.output']);
qd_bk	      = load([folder,'/qd_bk.output']);
u_bk        = load([folder,'/u_bk.output']);
des_q_bk    = load([folder,'/des_q_bk.output']);
dt_bk       = load([folder,'/dt_bk.output']);
q_bk	      = load([folder,'/q_bk.output']);
qd_filt_bk	      = load([folder,'/qd_filt_bk.output']);
q_filt_bk	      = load([folder,'/q_filt_bk.output']);
ni = load([folder,'/storage_index.output']);
nq = size(q_bk,2);


t = (0:(ni-1))*0.001;

figure(1);clf;hold on;
for i=1:nq
  subplot (pl,pc,i);hold on;
   plot(t,q_bk(1:ni,i));
   %plot(t,q_filt_bk(1:ni,i),'k');
   plot(t,des_q_bk(1:ni,i),'r');
end 
legend('position','desired_position');

figure(2);clf;hold on;
for i=1:nq
  subplot (pl,pc,i);hold on;
  plot(t,gradient(q_bk(1:ni,i),0.001),'k');
  %plot(t,qd_filt_bk(1:ni,i));
  plot(t,des_qd_bk(1:ni,i),'r');
end
legend('velocity','desired_velocity');

figure(3);clf;hold on;
for i=1:nq
  subplot(pl,pc,i);hold on;
%  plot(t,measured_effort_bk(1:ni,i),'c');
  plot(t,p_effort_bk(1:ni,i),'r');
  plot(t,d_effort_bk(1:ni,i),'m');
  plot(t,gp_effort_bk(1:ni,i),'g','LineWidth',2);
  plot(t,u_bk(1:ni,i));

end
legend('pos u','vel u','gp u','total u');


% task space logs
unix(['cp ../../traj_executer_pkg/bin/traj_ref ',folder,'/'])
unix(['cp ../../traj_executer_pkg/bin/traj_act ',folder,'/'])

traj_ref = load([folder,'/traj_ref']);
traj_act = load([folder,'/traj_act']);
figure(4);clf;hold on;
plot3(traj_ref(:,1),traj_ref(:,2),traj_ref(:,3),'r-');
plot3(traj_act(:,1),traj_act(:,2),traj_act(:,3),'b-');
axis equal; axis tight;
view(90,0)
legend('reference','actual')
% myexportfig(4,'circle.pdf')

% effort statistics
total_abs_u = sum(abs(d_effort_bk))+sum(abs(p_effort_bk))+sum(abs(gp_effort_bk))
display('gp_effort [%]: ');
sum(abs(gp_effort_bk))*100./total_abs_u
display('p_effort [%]: ');
sum(abs(p_effort_bk))*100./total_abs_u
display('d_effort [%]: ');
sum(abs(d_effort_bk))*100./total_abs_u