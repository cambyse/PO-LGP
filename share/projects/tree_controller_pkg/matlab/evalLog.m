clear
% copy log files from .ros/ to local folder
NEW = 1
[~, b] = unix('ls logs'); id = max(str2num(b))+NEW; folder = ['logs/',num2str(id)]; 
unix(['mkdir -p ', folder]); unix(['cp ~/.ros/*.output ', folder]);

% PLOT OPTIONS
pl = 2; pc =4;

d_effort_bk = load([folder,'/d_effort_bk.output']);
des_qd_bk   = load([folder,'/des_qd_bk.output']);
p_effort_bk = load([folder,'/p_effort_bk.output']);
qd_bk	      = load([folder,'/qd_bk.output']);
u_bk        = load([folder,'/u_bk.output']);
des_q_bk    = load([folder,'/des_q_bk.output']);
dt_bk       = load([folder,'/dt_bk.output']);
q_bk	      = load([folder,'/q_bk.output']);

nq = size(q_bk,2);
ni = size(q_bk,1) - length(find(dt_bk==0));

figure(1);clf;hold on;
for i=1:nq
  subplot (pl,pc,i);hold on;
  plot(q_bk(1:ni,i));
  plot(des_q_bk(1:ni,i),'r');
end
legend('position','desired_position');

figure(2);clf;hold on;
for i=1:nq
  subplot (pl,pc,i);hold on;
  plot(gradient(q_bk(1:ni,i),0.001),'k');
  plot(qd_bk(1:ni,i));
  plot(des_qd_bk(1:ni,i),'r');
end
legend('velocity','filtered velocity','desired_velocity');

figure(3);clf;hold on;
for i=1:nq
  subplot(pl,pc,i);hold on;
   plot(d_effort_bk(1:ni,i),'r');
  plot(u_bk(1:ni,i));
   plot(p_effort_bk(1:ni,i),'k');
  
end
legend('vel u','total u','pos u');


figure(4);clf;hold on;
plot(diff(dt_bk(1:ni)));

legend('dt','desired_position');
