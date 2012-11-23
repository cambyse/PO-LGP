function twolink_res_maccepa(Tout, Qout, Uout, model, dt, xt, ...
                             draw_maccepa, flipped_model)
close all;

if (nargin < 6 || isempty(xt))
    draw_target = 0;
else
    draw_target = 1;
end

if (nargin < 7 || isempty(draw_maccepa))
    draw_maccepa = 0;
end

if (nargin < 8 || isempty(flipped_model))
    flipped_model = 0;
end

n_steps = size(Uout,1);

%%%%%% joint angles, servo motor command, angles %%%%%
figure(1);
clf;
subplot(311)
hold on;
plot(Tout,Qout(:,1),'b-',Tout,Qout(:,2),'r-');
legend('th1','th2');
xlabel('time');
ylabel('joint angles');
grid on;

subplot(312);
hold on;
plot(Tout(1:n_steps),Uout(:,1),'b-');
plot(Tout,Qout(:,5),'r--');
legend('u_1','q_{m1}');
xlabel('time');
ylabel('u_1, q_{m1}');
grid on;

subplot(313)
hold on;
plot(Tout(1:n_steps),Uout(:,2),'b-');
plot(Tout,Qout(:,6),'r--');
legend('u_2','q_{m2}');
xlabel('time');
ylabel('u_2, q_{m2}');
grid on;

%%%%%% spring tension, joint torque %%%%%%
figure(2)
clf;

Tau = zeros(2,n_steps);
F = zeros(1,n_steps);
for i=1:n_steps
    Qout_tmp = [Qout(i,2);Qout(i,4)];
    Uout_tmp = Qout(i,5:6);
    Tau(:,i) = get_torque_maccepa(Qout_tmp, Uout_tmp,model);
    F(i) = get_spring_force_maccepa(Qout_tmp,Uout_tmp,model);
end

subplot(311)
hold on;
plot(Tout(1:n_steps),-F(1:n_steps),'b-');
legend('F');
xlabel('time');
ylabel('spring tension');
grid on;

Stiffness = zeros(1,n_steps);
for i=1:n_steps
    Qout_tmp = Qout(i,2);
    Uout_tmp = Qout(i,5:6);
    Stiffness(i) = get_stiffness_maccepa(Qout_tmp, Uout_tmp,model);
end

subplot(312)
hold on;
plot(Tout(1:n_steps),Tau(2,1:n_steps),'b-');
legend('tau');
xlabel('time');
ylabel('joint torque');
grid on;

subplot(313)
hold on;
plot(Tout(1:n_steps),Stiffness,'b-');
legend('k');
xlabel('time');
ylabel('stiffness');
grid on;

%%%%%%% stick diagram %%%%%%
% robot movemnet
dt_plot = 0.02;
dt_hand = 0.01;

axis_x = model.l(1)+model.l(2);
axis_y = axis_x/5;

ind  = 1:floor(dt_plot/dt):length(Tout);
ind2 = 1:floor(dt_hand/dt):length(Tout);

ind(end+1) = length(Tout); % index for T_final 

for i=1:length(Tout) 
  X2{1}(:,i) = kine1(Qout(i,1:2)', model);
  X2{2}(:,i) = kine(Qout(i,1:2)', model);
end

for i=1:length(Tout) 
  X2{1}(:,i) = kine1(Qout(i,1:2)', model);
  X2{2}(:,i) = kine(Qout(i,1:2)', model);
end

X20 = zeros(2,length(Tout));

figure(3);
hold on

% target
if (draw_target)
    plot(xt(1), xt(2), 'k+', 'MarkerSize', 12);
end

plot(X2{1}(1,ind),X2{1}(2,ind),'ko')
plot(X2{2}(1,ind2),X2{2}(2,ind2),'k.')

% draw maccepa
blue = [0 0 1];
gray  = [0.3 0.3 0.3];
gray2 = [0.7 0.7 0.7];

for i=1:length(ind)
    if (i~=1 &&  i~=length(ind))
        if(~flipped_model)
            plot_maccepa_schematic3 ...
                (Qout(ind(i),2),Qout(ind(i),5:6), [],model,...
                 [X2{1}(1,ind(i));X2{1}(2,ind(i))],...
                 Qout(ind(i),1)-pi/2, gray2);
        else
            plot_maccepa_schematic3 ...
                (-Qout(ind(i),2),[-Qout(ind(i),5);Qout(ind(i),6)], [],model,...
                 [X2{1}(1,ind(i));X2{1}(2,ind(i))],...
                 Qout(ind(i),1)+Qout(ind(i),2)-pi/2-pi, gray2);            
        end
    end
end

% start
i=1;
if(~flipped_model)
    plot_maccepa_schematic3 ...
        (Qout(ind(i),2),Qout(ind(i),5:6), [],model,...
         [X2{1}(1,ind(i));X2{1}(2,ind(i))],...
         Qout(ind(i),1)-pi/2, blue);        
else
    plot_maccepa_schematic3 ...
        (-Qout(ind(i),2),[-Qout(ind(i),5);Qout(ind(i),6)], [],model,...
         [X2{1}(1,ind(i));X2{1}(2,ind(i))],...
         Qout(ind(i),1)+Qout(ind(i),2)-pi/2-pi, blue);            
end

% end
i=length(ind);
if(~flipped_model)
    plot_maccepa_schematic3 ...
        (Qout(ind(i),2),Qout(ind(i),5:6), [],model,...
         [X2{1}(1,ind(i));X2{1}(2,ind(i))],...
         Qout(ind(i),1)-pi/2, blue);        
else
    plot_maccepa_schematic3 ...
        (-Qout(ind(i),2),[-Qout(ind(i),5);Qout(ind(i),6)], [],model,...
         [X2{1}(1,ind(i));X2{1}(2,ind(i))],...
         Qout(ind(i),1)+Qout(ind(i),2)-pi/2-pi, blue);            
end 

% robot
line([X20(1,ind);X2{1}(1,ind);X2{2}(1,ind)], ...
     [X20(2,ind);X2{1}(2,ind);X2{2}(2,ind)], 'Color', 'k')

axis equal;
axis([-axis_x axis_x -axis_x axis_y]);
grid on;
hold off

