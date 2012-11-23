model = twolink_maccepa_model
x0 = [-0.64 -1.85 0 0 0 0 0 0]';
xT = [ 0.64  1.85 0 0 0 0 0 0]';
u = zeros(2,1);
[A,a,B] = get_dynamics(x0, u, model)

load mytraj.mat;
tau = 9.9300e-03;
T=[0:tau:61*tau];
twolink_anim_maccepa(T, mytraj, model, xT);

load sample_dat
twolink_anim_maccepa(Tout,Qout,model,xt)  
%twolink_res_maccepa(Tout,Qout,Uout,model,dt,xt)


%% iterate the dynamics


model = twolink_maccepa_model;
load sample_dat;
T = size(Tout,1);
x = Qout(1,:)';
xT = Qout(T,:)';
X = x';
for t=1:T-1
	u = Uout(t,:)';
	[A,a,B] = get_dynamics(x, u, model);
	x = x + (A*x+a+B*u)*dt;
	X = [X; x'];
endfor
save -ascii sim.mat X
