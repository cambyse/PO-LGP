% load sample data
>> load sample_dat

% plot results
>> twolink_res_maccepa(Tout,Qout,Uout,model,dt,xt)

% animation
>> twolink_anim_maccepa(Tout,Qout,model,xt)       

% variables
Tout: time (t)
Qout: state (x)
Uout: control (u)
model: robot model structure
dt: simulation time step
xt: target
