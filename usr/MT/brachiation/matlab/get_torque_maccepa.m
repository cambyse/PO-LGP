% Calculate torque for MACCEPA model given state x and command u.
% (This is a wrapper for MaccepaModel.mex).
% 
%  in:
%      x     - state (pos,vel)
%      u     - motor commands 
%      model - model structure (for compatibility, not used)
%  out:
%      tau   - torque
%
%#eml

function tau = get_torque_maccepa(x,u,model)

%tau=MaccepaModel('maccepa_model_get_torque',x,u,model);

% actuator model parameters
k = model.spring_constant;
B = model.lever_length;
C = model.pin_displacement;
r = model.drum_radius;

tau = 0;

a = u(1)-x(1);

tau = k*B*C*sin(a)* ...
      ( 1 + (r*u(2)-(C-B) )/sqrt(B^2+C^2-2*B*C*cos(a)));

