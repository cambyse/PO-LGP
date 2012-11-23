%#eml

function dtau_dx = get_torque_derivatives_maccepa_aug (x, model)
dimX = size(x,1); % 8 (augmented state)
dimQ = 2;

% actuator model parameters
kappa = model.spring_constant;
B = model.lever_length;
C = model.pin_displacement;
r = model.drum_radius;

k = 0;
alpha = 0;
beta  = 0;
gamma = 0;

alpha = x(5)-x(2);
beta  = r*x(6) - (C-B);
gamma = sqrt(B^2+C^2-2*B*C*cos(alpha));

xtmp = [x(2);x(4)];
utmp = x(5:6,1);
k = get_stiffness_maccepa(xtmp, utmp, model);

dtau_dx = zeros(dimQ,dimX);

%dtau_dx(2,1) = 0; % q1
dtau_dx(2,2) = -k; % q2
%dtau_dx(2,3) = 0; % q1dot
%dtau_dx(2,4) = 0; % q2dot
dtau_dx(2,5) = k;  % qm1
dtau_dx(2,6) = (kappa*r*sin(alpha)*B*C)/(gamma); %qm2
%dtau_dx(2,7) = 0; % qm1dot
%dtau_dx(2,8) = 0; % qm2dot


