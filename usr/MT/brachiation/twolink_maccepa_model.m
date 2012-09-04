function model = twolink_maccepa_model
% parameters from David's maccepa_arm_hw.m

% mass 
m1_l = 0.366;
m1_m = 0.054;      % damping motor mass 0.054
m1 = m1_l+m1_m;

m21 = 0.16;
m22 = 0.07;
m2 = m21+m22;

% length
l1 = 0.25;
l21 = 0.175;
l22 = 0.175; % length 2 slider
dl2 = 0.1; % link overlap (original 0.045) -> d2l = 0.1 such that l1=l2
l2 = l21+l22-dl2;


% center of mass location
lc1 = 0.135;
lc21 = 0.067;
lc22 = 0.095; % CG location measured from the end of length 2 slider
lc2 = (m21*lc21 + m22*(l21+lc22-dl2))/m2; % 0.0983478261

% moment of inertia (link)
Ic1_l = 2184448*(10^-9); % 2.184448e-03
Ic21_l = 283721*(10^-9); % 2.83721e-04
Ic22_l = 151717*(10^-9); % 1.51717e-04

% need to shift momemt of inertia using parallel axis theorem
h21 = lc2 - lc21;
h22 = l21+lc22-dl2 - lc2;
Ic21_cg = Ic21_l + m21*h21^2; % parallel axis theorem
Ic22_cg = Ic22_l + m22*h22^2;
Ic2_l  = (Ic21_cg+Ic22_cg); %  9.520502e-04

% moment of inter (damper motor and gear head)
n_ghd = [19;14];
I_mot = [41.8*(10^-7);41.8*(10^-7)];
I_ghd = [0.4*(10^-7);0.4*(10^-7)];

Im = (n_ghd.^2).*(I_mot + I_ghd);
Im1 = 0; % no first damper motor
Im2 = Im(2); % 0.0008271200

% definition of the robot parameters
m = [m1 m2]'; 
l = [l1 l2]'; 
lc = [lc1 lc2]';
Ic = [Ic1_l Ic2_l+Im2]'; % moment of inertia

D = [0.01 0;0 0.01]; % viscous damping coefficient

g = 9.80665;       % gravity
%g = 0;             % no gravity


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%link CG location without damping motor
lc1_l = (lc1*m1 - m1_m*l1)/m1_l; % 0.1180327869

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

model.m = m;
model.l = l;
model.lc= lc;
model.Ic = Ic;
model.D = D;
model.g = g;

% MACCEPA model parameters
model.spring_constant  = 771;   % K
model.lever_length     = 0.03;  % B
model.pin_displacement = 0.125; % C 
model.drum_radius      = 0.01;  % r


