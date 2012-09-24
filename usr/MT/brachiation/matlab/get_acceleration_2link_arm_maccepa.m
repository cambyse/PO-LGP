%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sub-function get_acceleration_2link_arm
% 
% in:   x             - state vector [position, velocity]'
%       u             - command vector 
%       model         - struct containing model parameters
% out:  acc           - acceleration
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [acc, M, V, G] = get_acceleration_2link_arm_maccepa(x, u, model)
%#eml

m = model.m;   % mass
l = model.l;   % link length
lc = model.lc; % CG location on the link
Ic = model.Ic; % link inertia
D = model.D;   % viscous damping matrix
g = model.g;   % gravitational constant

M = zeros(2,2);
V = zeros(2,1);
G = zeros(2,1);

q  = zeros(2,1);
qd = zeros(2,1);
acc = zeros(2,1);

q  = x(1:2);
qd = x(3:4);

% joint torque
tau = zeros(2,1);
%tau = get_torque_sea_aug(x, u);
xtmp = [x(2);x(4)]; % xtmp = [q2, q2dot]';
utmp = x(5:6,1);      % utmp = [qm1, qm2]';
tau(2) = get_torque_maccepa(xtmp, utmp, model);

s_q = sin(q);
c_q = cos(q);
s_q12 = sin(q(1)+q(2));

m11 = Ic(1)+Ic(2)+ ...
      + m(1)*lc(1)^2 ...
      + m(2)*(l(1)^2 + lc(2)^2 + 2*l(1)*lc(2)*c_q(2));

m12 = Ic(2)+m(2)*(lc(2)^2 + l(1)*lc(2)*c_q(2));
m21 = m12;
m22 = Ic(2)+lc(2)^2*m(2);

M = [m11 m12;m21 m22];

V(1) = - lc(2)*m(2)*l(1)*s_q(2)*( 2*qd(1)*qd(2)+qd(2)^2 );
V(2) =   lc(2)*m(2)*l(1)*s_q(2)*qd(1)^2;

G(1) = m(1)*g*lc(1)*s_q(1)  ...
      + m(2)*g*(l(1)*s_q(1) + lc(2)*s_q12 );
G(2) = g*lc(2)*m(2)*s_q12;

%detM = m11*m22-m12*m21;
invM =1/(m11*m22-m12*m21)*[m22 -m12;-m21 m11];

acc = invM*(-V-G-D*qd+tau);
%acc = inv(M)*(-V-G-D*qd+tau);
%acc = M\(-V-G-D*qd+tau);
