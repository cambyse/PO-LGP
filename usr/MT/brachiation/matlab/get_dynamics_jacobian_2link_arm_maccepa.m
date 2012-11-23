% Function for calculating the Jacobian of 2link arm dynamics
% in analytical form
% in:
%    x, u, model 
%
% out:
%    xdot_x,xdot_u - Jacobian w.r.t. state and action.
%
function [xdot_x,xdot_u] = ...
    get_dynamics_jacobian_2link_arm_maccepa ( x, u, model )

%#eml

dimX = size(x,1);
dimU = size(u,1);
dimQ = 2;

m = model.m;   % mass
l = model.l;   % link length
lc = model.lc; % CG location on the link
Ic = model.Ic; % link inertia
D = model.D;   % viscous damping matrix
g = model.g;   % gravitational constant

xdot_x = zeros(dimX, dimX);
xdot_u = zeros(dimX, dimU);

q  = x(1:2);
qd = x(3:4);

tau = zeros(2,1);
%tau = get_torque_sea_aug(x, u);
xtmp = [x(2);x(4)]; % xtmp = [q2, q2dot]';
utmp = x(5:6,1);    % utmp = [qm1, qm2]';
tau(2) = get_torque_maccepa(xtmp, utmp, model);

% sine and cosine
s_q = sin(q);
c_q = cos(q);
s_q12 = sin(q(1)+q(2));
c_q12 = cos(q(1)+q(2));

% inertia matrix
%M = zeros(2,2);
%V = zeros(2,1);
%G = zeros(2,1);
%acc_tmp = zeros(2,1);

% reuse the torque controlled part
[acc_tmp, M, V, G] = get_acceleration_2link_arm_maccepa(x, u, model);

xdot_x(1:2, 1:4) = [0 0 1 0; 
                    0 0 0 1];

dMdx = zeros(2, 8);
dMdx(1,3) = -m(2)*2*l(1)*lc(2)*s_q(2);
dMdx(1,4) = -m(2)*l(1)*lc(2)*s_q(2);
dMdx(2,3) = dMdx(1,4);
dMdx(2,4) = 0;

dCGdx  = zeros(2,4);

dCGdx(1,1) =  m(1)*g*lc(1)*c_q(1)  ...
    + m(2)*g*(l(1)*c_q(1) + lc(2)*c_q12 );
dCGdx(1,2) = - lc(2)*m(2)*l(1)*c_q(2)*( 2*qd(1)*qd(2)+qd(2)^2 ) ...
    + g*lc(2)*m(2)*c_q12;
dCGdx(1,3) = - lc(2)*m(2)*l(1)*s_q(2)*2*qd(2) + D(1,1);
dCGdx(1,4) = - lc(2)*m(2)*l(1)*s_q(2)*(2*qd(1)+2*qd(2)) + D(1,2);

dCGdx(2,1) =  m(2)*g*lc(2)*c_q12;
dCGdx(2,2) =  lc(2)*m(2)*l(1)*c_q(2)*qd(1)^2 ... 
    + g*lc(2)*m(2)*c_q12;
dCGdx(2,3) = lc(2)*m(2)*l(1)*s_q(2)*2*qd(1) + D(2,1);
dCGdx(2,4) = D(2,2);

dCGdx = -dCGdx;

invM = inv(M);

In_invM = zeros(8,8);
In_invM(1:2,1:2) = invM;
In_invM(3:4,3:4) = invM;
In_invM(5:6,5:6) = invM;
In_invM(7:8,7:8) = invM;

CGu = zeros(2,1);
CGu = -V-G -D*qd + tau;

% note: V=C with a bit of notational abuse
% In_CGu = In (times) (-C-g+u) 
In_CGu = zeros(8,4);
In_CGu(1:2,1) = CGu;
In_CGu(3:4,2) = CGu;
In_CGu(5:6,3) = CGu;
In_CGu(7:8,4) = CGu;

xdot_x(3:4,1:4) = invM*(-dMdx*In_invM*In_CGu+dCGdx);

%tautmp  = zeros(dimQ,1);
%dtau_du = zeros(dimQ,dimU);
%[tautmp, dtau_dx, dtau_du] = get_torque_sea_aug(x,u);

dtau_dx = zeros(dimQ,dimX);
dtau_dx = get_torque_derivatives_maccepa_aug(x, model);

alpha = model.alpha;

xdot_x(3:4,:) = xdot_x(3:4,:) + invM*dtau_dx;
xdot_x(5:6,:) = [0 0 0 0 0 0 1 0;
                 0 0 0 0 0 0 0 1];
xdot_x(7,:) = [0 0 0 0 -alpha(1)^2  0       -2*alpha(1)  0];
xdot_x(8,:) = [0 0 0 0  0       -alpha(2)^2  0       -2*alpha(2)];

xdot_u(7,1) = alpha(1)^2;
xdot_u(8,2) = alpha(2)^2;

