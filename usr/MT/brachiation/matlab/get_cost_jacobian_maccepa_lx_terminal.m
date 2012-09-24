function [l_x, derivs]  = get_cost_jacobian_maccepa_lx_terminal(x,u,t,QT,xt,model)
% note 
% x:  state (joint angles and joint velocities)
% xt: target pos and vel (in the cartesian coordinates)

%#eml
eml.extrinsic('pause');

dimX = size(x,1);

l1 = model.l(1);
l2 = model.l(2);

q1 = x(1);
q2 = x(2);
q1d = x(3);
q2d = x(4);

qvec = [q1 q2]';
qdvec = [q1d q2d]';

xt1 = xt(1);
xt2 = xt(2);
xt3 = xt(3);
xt4 = xt(4);

q11 = QT(1,1);
q12 = QT(1,2);
q13 = QT(1,3);
q14 = QT(1,4);

q21 = QT(2,1);
q22 = QT(2,2);
q23 = QT(2,3);
q24 = QT(2,4);

q31 = QT(3,1);
q32 = QT(3,2);
q33 = QT(3,3);
q34 = QT(3,4);

q41 = QT(4,1);
q42 = QT(4,2);
q43 = QT(4,3);
q44 = QT(4,4);

%l_x = zeros(4,1);
l_x = zeros(dimX,1);

s1 = sin(q1);
%c1 = cos(q1);
s12 = sin(q1+q2);
c12 = cos(q1+q2);

J = fn_jacobian_2link_arm(x(1:2,1), model);
J1 = J(1,1:2);
J2 = J(2,1:2);

dJ1dq1 = [-l2*sin(q2+q1)-l1*sin(q1),-l2*sin(q2+q1)];
dJ1dq2 = [-l2*sin(q2+q1),-l2*sin(q2+q1)];
dJ2dq1 = [l2*cos(q2+q1)+l1*cos(q1),l2*cos(q2+q1)];
dJ2dq2 = [l2*cos(q2+q1),l2*cos(q2+q1)];

P = zeros(4,4);
P(1:2,1:2) = J';
P(3:4,3:4) = J';
P(1,3) = dJ1dq1*qdvec;
P(1,4) = dJ2dq1*qdvec;
P(2,3) = dJ1dq2*qdvec;
P(2,4) = dJ2dq2*qdvec;

% x is state (q1,q2,qd1,qd2) and xt is given in cartesian coordinates
% thus, we need to compute cartesian position x_cart
x_cart = fn_forward_kinematics_2link_arm(x(1:2), model);
xd_cart = J*qdvec;

l_x(1:4,:) = 2*P*QT*([x_cart;xd_cart]-xt);

derivs.P = P;
derivs.dJ1dq1  = dJ1dq1;
derivs.dJ1dq2  = dJ1dq2;
derivs.dJ2dq1  = dJ2dq1;
derivs.dJ2dq2  = dJ2dq2;