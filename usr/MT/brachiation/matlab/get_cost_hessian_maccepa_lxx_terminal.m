function l_xx = get_cost_hessian_maccepa_lxx_terminal(x,u,t,QT,xt,model)
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

% sines and cosines
s1 = sin(q1);
c1 = cos(q1);
s12 = sin(q1+q2);
c12 = cos(q1+q2);

% Jacobian
J = fn_jacobian_2link_arm(x(1:2,1), model);
J1 = J(1,1:2);
J2 = J(2,1:2);

% derivatives of Jacobian
[l_x_tmp, derivs] = get_cost_jacobian_maccepa_lx_terminal(x,u,t,QT,xt, model);

P = derivs.P;
dJ1dq1 = derivs.dJ1dq1; 
dJ1dq2 = derivs.dJ1dq2; 
dJ2dq1 = derivs.dJ2dq1;
dJ2dq2 = derivs.dJ2dq2;

dJ11dq1 = dJ1dq1(1);
dJ12dq1 = dJ1dq1(2);

dJ11dq2 = dJ1dq2(1); 
dJ12dq2 = dJ1dq2(2); 

dJ21dq1 = dJ2dq1(1);
dJ22dq1 = dJ2dq1(2);

dJ21dq2 = dJ2dq2(1);
dJ22dq2 = dJ2dq2(2);

dJ1dq1_qdvec = P(1,3);
dJ2dq1_qdvec = P(1,4);
dJ1dq2_qdvec = P(2,3);
dJ2dq2_qdvec = P(2,4);

% compute second derivatives
% second derivatives of x_cart1, x_cart2
d2f1_dq1dq1 = dJ11dq1;
d2f1_dq1dq2 = dJ12dq1; % = dJ11dq2
d2f1_dq2dq2 = dJ12dq2;

d2f2_dq1dq1 = dJ21dq1;
d2f2_dq1dq2 = dJ22dq1; % dJ21dq2
d2f2_dq2dq2 = dJ22dq2;

% second derivatives of Jacobian J
d2J11_dq1dq1 = -l2*c12-l1*c1;
d2J11_dq1dq2 = -l2*c12;
d2J11_dq2dq2 = -l2*c12;

d2J12_dq1dq1 = -l2*c12;
d2J12_dq1dq2 = -l2*c12;
d2J12_dq2dq2 = -l2*c12;

d2J21_dq1dq1 = -l2*s12-l1*s1;
d2J21_dq1dq2 = -l2*s12;
d2J21_dq2dq2 = -l2*s12;

d2J22_dq1dq1 = -l2*s12;
d2J22_dq1dq2 = -l2*s12;
d2J22_dq2dq2 = -l2*s12;

% cartesian states
x_cart = fn_forward_kinematics_2link_arm(x(1:2), model);
xd_cart = J*qdvec;

x_errvect = [x_cart;xd_cart]-xt;

% sines and cosines
s1 = sin(q1);
c1 = cos(q1);
s12 = sin(q1+q2);
c12 = cos(q1+q2);

%l_xx = zeros(4,4);
l_xx = zeros(dimX, dimX);

lxx11 = ...
    2*P(1,:)*QT*P(1,:)' ...
    +2*[ ...
        d2f1_dq1dq1 ...
        d2f2_dq1dq1 ...
        d2J12_dq1dq1*q2d+d2J11_dq1dq1*q1d ...
        d2J22_dq1dq1*q2d+d2J21_dq1dq1*q1d ...
        ]*QT*x_errvect;

lxx12 = ...
    2*P(1,:)*QT*P(2,:)' ...
    +2*[ ...
        d2f1_dq1dq2 ...
        d2f2_dq1dq2 ...
        d2J12_dq1dq2*q2d+d2J11_dq1dq2*q1d ...
        d2J22_dq1dq2*q2d+d2J21_dq1dq2*q1d ...
       ]*QT*x_errvect;

lxx13 = ...
    2*P(1,:)*QT*P(3,:)' ...
    +2*x_errvect'*QT*[0; 0; dJ11dq1; dJ21dq1];

lxx14 = ...
    2*P(1,:)*QT*P(4,:)' ...
    +2*x_errvect'*QT*[0; 0; dJ12dq1; dJ22dq1];

lxx22 = ...
    2*P(2,:)*QT*P(2,:)' ...
    + 2*[d2f1_dq2dq2 d2f2_dq2dq2 ...
         (d2J12_dq2dq2*q2d+d2J11_dq2dq2*q1d) ...
         (d2J22_dq2dq2*q2d+d2J21_dq2dq2*q1d)] ...
    *QT*x_errvect;

lxx23 =  ...
    2*P(2,:)*QT*P(3,:)' ...
    + 2*x_errvect'*QT*[0;0;dJ11dq2;dJ22dq2];

lxx24 =  ...
    2*P(2,:)*QT*P(4,:)' ...
    + 2*x_errvect'*QT*[0;0;dJ12dq2;dJ22dq2];

lxx33 = 2*P(3,:)*QT*P(3,:)';
lxx34 = 2*P(3,:)*QT*P(4,:)';
lxx44 = 2*P(4,:)*QT*P(4,:)';

% Note: Hessian is symmetric
l_xx(1:4, 1:4) = [lxx11 lxx12 lxx13 lxx14; ...
                  lxx12 lxx22 lxx23 lxx24; ...
                  lxx13 lxx23 lxx33 lxx34; ...
                  lxx14 lxx24 lxx34 lxx44];