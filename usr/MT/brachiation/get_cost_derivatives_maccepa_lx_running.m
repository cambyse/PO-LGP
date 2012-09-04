function [l_x,l_xx] = ...
    get_cost_derivatives_maccepa_lx_running(x, R2, model)

%#eml
dimX = size(x,1);
%dimU = size(u,1);

k = model.spring_constant;
B = model.lever_length;
C = model.pin_displacement;
r = model.drum_radius;

xtmp = [x(2);x(4)]; % xtmp = [q2, q2dot]';
utmp = x(5:6,1);    % utmp = [qm1, qm2]';

F = 0;
F = get_spring_force_maccepa(xtmp, utmp, model);

alpha = x(5)-x(2);
beta  = r*x(6)-(C-B);
gamma = sqrt(B^2+C^2-2*B*C*cos(alpha));

dF_dx = zeros(1,dimX);

dF_dx(2) =  k*sin(alpha)*B*C/gamma;    % dF_dq
dF_dx(5) = -k*sin(alpha)*B*C/gamma;    % dF_du1 = -dF_dq
dF_dx(6) = -k*r;                       % dF_du2

l_x = 2*R2*F*dF_dx';

P = dF_dx';

dPdx1 = zeros(8,1);

dPdx2 = zeros(8,1);
dPdx2(2) =  k*cos(alpha)*B*C/gamma ...
            - k*(sin(alpha)*B*C)^2/(gamma^3);
dPdx2(5) = -dPdx2(2);

dPdx3 = zeros(8,1);
dPdx4 = zeros(8,1);

dPdx5 = zeros(8,1);
dPdx5(2) = dPdx2(5);
dPdx5(5) = dPdx2(2);

dPdx6 = zeros(8,1);
dPdx7 = zeros(8,1);
dPdx8 = zeros(8,1);

dPdx = [dPdx1 dPdx2 dPdx3 dPdx4 dPdx5 dPdx6 dPdx7 dPdx8];

l_xx = 2*R2*(F*dPdx + P*P');
