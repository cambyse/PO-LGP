%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function for computing the cost 
%
% 	J = x^T QT x + \int_0^T x^T Q x + u^T R u dt
% 
% NOTE: For use with ILQG, the function should be called with t=NaN to access
% the final cost (and derivatives).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%#eml

function l = get_cost_quadratic_reaching_maccepa ...
    (x,u,t,params,xt,model)
% xt: target pos and vel in Cartesian coordinates

dimX = size(x,1);
dimU = size(u,1);

QT = params.QT;
R1 = params.R1;
R2 = params.R2;

% compute cost
if isnan(t)
    q = zeros(2,1);
    qd = zeros(2,1);
    q = x(1:2);
    qd= x(3:4);
    Jac = zeros(2);
    
    xcart = zeros(4,1);
    xcart(1:2) = fn_forward_kinematics_2link_arm(q,model);
    Jac = fn_jacobian_2link_arm(q,model);
    xcart(3:4) = Jac*qd;
    
    l = (xcart-xt)'*QT*(xcart-xt);
else
    xtmp = [x(2);x(4)]; % xtmp = [q2, q2dot]';
    utmp = x(5:6,1);      % utmp = [qm1, qm2]';
    
    %tau = zeros(2,1);
    %tau(2) = get_torque_maccepa(xtmp, utmp, model);
    %l = u'*R1*u;

    F = 0;
    F = get_spring_force_maccepa(xtmp, utmp, model);
    
    l = u'*R1*u + R2*F^2;

end
