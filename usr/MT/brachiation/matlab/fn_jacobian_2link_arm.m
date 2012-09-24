% jacobian for 2link arm
function J = fn_jacobian_2link_arm(q, model)
%#eml

l = model.l;
%lc = model.lc;

l1=l(1); l2=l(2);

J = zeros(2,2);

J(1,1) = l1*cos(q(1)) + l2*cos(q(1)+q(2));
J(1,2) = l2*cos(q(1)+q(2));

J(2,1) = l1*sin(q(1)) + l2*sin(q(1)+q(2));
J(2,2) = l2*sin(q(1)+q(2));
