% forward kinematics for 2link arm
function x = fn_forward_kinematics_2link_arm(q, model)
%#eml

l = model.l;

x=zeros(2,1);
l1=l(1); l2=l(2);

x(1) = l1*sin(q(1))+l2*sin(q(1)+q(2));
x(2) = -l1*cos(q(1))-l2*cos(q(1)+q(2));
