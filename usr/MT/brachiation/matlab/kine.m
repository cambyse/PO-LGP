% test for inverse kinematics for 2link arm
function x = kine(q, robot)

l = robot.l;

x=zeros(2,1);
l1=l(1); l2=l(2);

x(1) = l1*sin(q(1))+l2*sin(q(1)+q(2));
x(2) = -l1*cos(q(1))-l2*cos(q(1)+q(2));
