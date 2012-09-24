% test for inverse kinematics for 2link arm
function x = kine1(q, robot)
  global l lc
  l = robot.l;
  lc = robot.lc;
  
  x=zeros(2,1);
  l1=l(1); l2=l(2);
    
  x(1) =  l1*sin(q(1));
  x(2) = -l1*cos(q(1));
