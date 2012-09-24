% calculation of link position for animation

function x =  twolink_calcpos(t, th, robot)

l = robot.l;

% tip position of the link
x(:,1) = [l(1)*sin(th(1));  -l(1)*cos(th(1))];
x(:,2) = x(:,1) + [ l(2)*sin(th(1)+th(2)); -l(2)*cos(th(1)+th(2))] ;

