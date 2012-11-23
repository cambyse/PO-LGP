% inverse kinematics for 2link arm
function q = fn_invkin_2link_arm(x, model, flag)

l = model.l;

q=zeros(2,1);

l1 = l(1); l2=l(2);

c_2 = (x(1)^2+x(2)^2-l1^2-l2^2)/(2*l1*l2);
s_2 =  sqrt(1-c_2^2); 

% when flag==1, choose s_2 = -sqrt(1-c_2^2)
if (nargin == 3 & flag == 1) 
    s_2 = -sqrt(1-c_2^2);
end

k1=l1+l2*c_2;
k2=l2*s_2;

c_1 = (k2*x(1)-k1*x(2))/(k1^2+k2^2);
s_1 = (k1*x(1)+k2*x(2))/(k1^2+k2^2);

q(1) = atan2(s_1, c_1);
q(2) = atan2(s_2, c_2);
