%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sub-function get_xdot_2link_arm
% in:   x             - state vector [position, velocity]'
%       u             - command vector 
%       model         - struct containing model parameters
% out:  xdot          - state derivative
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%#eml
function xdot = get_xdot_2link_arm_maccepa ( x, u, model)

dimX = size(x,1);
xdot = zeros(dimX, 1);
acc  = zeros(2,1);

alpha = model.alpha;

% x1 = q1, x2 = q2, x3 = q1dot, 
% x4 = q2dot, x5 = qm1, x6 = qm2, x7 = qm1dot, x8 = qm2dot

acc = get_acceleration_2link_arm_maccepa ( x, u, model );

xdot = ...
[x(3:4); ...
 acc; ...
 x(7:8); ...
 -2*alpha(1)*x(7)-alpha(1)^2*(x(5)-u(1)); ...
 -2*alpha(2)*x(8)-alpha(2)^2*(x(6)-u(2)); ...
 ];

