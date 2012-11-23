% Estimate joint stiffness of simplified MACCEPA model for a given command u and state x. 
function stiffness = get_stiffness_maccepa ( x, u, model )

%#eml

k = model.spring_constant;
B = model.lever_length;
C = model.pin_displacement;
r = model.drum_radius;

a = u(1)-x(1);
beta  = r*u(2) - (C-B);
gamma = sqrt(B^2+C^2-2*B*C*cos(a)); % note that since C>B, gamma>0

stiffness = 0;

stiffness = k*cos(a)*B*C*(1+beta/gamma) ...
    -k*(sin(a)*B*C)^2*beta/(gamma^3);

