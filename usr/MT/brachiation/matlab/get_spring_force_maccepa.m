% Estimate spring force of simplified MACCEPA model for a given command u and state x. 
function F = get_spring_force_maccepa ( x, u, model )

%#eml

k = model.spring_constant;
B = model.lever_length;
C = model.pin_displacement;
r = model.drum_radius;

F = 0;

a = u(1)-x(1);

L0 = C-B;
L  = sqrt(B^2+C^2-2*B*C*cos(a)) + r*u(2);

F = -k*(L-L0);