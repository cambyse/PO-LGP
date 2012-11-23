% Estimate spring force of simplified MACCEPA model for a given command u and state x. 
function F = get_spring_tension_maccepa ( x, u, model )

%#eml

F = 0;
F = get_spring_force_maccepa(x, u, model);

F = -F; % tension = -spring force