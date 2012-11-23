function [A, a, B] = get_dynamics ( x, u, model );

[A, B] = get_dynamics_jacobian_2link_arm_maccepa ( x, u, model );

xdot = get_xdot_2link_arm_maccepa(x, u, model);

a = xdot - A*x - B*u;
