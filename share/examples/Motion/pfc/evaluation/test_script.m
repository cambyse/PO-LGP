clear

filename = '../../mpc/out/x.output';

goal = [0.0145, 0.5333, 0.5773, -0.1295, -0.8577, -0.7189, -0.1314];
% [acc_costs, goalPos_costs, goalVec_costs] = evaluate_motion(filename, goal, 0.04)

filename = '../out/scenes/scene1/';
[acc_costs, goalPos_costs, goalVec_costs] = evaluate_motion(filename)