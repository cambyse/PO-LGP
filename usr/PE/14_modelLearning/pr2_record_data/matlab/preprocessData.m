clear
r1 = load('../data/run1_filtered.mat');
r2 = load('../data/run2_filtered.mat');

pos = [r1.pos;r2.pos];
vel = [r1.vel;r2.vel];
effort = [r1.effort;r2.effort];
acc = [r1.acc;r2.acc];

% map dimension 5 and 7 to [-pi,pi]
pos(:,5) = mod(pos(:,5)+pi,2*pi)-pi;
pos(:,7) = mod(pos(:,7)+pi,2*pi)-pi;



save('../data/data_filtered.mat','pos','vel','acc','effort');

clear
r1 = load('../data/run1_unfiltered.mat');
r2 = load('../data/run2_unfiltered.mat');

pos = [r1.pos;r2.pos];
vel = [r1.vel;r2.vel];
effort = [r1.effort;r2.effort];
acc = [r1.acc;r2.acc];

% map dimension 5 and 7 to [-pi,pi]
pos(:,5) = mod(pos(:,5)+pi,2*pi)-pi;
pos(:,7) = mod(pos(:,7)+pi,2*pi)-pi;

save('../data/data_unfiltered.mat','pos','vel','acc','effort');