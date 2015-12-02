D = [-0.5,0.3;0.5,-0.1;0.9,1.2;0.4,-0.5;0.8,0.3;-0.7,0.2;-0.8,0.2];

[mean,var] = gp(D(:,1),D(:,2),@(x,xPrime)k_gauss(x,xPrime,0.02,1),0);

xRange = linspace(-1,1,100)';
y = mean(xRange);
varY = var(xRange);
plot(xRange,y);
hold on;
fill([xRange' fliplr(xRange')], [y' fliplr((y+sqrt(varY))')], 'b', 'EdgeColor','none', 'facealpha',.5);
fill([xRange' fliplr(xRange')], [y' fliplr((y-sqrt(varY))')], 'b', 'EdgeColor','none', 'facealpha',.5);
plot(D(:,1),D(:,2),'.')

xBest = calcXBest(D(:,1),D(:,2));
a = mpi(mean,var,xBest);
xNext = fminbnd(a,-1,1);
