sigma = 0.00000000000001;
x = rand(1)*8-4;
y = evalF(x,sigma);

D = [x,y];

for i = 1 : 3
    x = rand(1)*8-4;
    y = evalF(x,sigma);

    D = [D;x,y];
end;

figure('units','normalized','outerposition',[0 0.05 1 0.95]);
[mean,var] = gp(D(:,1),D(:,2),@(x,xPrime)k_gauss(x,xPrime,0.5,0.5),0.01);
xRange = linspace(-4,4,500)';
y = mean(xRange);
varY = var(xRange);
subplot(1,2,1);
plot(xRange,y);
hold on;
plot([-4:0.01:4],f([-4:0.01:4]),'--r')
fill([xRange' fliplr(xRange')], [y' fliplr((y+sqrt(varY))')], 'b', 'EdgeColor','none', 'facealpha',.5);
fill([xRange' fliplr(xRange')], [y' fliplr((y-sqrt(varY))')], 'b', 'EdgeColor','none', 'facealpha',.5);
plot(D(:,1),D(:,2),'.','MarkerSize',20)
hold off;

    
nOfIt = 50;

for i = 1 : nOfIt
    [mean,var] = gp(D(:,1),D(:,2),@(x,xPrime)k_gauss(x,xPrime,0.5,0.5),0.01);
    xRange = linspace(-4,4,500)';
    y = mean(xRange);
    varY = var(xRange);
    subplot(1,2,1);
    plot(xRange,y);
    hold on;
    plot([-4:0.01:4],f([-4:0.01:4]),'--r')
    fill([xRange' fliplr(xRange')], [y' fliplr((y+sqrt(varY))')], 'b', 'EdgeColor','none', 'facealpha',.5);
    fill([xRange' fliplr(xRange')], [y' fliplr((y-sqrt(varY))')], 'b', 'EdgeColor','none', 'facealpha',.5);
    plot(D(:,1),D(:,2),'.','MarkerSize',20)
    hold off;
    [xBest,yBest] = calcXBest(D(:,1),D(:,2));
    a = mpi(mean,var,yBest,i);
    subplot(1,2,2);
    plot(linspace(-4,4,100),a(linspace(-4,4,100)'));
    xNext = fminbnd(a,-4,4);
    y = evalF(xNext,sigma);
    D = [D;xNext,y];
    drawnow;
    i
    xNext
    xBest
    pause(0.1);
    %waitforbuttonpress;
end;

