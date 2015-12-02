D = [-0.5,0.3;0.5,-0.1;0.9,1.2;0.4,-0.5;0.8,0.3;-0.7,0.2;-0.8,0.2];
%D = [-0.5,0.3;0.5,-0.1];
%D = zeros(100,2);
%D(:,1) = linspace(-1,1,100);
%D(:,2) = sin(D(:,1)).*rand(100,1);

n = size(D,1);
d = size(D,2)-1;
xi = D(:,1:d);
yi = D(:,end);

gamma = 2;
l = 0.2;
k = @(xi,xj) expKernel(xi, xj, gamma, l);

K = calcKernelMatrix(k,xi);

lambda = 0;%0.1;
sigma = 0.1;

xRange = linspace(-1,1,1000);
nXRange = numel(xRange);
kappa = zeros(d,1);
meanF = zeros(nXRange,1);
varF = zeros(nXRange,1);
varFP = zeros(nXRange,1);

for i = 1 : nXRange
    for j = 1 : n
        kappa(j,1) = k(xRange(i),xi(j,:));
    end;
    meanF(i,1) = kappa'*inv(K+lambda*eye(n,n))*yi;
    varF(i,1) = sqrt(k(xRange(i),xRange(i)) - kappa'*inv(K+lambda*eye(n,n))*kappa);
    varFP(i,1) = sqrt(varF(i,1)^2 + sigma^2);
end;

plot(xi,yi,'xr','MarkerSize',10);
hold on;
fill([xRange fliplr(xRange)], [meanF' fliplr((meanF+varF)')], 'b', 'EdgeColor','none', 'facealpha',.5);
fill([xRange fliplr(xRange)], [meanF' fliplr((meanF-varF)')], 'b', 'EdgeColor','none', 'facealpha',.5);
plot(xRange,meanF,'Color',[0.8 0.5 0],'LineWidth',2);
plot(xi,yi,'xr','MarkerSize',10,'LineWidth',2);
title('gamma = 2, without sigma^2');
hold off;

figure;
plot(xi,yi,'xr','MarkerSize',10);
hold on;
fill([xRange fliplr(xRange)], [meanF' fliplr((meanF+varFP)')], 'b', 'EdgeColor','none', 'facealpha',.5);
fill([xRange fliplr(xRange)], [meanF' fliplr((meanF-varFP)')], 'b', 'EdgeColor','none', 'facealpha',.5);
plot(xRange,meanF,'Color',[0.8 0.5 0],'LineWidth',2);
plot(xi,yi,'xr','MarkerSize',10,'LineWidth',2);
title('gamma = 2, with sigma^2');
hold off;

