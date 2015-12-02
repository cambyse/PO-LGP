D = [1,1,1;-0.5,1,-0.5;-1,-1,0.5;-1.2,-0.9,0.4;0.5,-1,1];
n = size(D,1);
d = size(D,2)-1;
xi = D(:,1:d);
yi = D(:,end);

gamma = 2;
l = 0.2;
k = @(xi,xj) expKernel(xi, xj, gamma, l);

K = calcKernelMatrix(k,xi);

lambda = 0.1;
sigma = 0.1;

xRange = linspace(-2,2,50);
yRange = linspace(-2,2,50);
nXRange = numel(xRange);
nYRange = numel(yRange);

kappa = zeros(d,1);

meanF = zeros(nXRange,nYRange);
stdF = zeros(nXRange,1);

for i = 1 : nXRange
    for m = 1 : nYRange
        xAkt = [xRange(i) yRange(m)];
        for j = 1 : n
           kappa(j,1) = k(xAkt,xi(j,:));
        end;
        meanF(i,m) = kappa'*inv(K+lambda*eye(n,n))*yi;
        stdF(i,m) = sqrt(k(xAkt,xAkt) - kappa'*inv(K+lambda*eye(n,n))*kappa + sigma^2); 
    end;
end;

[y,x] = meshgrid(xRange,yRange);
h(1) = surf(x,y,meanF);
hold on;
h(2) = surf(x,y,meanF+stdF,'EdgeColor','none','facealpha',0.5);
h(3) = surf(x,y,meanF-stdF,'EdgeColor','none','facealpha',0.5);
plot3(xi(:,1),xi(:,2),yi,'xr','MarkerSize',10,'LineWidth',3);
xlabel('x');
