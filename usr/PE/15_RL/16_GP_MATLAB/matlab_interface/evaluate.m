x_grid = [paramLim(1):0.01:paramLim(2)]';

% train regression problem
[m, s2] = gp(hypR, @infExact, [], covfunc, likfunc, X(YS==1), Y(YS==1), x_grid);

% train classification problem
[~,ymu,ys2,fmu,fs2,hypC,lp] = gpExploration(X,YS,x_grid,hypC);
[vMax, iMax] = max(Y);
e=1e-1;

% Prob of Improvement
PI = normcdf((m-vMax-e)./sqrt(s2));

% find next exploration point
yS_pred = ymu>0&ymu<1e-1;
p_con = (ymu+1)/2;

[~,k] = max(yS_pred.*(e*fs2+PI));
x_n = x_grid(k);

%% plotting
figure(1);clf;hold on;
plot(x_grid,m);
plot(x_grid,m+2*sqrt(s2),'--');
plot(x_grid,m-2*sqrt(s2),'--');
plot(X,Y,'r.');

figure(2);clf;hold on;
plot(x_grid,yS_pred.*(e*fs2+PI),'m');
plot(X(YS==1),Y(YS==1),'.');
plot(X(YS==-1),Y(YS==-1),'x');
plot(x_grid,p_con,'-k');
plot(x_n,1,'r','MarkerSize',20);
