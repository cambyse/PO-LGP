% train regression problem
%hypR = minimize(hypR, @gp, -200, @infExact, [], covfunc, likfunc, X(YS==1,:), Y(YS==1));

[m, s2] = gp(hypR, @infExact, meanfuncR, covfuncR, likfuncR, X(YS==1,:), Y(YS==1), x_grid);

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
x_n = x_grid(k,:)';

%% plotting
figure(1);clf;hold on;
surf1 = surface(t1,t2,reshape(m,size(t1)));
%surf2 = surface(t1,t2,reshape(m+2*sqrt(s2),size(t1)));
plot3(X(:,1),X(:,2),Y,'r.');
view(2);

figure(2);clf;hold on;
surface(t1,t2,reshape(yS_pred.*(e*fs2+PI),size(t1)));
plot3(X(YS==1,1),X(YS==1,2),Y(YS==1)*0-0.1,'r.');
plot3(X(YS==-1,1),X(YS==-1,2),Y(YS==-1)*0-0.1,'rx');
surface(t1,t2,reshape(p_con,size(t1)));
plot3(x_n(1),x_n(2),-1,'g.','MarkerSize',30);
view(2);
save('state.mat');
