clear;
myDefs;

VAL_d = load('../out/VAL_d');
PHI_d = load('../out/PHI_d');
X_d = load('../out/X_d');
VAL_hist = load('../out/VAL_hist');
PHI_hist = load('../out/PHI_hist');
X_hist = load('../out/X_hist');
PARAM_hist = load('../out/PARAM_hist');
W = load('../out/W');
w = load('../out/w');
b = load('../out/b');

featIdx = 10:12;%4:6;
ssR = 1;

xx = [-1.2:.1:1.2]';
yy = [-1.2:.1:1.2]';

[XX,YY]=meshgrid(xx,yy);
S = [XX(:),YY(:),XX(:)*0+2]';
zz3 = diag(S'*W*S)'+w'*S+b;
zz3=reshape(zz3,length(xx),length(yy));

q = [0:0.01:2*pi]';
S = [sin(q)*1.2,cos(q)*1.2,q*0+2]';

% f = diag(PHI_d(1:ssR:end,featIdx)*W*PHI_d(1:ssR:end,featIdx)')+PHI_d(1:ssR:end,featIdx)*w+b;
f1 = diag(S'*W*S)'+w'*S+b;
f2 = diag(S'*eye(3)*S)'+[0;-1.2;-2]'*S+1;%diag(PHI_d(1:ssR:end,featIdx)*eye(3)*PHI_d(1:ssR:end,featIdx)')+PHI_d(1:ssR:end,featIdx)*[0;-1.2;-2]+1;

% 
[mf1,mf1i] = min(f1);
disp('distance to goal')
disp(norm(S(:,mf1i)-[0;1.2;2]))

%%
figure(1);clf;hold on;
plot3(PHI_hist(1:ssR:end,featIdx(1)),PHI_hist(1:ssR:end,featIdx(2)),VAL_hist(1:ssR:end),'b.','MarkerSize',5);
plot3(PHI_d(1:ssR:end,featIdx(1)),PHI_d(1:ssR:end,featIdx(2)),VAL_d(1:ssR:end),'r.','MarkerSize',20);

% plot goal
plot3(0,1.2,0,'.k','MarkerSize',40)
grid on;

% plot quadratic estimate
% plot3(S(1,:),S(2,:),f1,'k','LineWidth',5);
% plot3(S(1,:),S(2,:),f2,'k');
% plot3(S(1,mf1i),S(2,mf1i),mf1,'g.','MarkerSize',20)
surf(xx,yy,zz3)
shading interp;
axis tight;

figure(2);clf;hold on;
plot(log(sum(diff(PARAM_hist(1:4:end,:)).^2,2)));
figure(3);clf;hold on;
plot(PARAM_hist);



















return;
xx = [min(x):.1:max(x)]';
yy = [min(y):.1:max(y)]';
[xx,yy]=meshgrid(xx,yy);
xx = xx(:);
yy = yy(:);
zz2 = diag([xx';yy']'*W(1:2,1:2)*[xx';yy'])'+w(1:2)'*[xx';yy']+b;

for j=1:length(xx)
 for i=1:length(yy)
%   zz2(i,j) = [xx(i);yy(j)]'*W(1:2,1:2)*[xx(i);yy(j)]+w(1:2)'*[xx(i);yy(j)];
 end
end
% surfc(xx,yy,zz2)
plot3(xx,yy,zz2,'g.');

shading flat;
return;
figure(5);clf;hold on;
surfc(xx,yy,zz2)
shading flat;
plot3(x,y,VAL_d,'r.');
axis tight;
return;

figure(1);clf;hold on;
plot(VAL_d(1:200:end))




figure(2);clf;hold on;
% plot3(PHI_d(:,10),PHI_d(:,11),PHI_d(:,12))
% plot3(PHI_d(:,10),PHI_d(:,11),PHI_d(:,12))
var(PHI_d)

figure(3);clf;hold on;
plot3(x,y,VAL_d,'.');

figure(4);clf;hold on;
plot(X_d,VAL_d,'.');