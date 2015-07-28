clear;
myDefs;
folder = ['data/door2/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

T = size(mbX,1);

%% plot model free variable over time
cDemo = sum(sum(compVel(Xdemo,1).^2))/T;

varList = {
  'mbXdes','mbXact';...
%   'mfXdes','mfXact';...
  'phaseXdes','phaseXact';...
 % 'mfFLact','mfUact';...
%  'mfUact';...
};

% evalFun = @(x) sum(sum(x(1:end,15).^2))/T;
% evalFun = @(x) sum(sum(compVel(x,1).^2))/T;
evalFun = @(x) sum(sum(compAcc(x,1).^2))/T;

col = {'r--','g','b'};
for v = 1:size(varList,1)
 figure(v);clf;hold on;
 for w = 1:size(varList,2)
  i = 0;
  cmb = [];
  while exist([varList{v,w},num2str(i)])
   Xv = eval([varList{v,w},num2str(i)]);
   cmb = [cmb;evalFun(Xv)];
   i=i+1;
  end
  plot(1:length(cmb),(cmb),col{w});
 end
 legend(varList{v,:})
end


%% eval model free part
Y = Y - min(Y);
[~,idxOpt] = max(Y(YS==1));
figure(9);clf;hold on;
plot3(X(YS==1,1),X(YS==1,2),Y(YS==1),'b.')
plot3(X(YS==-1,1),X(YS==-1,2),0*Y(YS==-1)+13,'r.')
plot3(X(idxOpt,1),X(idxOpt,2),Y(idxOpt),'g.','MarkerSize',30)

addpath('/home/englerpr/Dropbox/research/code/gpml/');
startup;
g = load([folder,'state.mat']);
[t1 t2] = meshgrid(linspace(min(X(:,1)),max(X(:,1)),500),linspace(min(X(:,2)),max(X(:,2)),500));
x_grid = [t1(:) t2(:)];
% hypR = minimize(g.hypR, @gp, -200, @infExact,  g.meanfuncR, g.covfuncR, g.likfuncR, X(YS==1,:), Y(YS==1));
[m, s2] = gp(g.hypR, @infExact, g.meanfuncR, g.covfuncR, g.likfuncR, X(YS==1,:), Y(YS==1), x_grid);

surf1 = surface(t1,t2,reshape(m,size(t1)));view(-99, 32);
% surf2 = surface(t1,t2,reshape(m+s2,size(t1)));view(-99, 32);
% set(surf2,'LineStyle','none')
% set(surf2,'FaceAlpha',0.5)
% set(surf2,'FaceColor','k')
% set(surf1,'FaceColor','c')
% set(surf1,'FaceAlpha',0.4)
set(surf1,'LineStyle','none')
set(surf1,'LineWidth',1.8)
grid on
% light;
% light('Position',[-15 -18 -2],'Style','infinite');
lighting flat;
camlight

[ymu, ys2, fmu, fs2, lp] = gp(g.hypC, @infLaplace, @meanConst,@covSEiso, @likLogistic, X, YS, x_grid);
[~,idx] = sort(abs(fmu));
idx = idx(1:2000);
plot3(x_grid(idx,1),x_grid(idx,2),m(idx),'k.')

xlabel('finger width [m]','FontName', 'Helvetica', 'FontSize', 14);
ylabel('finger position [m]','FontName', 'Helvetica', 'FontSize', 14);
zlabel('reward','FontName', 'Helvetica', 'FontSize', 14);
myexportfig(9,'result.png');