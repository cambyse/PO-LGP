clear;
myDefs;
folder = ['data/door106cma/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end
samples
values

n = size(X,1);
iter=1:n;
%%
figure(1);clf;hold on;
plot3(X(YS==1,1),X(YS==1,2),Y(YS==1),'g.')
plot3(X(YS==-1,1),X(YS==-1,2),0*Y(YS==-1),'r.')


%%
figure(2);clf;hold on;
% col={'b','r','g','m','y','b','r','g','m','y'};
nC = (size(X,1)-mod(size(X,1),6))/6;
col = [linspace(0.,1.,nC)',linspace(0.4,.5,nC)',linspace(1,0,nC)']
plot(mean1(1),mean1(2),'+k')
grid on;axis tight;
M = [];S = [];
for i=1:6:(size(X,1)-mod(size(X,1),6))
 plot(X(i:i+6,1),X(i:i+6,2),'Color',col((i-1)/6+1,:),'Marker','.','LineStyle','none');
 mx = mean(X(i:i+6,:));
 sx = std(X(i:i+6,:));
 plot(mx(1),mx(2),'Color',col((i-1)/6+1,:),'Marker','o','LineStyle','none');
 plot([mx(1)-sx(1);mx(1)+sx(1)],[mx(2);mx(2)],'Color',col((i-1)/6+1,:),'LineStyle','-');
 plot([mx(1);mx(1)],[mx(2)-sx(2);mx(2)+sx(2)],'Color',col((i-1)/6+1,:),'LineStyle','-');
 save([folder,'stdv.dat'],'sx','-ascii');
%  
 M= [M;mx];
 S= [S;sx];
end

%% 
figure(3);clf;hold on;
plot(iter(YS==1),Y(YS==1),'-.');

%%
figure(4);clf;hold on;
plot(M(:,1),'b.-');
plot(M(:,2),'g.-');
plot(M(:,1)+S(:,1),'b--')
plot(M(:,1)-S(:,1),'b--')
plot(M(:,2)+S(:,2),'g--')
plot(M(:,2)-S(:,2),'g--')

