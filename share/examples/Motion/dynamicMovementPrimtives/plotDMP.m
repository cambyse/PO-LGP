clear;
myDefs;
folder = ['data/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

figure(4);clf;hold on;
plot(x_bk,x_bk)
plot(C,C*0,'.')
plot(C,C*0,'.')

x=[0:.005:1]';
evalBasis(C,H,x);



figure(5);clf;hold on;
plot(y_bk(:,1))
plot(y_ref(:,1),'r')
plot(y_bk(:,2),'b')
plot(y_ref(:,2),'r')

figure(6);clf;hold on;
plot(Return);
