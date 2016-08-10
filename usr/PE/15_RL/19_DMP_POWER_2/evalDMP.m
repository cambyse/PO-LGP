clear;
myDefs;
folder = ['data/door304/'];

files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

i = 0;
cDemo = sum(sum(abs(compAcc(Xdemo,1))));
cDMP = [];
cDMP2 = [];
while exist(['y_bk',num2str(i)])
 FL = eval(['mfFLact',num2str(i)]);
 X = eval(['y_bk',num2str(i)]);
 cDMP = [cDMP;sum(sum(abs(FL)))/size(FL,1)];
%  cDMP2 = [cDMP2;sum(exp(-20*sum(abs(compVel(X,1)),2)))/size(Xdemo,1)];
%  cDMP = [cDMP;sum(sum(abs(compVel(X,1)),2))];
 i=i+1;
 if i>iter
  break;
 end
end

%% plot
figure(1);clf;hold on;
% plot(cDMP);
% plot(s_Return(end-iter:end,2),exp(-log(s_Return(end-iter:end,1)+1e-5).^2),'.');
plot(s_Return(end-iter:end,2),s_Return(end-iter:end,1)+1e-5,'.');
axis tight;

%%
figure(2);clf;hold on;
plot(param(:,end));
plot(param(:,end-1));
plot(param(:,end-2));
plot(param(:,end-3));
plot(param(:,end-4));
plot(param(:,end-5));

figure(3);clf;hold on;
i=0;
while exist(['y_bk',num2str(i)])
 tmp = eval(['y_bk',num2str(i)]);
 plot(tmp(:,1),'r');
 plot(tmp(:,2),'g');
 plot(tmp(:,3),'b');
 plot(tmp(:,4),'m');
 plot(tmp(:,5),'c');
 plot(tmp(:,6),'k');
 i=i+1;
 if i>iter
  break;
 end
end
