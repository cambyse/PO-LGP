clear;
myDefs;

load('s0.dat');
load('sOpt.dat');
load('X.dat');
load('Xres.dat');

figure(1);clf;hold on;
plot(s0(:,1),s0(:,2),'o')
plot(sOpt(:,1),sOpt(:,2),'r.')

n = 1;


figure(2);clf;hold on;
% plot(X(:,1),sum(compAcc(X(:,2:end),n).^2,2),'r')
plot(Xres(:,1),sum(compVel(Xres(:,2:end),n).^2,2),'g')


figure(3);clf;hold on;
% plot(X(:,1),sum(compAcc(X(:,2:end),n).^2,2),'r')
plot(Xres(:,1),sum(compAcc(Xres(:,2:end),n).^2,2),'g')

