clear;
myDefs;
FLgood =  load(['plots/good/FLact.dat']);
FLbad =  load(['plots/bad/FLact.dat']);


figure(1);clf;hold on;
plot(sum(abs(FLgood),2),'g')
plot(sum(abs(FLbad),2),'r')

sum(sum(abs(FLgood(25706:end,:)),2))
sum(sum(abs(FLbad(25706:end,:)),2))