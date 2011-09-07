clear
clc
%load qD.dat;
%load xD.dat;
qD = load('q.dat');
xD = load('target.dat');
load x.dat;
N = size(qD,1)/400;
q = reshape(qD',400*7,[])';

if false
d = dist2(q,q)/(400*7);
imagesc(d);
s = exp(-d);
[a ] = kernel_WithPlug(s,3);
scatter3(a(:,1),a(:,2),a(:,3),30,1:N,'filled');
end

nCl = 16;
[ind meanQ] = kmeans(q,nCl,'replicates',5);

for i = 1:nCl
    subplot(4,4,i );
    indcl = find(ind == i);
   qcl = q(indcl,:);
  
    if size(indcl,1) > 1
    meanX(i,:) = mean(xD(indcl,:));
      stcl = std(qcl)/sqrt(size(indcl,1))'; 
    else
        meanX(i,:) = xD(indcl,:);
        stcl = qcl*0;
    end
    
     plot(reshape(stcl,7,[])')
end

meanQ = reshape(meanQ',7,[])';
save('qD.dat','meanQ','-ascii');
save('xD.dat','meanX','-ascii');

return

load x.dat
%load plot.dat
%x = plot;
T = 400;
N = size(x,1)/T;
hold on
for i = 0:N-1
    traj = x(i*T+1:i*T+T,:);
    traj = traj(1:10:T,:);
    plot3(traj(:,1),traj(:,2),traj(:,3));
    scatter3(traj(1,1),traj(1,2),traj(1,3),40,'filled');
end




return
%for z.trana
d = zeros(size(textdata,1),8);
for i = 1:size(d,1)
  % d(i,1) = str2num(textdata{i,2}); 
   d(i,2) = str2num(textdata{i,4});
   d(i,3) = str2num(textdata{i,6});
   
    d(i,4) = str2num(textdata{i,8});
   d(i,5) = str2num(textdata{i,9});
    d(i,6) = str2num(textdata{i,10});
    d(i,7) = str2num(textdata{i,11});
    d(i,8) = str2num(textdata{i,12});
end 
plot(d)
