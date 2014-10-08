cd ../09-gamepadDemo/
load joints.dat
cd ../10-nik-MultiPlan/

joints = joints(:,1:7);

plot(joints)

cl1 = joints(1250:2100,:);
clust1 = interp1(1:size(cl1,1),cl1,1:size(cl1,1)/400:size(cl1,1));
subplot(1,2,1)
plot(cl1)
subplot(1,2,2)
plot(clust1)


cl1 = joints(2500:3650,:);
clust2 = interp1(1:size(cl1,1),cl1,1:size(cl1,1)/400:size(cl1,1));

cl1 = joints(4200:5150,:);
clust3 = interp1(1:size(cl1,1),cl1,1:size(cl1,1)/400:size(cl1,1));

cl1 = joints(5400:6500,:);
clust4 = interp1(1:size(cl1,1),cl1,1:size(cl1,1)/400:size(cl1,1));

total = [clust1' clust2' clust3' clust4' ]';


save('ClustQ4.txt','total','-ascii')