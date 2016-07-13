clear
myDefs

folders=6:7;%4:10;

figure(1);
for i=1:length(folders)
 x = load(['../13_PR2_DOOR/data/run',num2str(folders(i)),'/pr2_joints']);
 t=1:size(x,1);
 size(x,1)
 plot(t,x(:,2))
end
