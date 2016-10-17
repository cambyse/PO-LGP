 d = load('data/FLObs_touchdownAndSlide_2.dat');
figure;
hold on;
plot(d(4000:end,1));
plot(d(4000:end,2));
plot(d(4000:end,3));
plot(d(4000:end,4));
plot(d(4000:end,5));
plot(d(4000:end,6));
legend('z','x','y','torque x', 'torque y', 'torque z')
ylabel('forces/torques')

endeffLPosObs = load('data/endeffLPosObs_.dat');
endeffLPosRef = load('data/endeffLPosRef_.dat');
%endeffLPosObsM2 = load('endeffLPosObs_M2.dat');
%endeffLPosObsI = load('endeffLPosObs_I.dat');
%limitsObs = load('limitsObs_.dat');
%figure;
%plot(endeffLPosObs(:,2),-endeffLPosObs(:,1),endeffLPosRef(:,2),-endeffLPosRef(:,1));

%figure;
%plot(endeffLPosObs(:,1),endeffLPosObs(:,3), endeffLPosRef(:,1),endeffLPosRef(:,3))
%norm(endeffLPosObs(100:1600,1:2)-endeffLPosRef(100:1600,1:2))
%sqrt(mean(sum((endeffLPosObs(100:2400,1:2)-endeffLPosRef(100:2400,1:2)).^2,2)))

%figure;
%plot(limitsObs);