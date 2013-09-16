%this script requires the octgpr package
pkg load octgpr
data = load("graspprec.test");
Z = data(:,3);
Y = data(:,2);
X = data(:,1);

subplot(1,2,1);
plot3(X,Y,Z, "r+");
title("samples costs from AICO");
hold on;
subplot(1,2,2);
%create mesh grid for gpr 
t = linspace (0, 1000, 50);
[xi,yi] = meshgrid (t, t);
zi = xi;
idev = 1./ std([X Y]);
gpr = gpr_train([X Y], Z, idev, 1e-5);
zr = gpr_predict(gpr, [vec(xi) vec(yi)]);
%reformat the regressed z
zimax = max (vec (zr)); zimin = min (vec (zr));
zr = reshape(zr, size(zi));
zr = min(zr,zimax);
zr = max(zr,zimin)
contourf(xi,yi,zr,20);
hold on;
pause;
