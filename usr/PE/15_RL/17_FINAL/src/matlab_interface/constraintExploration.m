% load('state.mat')

% predict with classifier
if trainHyperC && (sum(YS==-1)>3)
  gpc.hypC = minimize(gpc.hypC, @gp, -200, gpc.inf, gpc.meanfuncC, gpc.covfuncC, gpc.likfuncC, X, YS);
end
[yc, ysc2, fcmu, fcs2] = gp(gpc.hypC, gpc.inf, gpc.meanfuncC, gpc.covfuncC, gpc.likfuncC, X, YS, t);

% predict with regression model
if trainHyperR && (size(X,1)>10)
  gpr.hypR = minimize(gpr.hypR, @gp, -200, @infExact, gpr.meanfuncR, gpr.covfuncR, gpr.likfuncR, X(YS==1,:), Y(YS==1));
end
[yr, sr2] = gp(gpr.hypR, @infExact, gpr.meanfuncR, gpr.covfuncR, gpr.likfuncR, X(YS==1,:), Y(YS==1), t);

% find search region for next datapoint
idxIR = find(fcmu>=0);
[~,idxB] = sort(abs(fcmu));
% idxB = find(abs(yc)<0.2);
idxB = find(fcmu>=0 & yc<0.2);
[~,b]=max(fcs2(idxB));

% compute expected improvement
[vMax, iMax] = max(Y(YS==1));
e = 0;
PI = normcdf((yr-vMax-e)./sqrt(sr2));

% compute boundary uncertainty
BU = sqrt(fcs2(idxB))/exp(gpc.hypC.cov(end));
BU(BU<0.2)=0;

a = zeros(size(t,1),1);
a(idxB) = BU;
a(idxIR) = a(idxIR) + PI(idxIR);

[~,k] = max(a);

x_exp = t(k,:);

% logging
save('state.mat')
