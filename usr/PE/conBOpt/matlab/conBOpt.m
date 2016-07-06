classdef conBOpt < handle
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Implementation of constrained Bayesian optimization
 %
 % Uses GP regression for the cost function and binary GP classification
 % for the success function. The implemented acquisition function combines
 % the probability of improvement and boundary uncertainty.
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 properties
  t;         % grid that defines search space
  n;         % input dimension
  e;         % PI trade-off parameter (default 0)
  bOffset;   % threshold of boundary offset (default 0.1)
  X,Y,YS;    % dataset consisting of input, reward, success
  gpr,gpc;   % hyper parameter for gp regression/classification
  trainHypR; % train hyper parameter of regression gp
  trainHypC; % train hyper parameter of classification gp
  optM;      % method used for optimizing the acquisition function
  %          1: grid search, 2: gradient optimization
  verbose;   % controls the output
  
  t1,t2,t3,yc,ysc2,fcmu,fcs2,yr,sr2,PI,BU,a,fx,x; % variables for plots
  statBC, statBCR, statNOF; % variables for stats
 end
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 methods
  function c = conBOpt(n,t,e,bOffset,verbose,optM,ellC,sfC,ellR,sfR,snR)
   c.n = n;
   c.t = t;
   c.bOffset = bOffset;
   c.e = e;
   c.verbose = verbose;
   c.optM = optM;
   c.X = []; c.Y = []; c.YS = [];
   
   % init classification gp
   c.trainHypC = false;
   c.gpc.meanfunc = @meanConst;  c.gpc.hyp.mean = -7;
   c.gpc.covfunc = {@covSEiso};  c.gpc.hyp.cov = log([ellC, sfC]);
   c.gpc.likfunc = @likLogistic; c.gpc.inf = @infLaplace;
   
   % init regression gp
   c.trainHypR = false;
   c.gpr.meanfunc = [];
   c.gpr.covfunc = @covSEiso;  c.gpr.hyp.cov = log([ellR; sfR]);
   c.gpr.likfunc = @likGauss;  c.gpr.hyp.lik = log(snR);
   
   % matrices for 2d plots
   if (n==2)
    c.t1 = reshape(c.t(:,1),sqrt(size(c.t,1)),sqrt(size(c.t,1)));
    c.t2 = reshape(c.t(:,2),sqrt(size(c.t,1)),sqrt(size(c.t,1)));
   end
   if (n==3)
    m = round(power(size(c.t,1),1/3));
    c.t1 = reshape(c.t(:,1),m,m,m);
    c.t2 = reshape(c.t(:,2),m,m,m);
    c.t3 = reshape(c.t(:,3),m,m,m);
   end
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function [x] = selectNextPoint(c)
   % predict with classifier
   if c.trainHypC && (sum(c.YS==-1)>3)
    c.gpc.hyp = minimize(c.gpc.hyp, @gp, -200, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS);
   end
   [c.yc, c.ysc2, c.fcmu, c.fcs2] = gp(c.gpc.hyp, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS, c.t);
   
   % predict with regression model
   if c.trainHypR && (size(c.X,1)>10)
    c.gpr.hyp = minimize(c.gpr.hyp, @gp, -200, @infExact, c.gpr.meanfunc, c.gpr.covfunc, c.gpr.likfunc, c.X(c.YS==1,:), c.Y(c.YS==1));
   end
   [c.yr, c.sr2] = gp(c.gpr.hyp, @infExact, c.gpr.meanfunc, c.gpr.covfunc, c.gpr.likfunc, c.X(c.YS==1,:), c.Y(c.YS==1), c.t);
   
   % compute acqusition function on grid t
   %    idxIR = find(c.yc>=c.bOffset);    % points in inner region
   idxIR = find(c.yc>c.bOffset);    % points in inner region
   [~,idxB] = sort(abs(c.fcmu));
   idxB = find(c.yc>-c.bOffset & c.yc<c.bOffset); % points on boundary
   [~,b]=max(c.fcs2(idxB));
   
   [yMax, iMax] = max(c.Y(c.YS==1));
   c.PI = zeros(size(c.t,1),1);
   c.PI(idxIR) = normcdf((c.yr(idxIR)-yMax-c.e)./sqrt(c.sr2(idxIR))); % prob. of improvement
   c.BU = zeros(size(c.t,1),1);
   c.BU(idxB) = sqrt(c.fcs2(idxB))/exp(c.gpc.hyp.cov(end));
   %    c.BU(c.BU<0.275)=0; % boundary uncertainty
   
   c.a = zeros(size(c.t,1),1); % acquisition function
   c.a = c.BU + c.PI;
   
   if (c.optM==1)
    %% grid search
    [c.fx,k] = max(c.a);
    c.x = c.t(k,:);
    x = c.x;
   elseif (c.optM==2)
    %% multistart optimization
    [c.x,c.fx] = c.optimizeAcquistion();
    x = c.x;
   end
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function conv = addDataPoint(c,x,y,ys)
   % make sure x is a row vector
   if size(x,1)>size(x,2)
    x = x';
   end
   % check if datapoint is already in database (causes problem with GP)
   conv = false;
   if length(c.X)>0 & (min(sum(abs(c.X-repmat(x,size(c.X,1),1)),2))<1e-5)
    conv = true;
    display(['conBOpt converged after ', num2str(length(c.X)),' iterations']);
   end
   
   % add datapoint
   c.X = [c.X; x];
   c.Y = [c.Y; y];
   c.YS = [c.YS; ys];
   
   % update statics
   Ytmp = c.Y;
   Ytmp(c.YS==-1) = -inf;
   [~,bcIdx] = max(Ytmp);
   c.statBC = [c.statBC; c.X(bcIdx,:)];
   c.statBCR = [c.statBCR; c.Y(bcIdx)];
   c.statNOF = [c.statNOF; sum(c.YS==-1)];
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function plot(c)
   if (c.verbose>0)
    if (c.n==1)
     %% 1d plotting
     set(0,'DefaultFigureWindowStyle','docked')
     figure(1);clf;hold on;
     title('Function value');
     shadedErrorBar(c.t,c.yr,c.sr2);
     if (exist('c.y_gt'))
      plot(c.t,c.y_gt,'-');
     end
     plot(c.X(c.YS==1),c.Y(c.YS==1),'ko');
     plot(c.X(c.YS==-1),c.Y(c.YS==-1),'rx');
     plot(c.t,c.yr,'b-');
     
     plot(c.statBC(end),c.statBCR(end),'.g','MarkerSize',20)
     plot(c.X(end),c.Y(end),'.m','MarkerSize',20)
     axis tight;
     
     figure(2);clf;hold on;
     title('Safety region');
     shadedErrorBar(c.t,c.fcmu,sqrt(c.fcs2));
     if (exist('c.yS_gt'))
      plot(c.t,c.yS_gt,'-');
     end
     plot(c.X(c.YS==1),c.YS(c.YS==1),'ko');
     plot(c.X(c.YS==-1),c.YS(c.YS==-1),'rx');
     plot(c.X(end),c.YS(end),'.m','MarkerSize',20)
     plot(c.t(abs(c.yc)<1e-1),c.yc(abs(c.yc)<1e-1),'r.','MarkerSize',15);
     axis tight;
     
     figure(3);clf;hold on;
     title('Acquisition function');
     plot(c.t,c.BU,'c-');
     plot(c.t,c.PI,'-');
     plot(c.x,c.fx,'.','MarkerSize',20);
     legend('BU','PI');
     ylim([0,1])
     drawnow;
     if (c.verbose>1)
%       keyboard;
     end
    elseif (c.n==2)
     %% 2d plotting
     set(0,'DefaultFigureWindowStyle','docked')
     figure(1);clf;hold on;
     title('Function value');
     tmp = reshape(c.yr,size(c.t1));
     ps = round(size(c.t1,1)/100);
     s=surface(c.t1(1:ps:end,1:ps:end),c.t2(1:ps:end,1:ps:end),tmp(1:ps:end,1:ps:end));
     set(s,'FaceColor','flat','LineStyle','none');
     plot3(c.X(c.YS==1,1),c.X(c.YS==1,2),c.Y(c.YS==1),'k.','MarkerSize',20);
     plot3(c.X(c.YS==-1,1),c.X(c.YS==-1,2),c.Y(c.YS==-1)*0+max(c.Y(c.YS==1)),'r.','MarkerSize',20);
     plot3(c.statBC(end,1),c.statBC(end,2),c.statBCR(end),'g.','MarkerSize',30);
     plot3(c.X(end,1),c.X(end,2),c.Y(end),'m.','MarkerSize',30);
     if (exist('c.Y_gt'))
      tmp = reshape(y_gt,size(t1));
      s=surface(t1(1:ps*5:end,1:ps*5:end),t2(1:ps*5:end,1:ps*5:end),tmp(1:ps*5:end,1:ps*5:end));
      set(s,'FaceColor','none');
     end
     view([-60,55]);
     grid on;
     axis tight;
     drawnow;
     
     figure(2);clf;hold on;
     title('Safety region');
     tmp = reshape(c.yc,size(c.t1));
     s=surface(c.t1(1:ps:end,1:ps:end),c.t2(1:ps:end,1:ps:end),tmp(1:ps:end,1:ps:end));
     set(s,'FaceColor','flat','LineStyle','none');
     grid on;
     plot3(c.X(c.YS==1,1),c.X(c.YS==1,2),c.YS(c.YS==1),'k.','MarkerSize',20);
     plot3(c.X(c.YS==-1,1),c.X(c.YS==-1,2),c.YS(c.YS==-1)*0,'r.','MarkerSize',20);
     plot3(c.statBC(end,1),c.statBC(end,2),c.statBCR(end),'g.','MarkerSize',30);
     plot3(c.X(end,1),c.X(end,2),1,'m.','MarkerSize',30);
     grid on;
     if (exist('c.YS_gt'))
      tmp = reshape(c.YS_gt,size(t1));
      s=surface(t1(1:ps*5:end,1:ps*5:end),t2(1:ps*5:end,1:ps*5:end),tmp(1:ps*5:end,1:ps*5:end));
      set(s,'FaceColor','none');
     end
     axis tight;
     drawnow;
     
     figure(3);clf;hold on;
     title('Acquisition function');
     plot3(c.t(:,1),c.t(:,2),c.BU,'.k')
     tmp = reshape(c.PI,size(c.t1));
     s=surface(c.t1(1:ps:end,1:ps:end),c.t2(1:ps:end,1:ps:end),tmp(1:ps:end,1:ps:end));
     set(s,'FaceColor','flat','LineStyle','none');     
     plot3(c.x(1),c.x(2),c.fx,'m.','MarkerSize',40)
     legend('BU','PI');
     view([-73,16]);
     zlim([1e-3,max([c.a;c.fx;1])])
     grid on;
     drawnow;
     if (c.verbose>1)
%       keyboard;
     end
    elseif (c.n==3)
     %% 3d plotting
     figure(1);clf;hold on;

     isoValues = sort(c.Y(c.YS == 1));
     num = numel(isoValues);
     
     %define the colormap
     clr = jet(num);
     colormap(clr)
     caxis([0 num])
     colorbar('YTick',(1:num)-0.5, 'YTickLabel',num2str(sort(isoValues(:))))
         
     [x,y,z,v] = reducevolume(c.t1,c.t2,c.t3,reshape(c.yc,size(c.t1,1),size(c.t2,1),size(c.t3,1)),[4,4,4]);
     pC = patch(isosurface(x,y,z,v,0));
     set(pC, 'FaceAlpha','0.2','EdgeColor',[0,0,0]/255,'EdgeAlpha',0.4);
     [~,sortInd] = sort(c.Y(c.YS==1));
     XS = c.X(c.YS==1,:);
     XS = XS(sortInd,:);
     scatter3(XS(:,1),XS(:,2),XS(:,3),100,clr,'filled');
     plot3(c.X(c.YS==-1,1),c.X(c.YS==-1,2),c.X(c.YS==-1,3),'rx','MarkerSize',40,'LineWidth',2);
     plot3(c.X(end,1),c.X(end,2),c.X(end,3),'go','MarkerSize',15);

     grid on;
     view([-73,16]);
     axis tight; axis equal;
     drawnow;
     if (c.verbose>1)
%       keyboard;
     end
    end
   end
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function stats(c)
   if (c.verbose>0)
    s = sprintf( '\n### Statistics ###\n');
    s = [s,sprintf( '- amount of samples         : %d\n',length(c.Y))];
    s = [s,sprintf( '- amount of failures        : %d\n',c.statNOF(end))];
    s = [s,sprintf(['- current candidate         : ',repmat('%6.4f  ',1,c.n),'\n'],c.X(end,:))];
    s = [s,sprintf( '- currend candidate reward  : %f\n',c.Y(end))];
    s = [s,sprintf( '- currend candidate success : %d\n',c.YS(end))];
    s = [s,sprintf(['- best candidate            : ',repmat('%6.4f  ',1,c.n),'\n'],c.statBC(end,:))];
    s = [s,sprintf( '- best candidate reward     : %f\n',c.statBCR(end))];
   end
   s
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function [x, fx] = optimizeAcquistion(c)
   lb = min(c.t);
   ub = max(c.t);
   [yMax] = max(c.Y(c.YS==1));
   sampleX0 = @() lb+rand(1,c.n).*(ub-lb);
   
   if (c.verbose<3)
    options = optimoptions('fmincon','GradObj','on','Display','off');
   else
    options = optimoptions('fmincon','GradObj','on','Display','iter-detailed');
   end
   
   %% optimize prob. of improvement inside the success region
   f = @(x) c.nPIfun(yMax,x);
   con = @(x) c.PIcon(x);
   fx = inf;
   for i = 1:1
    [~,k] = max(c.PI);
    x0 = c.t(k,:);
    while con(x0)>0
     x0 = sampleX0();     % sample feasible start point
    end
    [x_i,f_i,optflag]=fmincon(f,x0,[],[],[],[],lb,ub,con,options);
    if (f_i<fx & optflag>0)
     x = x_i;  fx = f_i; % pick best candidate
    end
   end
   
   %% optimize BU on the boundary of the success region
   f = @(x) c.nBUfun(x);
   con = @(x) c.BUcon(x);
   for i = 1:1
    [~,k] = max(c.BU);
    x0 = c.t(k,:);
    while sum(con(x0)>0)>0
     x0 = sampleX0();     % sample feasible start point
    end
    [x_i,f_i,optflag]=fmincon(f,x0,[],[],[],[],lb,ub,con,options);
    if (optflag<0)
     optflag
%      keyboard;
    end
    if (f_i<fx & optflag>0)
     x = x_i;  fx = f_i; % pick best candidate
    end
   end
   
   fx = -fx;
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  % This function computes the negative probability of improvement and
  % the corresponding gradient with respect to the input
  function [f,dfdx] = nPIfun(c,yMax,x)
   X = c.X(c.YS==1,:);
   Y = c.Y(c.YS==1);
   [yr, sr2] = gp(c.gpr.hyp, @infExact, c.gpr.meanfunc, c.gpr.covfunc, c.gpr.likfunc,X,Y,x);
   K = covSEiso(c.gpr.hyp.cov,X,X)+eye(size(X,1))*exp(2*c.gpr.hyp.lik);
   alpha = inv(K)*Y;
   k = covSEiso(c.gpr.hyp.cov,x,X);
   ell2 = exp(c.gpr.hyp.cov(1)).^2;
   L = eye(size(X,2))./ell2;
   Xt = repmat(x,size(X,1),1)-X;
   dyrdx = -L*Xt'*(k'.*alpha);
   dkdx = zeros(size(X,1),c.n);
   for j=1:size(X,1)
    dkdx(j,:) = -L*Xt(j,:)'*k(j);
   end
   dsr2dx = -2*k*inv(K)*dkdx;
   z = (yr-yMax-c.e)./sqrt(sr2);
   f = -normcdf(z);
   dzdx = dyrdx/sqrt(sr2) -(yr-yMax-c.e)*0.5*dsr2dx'*sr2^(-3/2);
   dfdx = -normpdf(z)*dzdx;
   f = f;
   dfdx = dfdx;
  end
  
  % This describes an inequality constraint that measures if an
  % input is inside the success region
  function [h,heq] = PIcon(c,x)
   %    [~, ~, fcmu, ~] = gp(c.gpc.hyp, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS, x);
   %    h = -fcmu;
   yc = gp(c.gpc.hyp, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS, x);
   h = -yc + c.bOffset;
   heq = [];
  end
  
  % This function computes the negative boundary uncertainty and
  % the corresponding gradient with respect to the input
  function [f,dfdx] = nBUfun(c,x)
   [~, ~, ~, fcs2] = gp(c.gpc.hyp, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS, x);
   K = covSEiso(c.gpc.hyp.cov,c.X,c.X);
   k = covSEiso(c.gpc.hyp.cov,x,c.X);
   ell2 = exp(c.gpc.hyp.cov(1)).^2;
   L = eye(size(c.X,2))./ell2;
   Xt = repmat(x,size(c.X,1),1)-c.X;
   [~, ~, fcmu] = gp(c.gpc.hyp, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS, c.X);
   p = sigmoid(fcmu);
   W = diag(p.*(1-p));
   sr22 = covSEiso(c.gpc.hyp.cov,x,x) - k*inv(K + inv(W))*k';
   %    dkdx = -L*Xt.*k';
   dkdx = zeros(size(c.X,1),c.n);
   for j=1:size(c.X,1)
    dkdx(j,:) = -L*Xt(j,:)'*k(j);
   end
   dsr2dx = -2*k*inv(K+inv(W))*dkdx;
   f = -sqrt(fcs2)/exp(c.gpc.hyp.cov(end));
   dfdx = -dsr2dx'/(2*sqrt(fcs2)*exp(c.gpc.hyp.cov(end)));
  end
  
  % This functions describes two inequality constraints that measures if an
  % input is on boundary of the classification gp
  function [h,heq] = BUcon(c,x)
   yc = gp(c.gpc.hyp, c.gpc.inf, c.gpc.meanfunc, c.gpc.covfunc, c.gpc.likfunc, c.X, c.YS, x);
   h = yc-c.bOffset;
   h = [h;-yc-c.bOffset];
   heq = [];
  end
 end
end
