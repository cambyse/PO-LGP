if (optPlot)
 if (size(t,2)==1)
  %% 1d plot
  set(0,'DefaultFigureWindowStyle','docked')
  figure(1);clf;hold on;
  title('Function value');
  shadedErrorBar(t,yr,sr2);
%   plot(t,y_gt,'-');
  plot(X(YS==1),Y(YS==1),'ko');
  plot(X(YS==-1),Y(YS==-1),'rx');
  plot(t,yr,'b-');
  plot(x_exp,yr(k),'.r','MarkerSize',20)
  axis tight;
  
  figure(2);clf;hold on;
  title('Safety region');
  shadedErrorBar(t,fcmu,sqrt(fcs2)/exp(gpc.hypC.cov(end)));
%   plot(t,yS_gt,'-');
  plot(X(YS==1),YS(YS==1),'ko');
  plot(X(YS==-1),YS(YS==-1),'rx');
  plot(t(abs(yc)<1e-1),yc(abs(yc)<1e-1),'r.','MarkerSize',15);
  
  
  figure(3);clf;hold on;
  title('Acquisition function');
  plot(t,PI,'-');
  plot(t,a,'g-');
  legend('PI','a');
 else
  %% 2d plot
  figure(1);clf;hold on;
  title('Function value');
  tmp = reshape(yr,size(t1));
  s=surface(t1(1:plot_ss:end,1:plot_ss:end),t2(1:plot_ss:end,1:plot_ss:end),tmp(1:plot_ss:end,1:plot_ss:end));
  set(s,'FaceColor','flat','LineStyle','none');
  plot3(X(YS==1,1),X(YS==1,2),Y(YS==1),'g.','MarkerSize',20);
  plot3(X(YS==-1,1),X(YS==-1,2),Y(YS==-1)*0+max(Y(YS==1)),'r.','MarkerSize',20);
  plot3(x_exp(1),x_exp(2),yr(k),'k.','MarkerSize',30);
  X_success = X(YS==1,:);
  Y_success = Y(YS==1);
  [~,max_idx] = max(Y_success);
  plot3(X_success(max_idx,1),X_success(max_idx,2),Y_success(max_idx),'m.','MarkerSize',40);
  view([-60,55]);
  grid on;
  
  [~,idx] = sort(abs(yc));
  idx = idx(1:100);
  plot3(t(idx,1),t(idx,2),yr(idx),'r.')
  
  figure(2);clf;hold on;
  title('Safety region');
  tmp = reshape(yc,size(t1));

  s=surface(t1(1:plot_ss:end,1:plot_ss:end),t2(1:plot_ss:end,1:plot_ss:end),tmp(1:plot_ss:end,1:plot_ss:end));
  set(s,'FaceColor','flat','LineStyle','none');
  grid on;
  plot3(X(YS==1,1),X(YS==1,2),YS(YS==1),'g.','MarkerSize',20);
  plot3(X(YS==-1,1),X(YS==-1,2),YS(YS==-1)*0,'r.','MarkerSize',20);
  plot3(x_exp(1),x_exp(2),1,'k.','MarkerSize',30);
  plot3(X_success(max_idx,1),X_success(max_idx,2),1,'m.','MarkerSize',40);

  grid on;
  
  %
  figure(3);clf;hold on;
  title('Acquisition function');
  tmp = reshape(a,size(t1));
%   s=surface(t1(1:plot_ss:end,1:plot_ss:end),t2(1:plot_ss:end,1:plot_ss:end),tmp(1:plot_ss:end,1:plot_ss:end));
%   s=surface(t1,t2,tmp);
  plot3(t(:,1),t(:,2),a,'.')
  plot3(t(:,1),t(:,2),PI,'r.')
%   set(s,'FaceColor','flat','LineStyle','none');
  plot3(x_exp(1),x_exp(2),1,'k.','MarkerSize',40)
  plot3(X_success(max_idx,1),X_success(max_idx,2),1,'m.','MarkerSize',40);

  % legend('a');
  view([-73,16]);
  grid on;
  zlim([1e-3,max([a;1])])
  
  [~,max_idx2] = min(sum(abs(X-repmat(X_success(max_idx,:),size(X,1),1)),2));

  
 end
end
