if (optPlot)
 if (size(t,2)==1)
  %% plotting
  set(0,'DefaultFigureWindowStyle','docked')
  figure(1);clf;hold on;
  title('Function value');
  shadedErrorBar(t,yr,sr2);
  plot(t,y_gt,'-');
  plot(X(YS==1),Y(YS==1),'ko');
  plot(X(YS==-1),Y(YS==-1),'rx');
  plot(t,yr,'b-');
  plot(x_exp,y_exp,'.r','MarkerSize',20)
  axis tight;
  
  figure(2);clf;hold on;
  title('Safety region');
  shadedErrorBar(t,fcmu,sqrt(fcs2)/exp(gpc.hypC.cov(end)));
  plot(t,yS_gt,'-');
  plot(X(YS==1),YS(YS==1),'ko');
  plot(X(YS==-1),YS(YS==-1),'rx');
  plot(t(abs(yc)<1e-1),yc(abs(yc)<1e-1),'r.','MarkerSize',15);
  
  
  figure(3);clf;hold on;
  title('Acquisition function');
  plot(t,PI,'-');
  plot(t,a,'g-');
  legend('PI','a');
 else
  figure(1);clf;hold on;
  title('Function value');
  tmp = reshape(yr,size(t1));
  s=surface(t1(1:plot_ss:end,1:plot_ss:end),t2(1:plot_ss:end,1:plot_ss:end),tmp(1:plot_ss:end,1:plot_ss:end));
  set(s,'FaceColor','flat','LineStyle','none');
%   tmp = reshape(y_gt,size(t1));
%   s=surface(t1(1:plot_ss:end,1:plot_ss:end),t2(1:plot_ss:end,1:plot_ss:end),tmp(1:plot_ss:end,1:plot_ss:end));
%   set(s,'FaceColor','flat','LineStyle','none');
  plot3(X(YS==1,1),X(YS==1,2),Y(YS==1),'g.','MarkerSize',20);
  plot3(X(YS==-1,1),X(YS==-1,2),Y(YS==-1)*0+max(Y(YS==1)),'r.','MarkerSize',20);
  plot3(x_exp(1),x_exp(2),yr(k),'k.','MarkerSize',30);
  view([-60,55]);
  grid on;
  
  figure(2);clf;hold on;
  title('Safety region');
  tmp = reshape(yc,size(t1));

  s=surface(t1(1:plot_ss:end,1:plot_ss:end),t2(1:plot_ss:end,1:plot_ss:end),tmp(1:plot_ss:end,1:plot_ss:end));
  set(s,'FaceColor','flat','LineStyle','none');
  grid on;
  plot3(X(YS==1,1),X(YS==1,2),YS(YS==1),'g.','MarkerSize',20);
  plot3(X(YS==-1,1),X(YS==-1,2),YS(YS==-1)*0,'r.','MarkerSize',20);
  plot3(x_exp(1),x_exp(2),1,'k.','MarkerSize',30);
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
  plot3(x_exp(1),x_exp(2),1,'m.','MarkerSize',40)
  % legend('a');
  view([-73,16]);
  grid on;
  zlim([1e-3,max([a;1])])
  
 end
end
