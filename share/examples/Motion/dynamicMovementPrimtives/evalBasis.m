function evalBasis(C,H,t)
  figure(10);clf;hold on;
  n = length(t);
  
  for i=1:length(C)
   y = exp(-H(i)*(repmat(C(i),n,1)-t).^2)
   plot(t,y);
  end
  
end