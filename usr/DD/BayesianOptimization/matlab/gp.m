function [mean,var] = gp(x,y,k,lambda)
n = size(x,1);
K = k(x,x);
invK = inv(K+lambda*eye(n,n));
mean = @(xE) k(x,xE)'*invK*y;
var = @(xE) diag(k(xE,xE)) - diag(k(x,xE)'*invK*k(x,xE)) + 0.2;
% for i = 1 : n
%     for j = 1 : n
%         kappa(j) = k(x(i),x(x(i,:)));
%     end;
%     mean(i,1) = kappa'*inv(K+lambda*eye(n,n))*y;
%     var(i,1) = k(x(i,:),x(i,:)) - kappa'*inv(K+lambda*eye(n,n))*kappa;
% end;
% for i = 1 : nXRange
%     for j = 1 : n
%         kappa(j,1) = k(xRange(i),xi(j,:));
%     end;
%     meanF(i,1) = kappa'*inv(K+lambda*eye(n,n))*yi;
%     varF(i,1) = sqrt(k(xRange(i),xRange(i)) - kappa'*inv(K+lambda*eye(n,n))*kappa);
%     varFP(i,1) = sqrt(varF(i,1)^2 + sigma^2);
% end;

end

