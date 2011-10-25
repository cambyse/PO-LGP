%input

filename = 'data2Class.txt';
d = 2;

% 1 = linear, 2 = quadratic
features = 1;

% input

[X y] = load_data(filename, d);
n = size(X)(1);

if features == 1
	fX = linear_features(X);
else 
	fX = quadratic_features(X);
endif

fd = size(fX)(2);


% classification

lambda = 0.1;
p = zeros([n 1]);
w = zeros([n 1]);
beta = zeros([fd 1]);

for t = 1 : 10
	for i = 1 : n
		p(i) = 1 / (exp(-fX(i,:)*beta) + 1);
		w(i) = p(i) * (1-p(i));
		W = diag(w);
	end
	
	H = - fX.'*W*fX - 2*lambda*eye(fd);
	beta = beta - inv(H) * fX.'*(y - p) - 2 * lambda * eye(fd) * beta;
end

logL = 0;
for i = 1 : n
	p(i) = 1 / (exp(-fX(i,:)*beta) + 1);
	logL = logL + y(i)*log(p(i)) + (1 - y(i))*log(1 - p(i)) - lambda*norm(beta)^2;
end
logL = 1/n*logL;

% output

disp('beta = '), disp(beta);

disp('log-likelyhood = '), disp(logL);

%generating test grid
min1 = min(X(:,1));
max1 = max(X(:,1));
min2 = min(X(:,2));
max2 = max(X(:,2));

k = 15;
z1 = [min1:(max1-min1)/(k-1):max1]; 
z2 = [min2:(max2-min2)/(k-1):max2];

Z = zeros([k*k 2]);

for i = 1:k
	for j = 1:k
		Z((i-1)*k+j,1) = z1(i);
		Z((i-1)*k+j,2) = z2(j);
	end	
end

%features of test grid
if features == 1
	fZ = linear_features(Z);	
else
	fZ = quadratic_features(Z);	
endif

%probabilities of test grid
pZ = zeros([k*k 1]);
for i = 1:k*k
	pZ(i) = 1 / (exp(-fZ(i,:)*beta) + 1);
end


% drawing figure
figure(1);
hold off;
scatter3(X(:,1), X(:,2), y, 5 , [0 0 0]);
hold on;
%scatter3(Z(:,1), Z(:,2), pZ, 5, [1 0 0]);
mesh(z1, z2, reshape(pZ, [k k]));