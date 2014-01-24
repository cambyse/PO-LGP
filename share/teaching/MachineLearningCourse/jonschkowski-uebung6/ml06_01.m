X = load_data('gauss.txt', 2);
n = size(X)(1);

% a) compute the mean
mu = 1/n*sum(X)

% b) center data
X_c = X - ones(n,1)*mu;

% c) compute Covariance Matrix
C = 1/n*X_c.'*X_c
C2 = 1/n*X.'*X - mu.'*mu

% d)  compute singular value decomposition: C = U*S*V', lamba = diag S, v = columns of V
[U, S, V] = svd(C);

% plot data and the two line segments between μ and μ + λj vj , j = 1, 2
figure(1);
axis equal;
hold off;
plot(X(:,1),X(:,2),'or');
hold on;
for j = 1 : 2
	L = [mu; mu + sqrt(S(j,j))*V(:,j).']
	plot(L(:,1), L(:,2), 'LineWidth', 3);
end