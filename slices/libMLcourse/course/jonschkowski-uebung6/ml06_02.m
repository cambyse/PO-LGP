d = 2;
X = load_data('mixture.txt', d);
n = size(X)(1);

K = 3;

% INITIALIZATION
q = zeros([K n]);
w = zeros([K n]);

mu = X(ceil(rand([1 K])*n),:);

I = eye(d);
C = zeros([K d d]);
for k = 1 : K
	C(k,:,:) = I;
end


for t = 1 : 30
	% EXPECTATION
	% q(ci = k) = P (ci = k | xi , μ1:K , Σ1:K ) ∝ N(xi | μk , Σk ) P (ci = k)
	
	
	for k = 1 : K
		a = 1/((2*pi)^(K/2)*det(squeeze(C(k,:,:)))^(1/2));
		for i = 1 : n
			x_minus_mu = (X(i,:)-mu(k,:)).';
			q(k,i) = a * exp(-1/2*x_minus_mu.'*inv(squeeze(C(k,:,:)))*x_minus_mu)*1/K;

		end
	end

	% normalize q, so that total probability = 1
	s = sum(q); 
	for i = 1 : n
		q(:,i) = q(:,i)/s(i);
	end
			
	% MAXIMIZATION
	% wki = q(ci = k)/(SUM_i(q(ci = k)))
	%μk = SUM_i(wki xi)
	%Ck = SUM_i(wki xi xi.' − μk μk.')
	
	s = sum(q.'); 
	for k = 1 : K
		w(k,:) = q(k,:)/s(k);
	end
		
	mu = w * X;
	
	for k = 1 : K
		C(k,:,:) = X.'*diag(w(k,:))*X - mu(k,:).'*mu(k,:);
	end
	
	figure(1);
	axis equal;
	hold off;
	plot(X(:,1),X(:,2),'or');
	hold on;
	for k = 1 : K
		[U, S, V] = svd(squeeze(C(k,:,:)));
		for j = 1 : 2
			L = [mu(k,:); mu(k,:) + sqrt(S(j,j))*V(:,j).'];
			plot(L(:,1), L(:,2), 'LineWidth', 3);
		end
	end
	
	t
	waitforbuttonpress 
end


