function X = quadratic_features(X)
	
	n = size(X)(1);
	d = size(X)(2);
	
	if d == 1
		X = [ones(n,1)   X   X.*X];
	elseif d == 2
		X = [ones(n,1)   X   X(:,1).*X(:,1)   X(:,1).*X(:,2)   X(:,2).*X(:,2)];
	else
		disp('not implemented yet, for d > 2');
	endif