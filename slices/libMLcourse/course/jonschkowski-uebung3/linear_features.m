function X = linear_features(X)
	
	n = size(X)(1);
	X = [ones(n,1) X];