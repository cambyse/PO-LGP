function K = k_gauss(X, XPrime, Sigma, sigma)
%implements a squared exponential kernel with width gamma

    %note that this uses the fact that the norm squared is nothing
    %but the scalarproduct of the vector with it self. Then one can use
    %the bilinearity of the scalarproduct. 
    K = sigma^2*exp(-(bsxfun(@plus,diag(X*inv(Sigma)*X'),diag(XPrime*inv(Sigma)*XPrime')') - 2*X*inv(Sigma)*XPrime')/2);
end
