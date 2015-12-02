function fx = evalF(x,sigma)
    fx = f(x+normpdf(x,0,sigma));
end

