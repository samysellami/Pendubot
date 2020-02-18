function [xt,xtd] = matrixBezierPolynomial(Xa,t_k,par)

    M = par.M;
    T = par.T;
    tau = t_k/T;
    
    xt = zeros(size(Xa(:,:,1)));
    xtd = zeros(size(Xa(:,:,1)));

    for k = 0:M
        b_km = bernstein_polynomial(k,M,tau);
        xt = xt + Xa(:,:,k+1)*b_km;
    end
   
    for k = 0:M-1
        b_km1 = bernstein_polynomial(k,M-1,tau);
        xtd = xtd + (Xa(:,:,k+2) - Xa(:,:,k+1))*b_km1;
    end
    xtd = M*xtd;
    
    function b = bernstein_polynomial(k,m,t)
        t1 = factorial(m)/(factorial(k)*factorial(m-k));
        b = t1*t^k*(1-t)^(m-k);
    end
end