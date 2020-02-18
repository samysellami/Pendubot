function dxdt = ode_alfa_bta_gma(t,x,k,phi0,thta0)
    dxdt = zeros(2,1);
    theta= x(1);
    theta_d = x(2);
    
    alfa = alpha(k,phi0,thta0,theta);
    bta = beta(k,phi0,thta0,theta);
    gma = gama(k,phi0,thta0,theta);

    dxdt(1) = theta_d;
    dxdt(2) = -(bta * theta_d^2 + gma )/alfa;
end