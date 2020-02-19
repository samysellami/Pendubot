function dxdt = ode_pendubot(t,x,u)
    dxdt = zeros(4,1);
    B = [1, 0]';

    dxdt(1:2) = x(3:4);
    dxdt(3:4) = M_mtrx_fcn(x(1:2))\(B*u' - n_vctr_fcn(x(1:2),x(3:4)));
end