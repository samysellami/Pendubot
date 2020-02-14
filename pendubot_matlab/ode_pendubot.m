function dxdt = ode_pendubot(t,x,u)
    dxdt = zeros(4,1);
    
%     vsc_frcn = [2 0; 0 0.5]*x(3:4);
%     clmb_frcn = [1.2 0; 0 0.25]*tanh(x(3:4)./0.05);%sign(x(3:4));
    B = [1, 0]';

    dxdt(1:2) = x(3:4);
    dxdt(3:4) = M_mtrx_fcn(x(1:2))\(B*u' - n_vctr_fcn(x(1:2),x(3:4)));
end