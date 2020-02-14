function [out] = check_K(q, q_dot, plnr, Ke, Kd, Kp, E_top)
% -----------------------------------------------------------------------
% This function check if the coefficient of the Lyapunov function
% candidtate satisfies the conditions of non existence of singularities in 
% the control law ( 5.29, 5.31 and 3.28 in the book) 
% Inputs:
%   q, q_dot - states of the system
%   Ke, Kd, Kp - Constants of the Lyapunov function candidate
%   plnr - robot parameters
%   E_top - Energie of the system at the top position
% Outputs:
%   out: boolean which is equal 1 if the conditions are true    
% -----------------------------------------------------------------------
g = 9.81;
eps = 0.1;
c = min(2* plnr.theta(4) * g , 2 * plnr.theta(5) * g);
c1 = min([2* plnr.theta(4) * g , 2 * plnr.theta(5) * g, (Kd - eps)/(Ke * plnr.theta(1))]);
out = 1;

% condition (5.29)
cond = 2 * plnr.theta(1) * (plnr.theta(4) + plnr.theta(5)) * g;
if Kd/Ke < cond
    out = 0;
    disp('condition 1 is not satisfied !!')
end

% condition (5.32)
V0 = Lyap(q, q_dot, plnr, Ke, Kd, Kp, E_top);
if  V0 > Ke * c^2/2
    out = 0;
    disp('condition 2 is not satisfied !!')
end

% condition (5.31)
E = Energie(q, q_dot, plnr);
E_tld = E - E_top;
if  abs(E_tld) > c
    out = 0;
    disp('condition 3 is not satisfied !!')
end

end

