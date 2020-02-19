function [V] = Lyap(q, q_dot, plnr, Ke, Kd, Kp, E_top)
% -----------------------------------------------------------------------
% This function compute the Lyapunov function candidate for a value of the 
% states 
% Inputs:
%   q, q_dot - states of the pendubot
%   Ke, Kd, Kp - Constants of the Lyapunov function candidate
%   plnr - robot parameters
%   E_top - Energie of the system at the top position
% Outputs:
%   L: value of the Lyapunov function    
% -----------------------------------------------------------------------
E = Energie(q, q_dot, plnr);

V = Ke/2 * (E - E_top)^2 + Kd/2 * q_dot(1)^2 + Kp/2 * (q(1) - pi/2)^2; 

end

