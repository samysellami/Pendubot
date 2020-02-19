function [E] = Energie(q, q_dot, plnr)
% -----------------------------------------------------------------------
% This function compute the Energie of the system for a value of the 
% states 
% Inputs:
%   q, q_dot - states of the pendubot
%   plnr - robot parameters
% Outputs:
%   E: value of the Energie of the system    
% -----------------------------------------------------------------------
g = 9.81;

E =  0.5 * q_dot' * M_mtrx_fcn(q) * q_dot + plnr.theta(4) * g * sin(q(1)) + plnr.theta(5) * g * sin (q(1) + q(2));      

end