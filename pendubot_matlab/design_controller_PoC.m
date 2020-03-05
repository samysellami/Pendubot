% Script for designing the controller 
T = T_x;
N = size(A,3);
dt = T/(N-1); % to consider zero 
tt = 0:dt:T;
Qi = eye(3); Qi(1,1) = 1;
Q = repmat(Qi,[1,1,N]);
R = repmat(1,[1,1,N]);


optns.solver    = 'sdpt3';
optns.M         = 20;
optns.sgma      = 1; 
optns.d         = 1e+5;
optns.pol_type  = 'T';

[K_mtrx, X] = solvePRDE(A,B,Q,R,tt,optns);

