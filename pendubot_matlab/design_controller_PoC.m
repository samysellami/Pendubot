
T = 2.01;
N = size(A,3);
dt = T/(N-1); % to consider zero 
t = 0:dt:T;
Q = repmat(eye(3),[1,1,N]);
R = repmat(1,[1,1,N]);


optns.solver    = 'sdpt3';
optns.M         = 20;
optns.sgma      = 0.5; 
optns.d         = 1e+5;
optns.pol_type  = 'T';

[K_mtrx, X] = solvePRDE(A,B,Q,R,t,optns);

save('K_mtrx2.mat','K_mtrx')