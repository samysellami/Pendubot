clc
clear all
close all

run('benchmark_system')

optns.solver    = 'sdpt3';
optns.M         = 25;
optns.sgma      = 0; 
optns.d         = 1e+5;
optns.pol_type  = 'T';

[K_mtrx, X] = solvePRDE(A,B,Q,R,t,optns);

nrm = zeros(size(X,3),1);
for i = 1:size(X,3)
    nrm(i) = norm(X(:,:,i) - H(:,:,i)) ;
end

if mean(nrm) <1e-3
    disp('Test is successfully solved')
end