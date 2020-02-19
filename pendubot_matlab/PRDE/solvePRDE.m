function [K_mtrx, X] = solvePRDE(A,B,Q,R,t,optns,varargin)

% OPTIMIZATION OPTIONS
solver      = optns.solver;
M           = optns.M;
sgma        = optns.sgma; % shift A
d           = optns.d; % boundaries on solution
pol_type    = optns.pol_type;

T           = t(end);
N           = length(t);
w           = 2*pi/T; % Frequency
n_sts       = size(A,1);  

% Polynomial paramters
pol_params.M = M;
pol_params.w = w;
pol_params.T = T;

J = 0;              % Objective function
C = [];             % Constraints

% Matrix coeffients of polynomial
if strcmp(pol_type,'T')
    Xa=sdpvar(n_sts,n_sts,M+1); 
    Xb=sdpvar(n_sts,n_sts,M);
elseif  strcmp(pol_type,'B')
    Xa=sdpvar(n_sts,n_sts,M+1);
end

for k = 1:1:N %for each time step
    % Evaluate trigonometric poynomial at time t_k
    if strcmp(pol_type,'T')
        [xt, xtd] = matrixTrigPolynomial(Xa,Xb,t(k),pol_params);
    elseif  strcmp(pol_type,'B')
        [xt, xtd] = matrixBezierPolynomial(Xa,t(k),pol_params);
    end

    J = J + trace(xt);

    Ct = [xtd + A(:,:,k)'*xt + xt*A(:,:,k) + 2*sgma*xt + Q(:,:,k), ... %
          xt*B(:,:,k); B(:,:,k)'*xt R(:,:,k)]; 
    
    C = [C, Ct >= 0, xt >= -d*eye(n_sts), xt<=d*eye(n_sts)];
end

if nargin == 7
    L = cell2mat(varargin);
    if strcmp(pol_type,'T')
        [xt1, ~] = matrixTrigPolynomial(Xa,Xb,t(1),pol_params);
        [xtN, ~] = matrixTrigPolynomial(Xa,Xb,t(end),pol_params);
    elseif  strcmp(pol_type,'B')
        [xt1, ~] = matrixBezierPolynomial(Xa,t(1),pol_params);
        [xtN, ~] = matrixBezierPolynomial(Xa,t(end),pol_params);
    end

    C = [C, L'*xt1*L == xtN];
end
optimize(C,-J,sdpsettings('solver',solver,'sdpt3.maxit',200))

if strcmp(pol_type,'T')
    X1 = value(Xa);
    X2 = value(Xb);
elseif  strcmp(pol_type,'B')
    X1 = value(Xa);
end   

% Allocating memory for output matrices
X = zeros(n_sts,n_sts,N);
K_mtrx = zeros(size(B,2),n_sts,N);

for k = 1:N %for each time step
    if strcmp(pol_type,'T')
        [X(:,:,k),~] = matrixTrigPolynomial(X1,X2,t(k),pol_params);
    elseif  strcmp(pol_type,'B')
        [X(:,:,k),~] = matrixBezierPolynomial(X1,t(k),pol_params);
    end
    K_mtrx(:,:,k) = -R(:,:,k)\B(:,:,k)'*X(:,:,k);
end

end