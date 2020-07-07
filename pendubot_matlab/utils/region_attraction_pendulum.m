%% Pendulum 
clc; clear; close all; 

% ------------------------------------------------------------------------
% System dynamics
% ------------------------------------------------------------------------

syms thta thta_d thta_2d u real 
m = 10; 
l = 1;
g = 9.8;

% Kinetic and potential energy
K = 0.5 * m * l^2 * thta_d^2;
P = m * g * l* (1-cos(thta)); 

% Lagrange equations
Lagr = K - P;
dLagr_dq = jacobian(Lagr,thta)';
dLagr_dqd = jacobian(Lagr,thta_d)';
t1 = jacobian(dLagr_dqd,[thta;thta_d])*[thta_d; thta_2d];
t1 = simplify(t1);
dnmcs = t1 - dLagr_dq;
M_sym = jacobian(dnmcs, thta_2d);
n_sym = simplify(dnmcs - M_sym*thta_2d);

matlabFunction(M_sym,'File','autogen/M_mtrx_fcn_pndlm','Vars',{[thta; thta_d]});
matlabFunction(n_sym,'File','autogen/n_vctr_fcn_pndlm','Vars',{[thta; thta_d]});

% linerization about the upper point 
f = [thta_d; M_sym\(u - n_sym)];
A = jacobian(f,[thta; thta_d]);
B = jacobian(f,u);
matlabFunction(f,'File','autogen/f_fcn_pndlm','Vars',{[thta; thta_d], u});
matlabFunction(A,'File','autogen/A_mtrx_pndlm','Vars',{[thta; thta_d]});
matlabFunction(B,'File','autogen/B_vctr_pndlm','Vars',{[thta; thta_d]});

%%
% ------------------------------------------------------------------------
% controller deisgn
% ------------------------------------------------------------------------

clc; clear;
controller_types = {'pp', 'lqr'};
controller = controller_types{2};
x_eq = [pi; 0];

A = A_mtrx_pndlm(x_eq);
B = B_vctr_pndlm(x_eq); 

% Controller coefficients
if strcmp(controller, 'pp')
    % desing state feedback controller using pole placement
    poles = [-2, -3];
    K = place(A, B, poles);
elseif strcmp(controller, 'lqr')
    % design lqr controller
    Q = [10,0;...
         0,10];
    R = 1;
    [K, P, ~] = lqr(A, B, Q, R);
end

if strcmp(controller, 'pp')
    % Closed loop system
    A_tilde = A - B*K;
    % Solve continous lyapunov equation for linearized system
    % V = delta_x' P delta_x   AP + PA' + Q = 0
    Q = [10,0 ;...
         0,10];
    P = lyap(A_tilde', Q);
end    

% domain of attraction
dom_att = [];
V_d_tot = [];
theta_range = pi-pi:0.01:pi+pi;
theta_d_range = -10:0.1:10;

for theta = theta_range
   for theta_d =  theta_d_range
        x = [theta; theta_d];
        u = -K * (x - x_eq);
        V_d = lyap_deriv_function(x,P,x_eq,u);
        if V_d < 0
            dom_att = [dom_att x];
            V_d_tot = [V_d_tot V_d];
        end
   end
end

figure
MARKER_SIZE = 10; %marker size of the scatter function
scatter(dom_att(1,:), dom_att(2,:),MARKER_SIZE,'k','filled', 'DisplayName', 'domaine of attraction')
hold on
scatter(pi, 0, 100,'r','filled','DisplayName', 'equilibrium')
xlabel('$\theta$','interpreter','latex')
ylabel('$\dot{\theta}$','interpreter','latex')
xlim([theta_range(1)  theta_range(end)])
ylim([theta_d_range(1)  theta_d_range(end)])
legend
grid on




%% ------------------------------------------------------------------------
% System Visualization
% ------------------------------------------------------------------------
clc;
n = 5000; % number of iterations 
T = 5; % Final time of the simulation
delta_t = T/n; % iterations step
g = 9.81; % gravity
q = 6; % intial positions 
q_d = -10; % initial velocities
q_s =  pi;  % stabilization point

ode_optns = odeset('RelTol',1e-10,'AbsTol',1e-10);
theta_tot = []; 

% simulate the dynamics
for i= 1:n  
    u = -K * [q-q_s; q_d];
    % applying the control input to the dynamics
    odefun = @(t,x)pndlm_dnmcs(t,x,u);
    [t,x] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns);
    q = x(end,1)';
    q_d = x(end,2)';
    theta_tot = [theta_tot q];
end

visualize_pendulum(theta_tot(1:100:end))
close all;



function V = lyap_function(x,P,x_eq)
    V = (x-x_eq)' * P * (x-x_eq);
end

function V_d = lyap_deriv_function(x,P,x_eq,u)
    V_d  = f_fcn_pndlm(x,u)' * P * (x-x_eq) + (x-x_eq)' * P * f_fcn_pndlm(x,u);
end

function   dxdt = pndlm_dnmcs(t,x,u)
    M = M_mtrx_fcn_pndlm(x);
    n = n_vctr_fcn_pndlm(x);  
        
    dxdt = zeros(2,1);
    dxdt(1) = x(2);
    dxdt(2) = M\( u - n);
end

%{
figure
[X,Y] = meshgrid(dom_att(1,1:400:end),dom_att(2,1:400:end));
Z = V_d_tot(1:400:end);
for i = 1 :182
    Z = [Z; V_d_tot(1:400:end)];
end
surf(X,Y,Z)
%} 
